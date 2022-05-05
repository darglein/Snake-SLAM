/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

//#define ENABLE_MAP_SYNC_TEST

#include "Tracking.h"

#include "saiga/core/image/ImageDraw.h"
#include "saiga/core/imgui/imgui.h"
#include "saiga/vision/cameraModel/MotionModel.h"

#include "Initialization/MonoInitializer.h"
#include "Initialization/RGBDInitializer.h"
#include "LocalMapping/LocalMapping.h"
#include "Map/MapPoint.h"
#include "Preprocess/Input.h"
#include "Preprocess/Preprocess.h"
namespace Snake
{
Tracking::Tracking() : Module(ModuleType::TRACKING), poseRefinement(trackingErrorFactorCoarse)
{
    perKeyframeData.resize(maxKeyframes);
    mnTrackReferenceForFrame.resize(maxKeyframes);
    createInitializer();
    CreateTable({7, 10, 10, 10}, {"Frame", "Inl. 1", "Inl. 2", "KF"});
}

Tracking::~Tracking()
{
    auto t = trackTimer.getTimeMS() / 1000.0;
    std::cout << ConsoleColor::BLUE << "SLAM finished. It took " << t << "s for " << trackFrame << " frames -> "
              << 1000.0 * t / trackFrame << "ms/frame -> " << trackFrame / t << "fps." << std::endl
              << ConsoleColor::RESET;
}

void Tracking::run()
{
    thread = Thread([this]() {
        Saiga::Random::setSeed(settings.randomSeed);
        Saiga::setThreadName("Tracking");

        while (true)
        {
            auto frame = preprocess->GetFrame();
            if (!frame)
            {
                break;
            }


            while (is_dataset && pause)
            {
                localMapping->UpdateViewer();
                std::this_thread::sleep_for(std::chrono::milliseconds(30));
            }

            while (tracking_barrier)
            {
                at_barrier = true;
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
            at_barrier = false;

            if (trackFrame++ == 0)
            {
                trackTimer.start();
            }

            predictor.Predict(frame);


            SAIGA_ASSERT(predictor.last_keyframe == last_keyframe);



            if (!pause)
            {
                Keyframe* new_kf;
                {
                    auto timer = ModuleTimer();
                    new_kf     = track(frame);
                }

                if (new_kf)
                {
                    localMapping->Process(new_kf);
                    reference_keyframe = new_kf;
                }
            }


            auto free_image_data = [frame]() {
                if (!settings.keep_all_frame_data)
                {
                    frame->image.free();
                    frame->image_rgb.free();
                    frame->right_image.free();
                    frame->right_image_rgb.free();

                    if (!settings.keep_valid_depth_image || !frame->validPose)
                    {
                        frame->depth_image.free();
                    }
                }
            };

            if (viewer)
            {
                static std::future<void> result;
                result = std::future<void>();
                result = globalThreadPool->enqueue([frame, free_image_data]() {
                    viewer->setFrame(std::make_unique<ViewerFrame>(frame));
                    free_image_data();
                });
            }
            else
            {
                free_image_data();
            }



            if (map_clear)
            {
                map.Clear();
                state              = State::NOT_INITIALIZED;
                last_keyframe      = nullptr;
                reference_keyframe = nullptr;
                lastTrackedFrame   = nullptr;
                createInitializer();
                map_clear = false;
                localMapping->UpdateViewer();
            }
        }
        trackTimer.stop();
    });
}

Keyframe* Tracking::track(FramePtr frame)
{
    bool frameState = false;


    Keyframe* new_kf = nullptr;

    if (state == State::NOT_INITIALIZED)
    {
        auto res = initializer->Initialize(frame);

        if (res.success)
        {
            last_keyframe      = res.kfLast;
            reference_keyframe = res.kfLast;
            predictor.SetLastKeyframe(last_keyframe);
            lastTrackedFrame = frame;
            state            = State::OK;

            res.kfFirst->frame->previousFrame = nullptr;
        }
    }
    else if (state == State::OK || state == State::RECOVERING)
    {
        frameState = TrackCoarse(frame);

        auto inliers_coarse = frame->trackingInliers;
        if (frameState)
        {
            // used for keyframe decision so we don't insert keyframes shortly after broken tracking
            SAIGA_ASSERT(frame->referenceKF());
            auto fine_result = TrackFine(frame);
            frameState       = std::get<0>(fine_result);
            new_kf           = std::get<1>(fine_result);
        }
        auto inliers_fine = frame->trackingInliers;

        (*output_table) << frame->id << inliers_coarse << inliers_fine << (new_kf ? new_kf->id() : 0);



        if (frameState)
        {
            if (state == State::RECOVERING)
            {
                //                std::cout << "Recovery Success at frame " << frame->id << std::endl;
                std::cout << ">> " << *frame << ": Tracking Recovered. Inliers: " << frame->trackingInliers
                          << std::endl;
            }
            state = State::OK;
            validInARow++;
        }
        else
        {
            validInARow = 0;
        }


        if (!frameState)
        {
            // the state is still ok so this is the first lost frame
            if (state == State::OK)
            {
                std::cout << ">> " << *frame << ": Tracking Lost." << std::endl;
                int num_kfs = map.KeyFramesInMap();
                if (num_kfs < 20)
                {
                    std::cout << ">> " << *frame << ": Clear Map, Num Keyframes: " << num_kfs << std::endl;
                    // Tracking lost right after initialization.
                    // -> clear complete map and initialize again
                    map_clear = true;
                }
                else
                {
                    // set recovering for a few frames
                    state          = State::RECOVERING;
                    recoverCounter = MAX_RECOVER_FRAMES;


                    // mark last frew frames as bad by increase the culling probability
                    Keyframe* kf = last_keyframe;
                    while (kf && kf->frame->id > frame->id - 30)
                    {
                        kf->cull_factor = 2;
                        kf              = kf->previousKF;
                    }
                    simplification->Add(last_keyframe);

                    // TEST_MAP_SYNC;
                }
            }
            else
            {
                // we are already in recovering and still didn't found a valid pose
                recoverCounter--;
                if (recoverCounter == 0)
                {
                    std::cout << ">> " << *frame << ": Starting Relocalization." << std::endl;
                    state = State::RELOCALIZE_AND_INITIALIZE;
                    createInitializer();
                }
            }
        }
    }
    else if (state == State::RELOCALIZE_AND_INITIALIZE)
    {
        frameState = try_localize(frame);
        // let's go first into recover mode to stabilize the pose estimate
        if (frameState)
        {
            // reloc successfull
            auto fine_state = TrackFine(frame);
            if (std::get<0>(fine_state))
            {
                validInARow    = 1;
                state          = State::RECOVERING;
                recoverCounter = MAX_RECOVER_FRAMES;

                new_kf = std::get<1>(fine_state);
                //                SAIGA_ASSERT(!fineState.keyframeCreated);
            }
            // TEST_MAP_SYNC;
        }
    }
    else
    {
        SAIGA_EXIT_ERROR("unknown state");
    }


    return new_kf;
}

void Tracking::createInitializer()
{
    switch (settings.inputType)
    {
        case InputType::RGBD:
            initializer = make_aligned_shared<RGBDInitializer>(settings.initialization_quality);
            break;
        case InputType::Mono:
            initializer = make_aligned_shared<MonoInitializer>(settings.initialization_quality);
            break;
        case InputType::Stereo:
            initializer = make_aligned_shared<RGBDInitializer>(settings.initialization_quality);
            break;
        default:
            SAIGA_EXIT_ERROR("No Initializer found for given inputtype.");
    }
}


}  // namespace Snake
