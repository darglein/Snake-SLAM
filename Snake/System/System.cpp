/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */
#include "System.h"

#include "saiga/core/image/png_wrapper.h"
#include "saiga/vision/reconstruction/VoxelFusion.h"
#include "saiga/vision/util/Ransac.h"

#include "IMU/ImuStateSolver.h"
#include "Optimizer/GlobalBundleAdjustment.h"
#include "Optimizer/LocalBundleAdjustment.h"
#include "Preprocess/FeatureDetector.h"
#include "Preprocess/Preprocess.h"
#include "Viewer/SnakeOpenGLViewer.h"

#include <filesystem>
namespace Snake
{
System::System(const Settings& _settings)
    : System(_settings,
             [&]()
             {
                 return _settings.viewer_settings.enabled
                            ? make_aligned_unique<SnakeOpenGLViewer>(_settings.viewer_settings, _settings.config_file)
                            : std::unique_ptr<SnakeOpenGLViewer>();
             })
{
}

System::System(const Settings& _settings, std::function<std::unique_ptr<ViewerInterface>(void)> create_viewer)

{
    settings = _settings;
    settings.Print();
    ransacRandomSeed = settings.randomSeed;
    Saiga::Random::setSeed(ransacRandomSeed);

    pause = settings.start_paused;

    {
        vocabulary.loadRaw(settings.voc_file);
        SAIGA_ASSERT(vocabulary.size() > 0);
        keyFrameDB = std::make_unique<KeyframeDatabase>();
    }

    {
        auto u_ptr = std::make_unique<Input>();
        input      = u_ptr.get();
        modules.push_back(std::move(u_ptr));
    }
    {
        auto u_ptr      = std::make_unique<FeatureDetector>();
        featureDetector = u_ptr.get();
        modules.push_back(std::move(u_ptr));
    }
    {
        auto u_ptr = std::make_unique<Preprocess>();
        preprocess = u_ptr.get();
        modules.push_back(std::move(u_ptr));
    }
    {
        auto u_ptr = std::make_unique<Tracking>();
        tracking   = u_ptr.get();
        modules.push_back(std::move(u_ptr));
    }
    {
        auto u_ptr   = std::make_unique<LocalMapping>();
        localMapping = u_ptr.get();
        modules.push_back(std::move(u_ptr));
    }
    {
        auto u_ptr = std::make_unique<LocalBundleAdjustment>();
        lba        = u_ptr.get();
        modules.push_back(std::move(u_ptr));
    }
    {
        auto u_ptr = std::make_unique<GlobalBundleAdjustment>();
        gba        = u_ptr.get();
        modules.push_back(std::move(u_ptr));
    }
    {
        auto u_ptr  = std::make_unique<LoopClosing>();
        loopClosing = u_ptr.get();
        modules.push_back(std::move(u_ptr));
    }
    {
        auto u_ptr     = std::make_unique<Simplification>();
        simplification = u_ptr.get();
        modules.push_back(std::move(u_ptr));
    }
    {
        auto u_ptr     = std::make_unique<DeferredMapper>();
        deferredMapper = u_ptr.get();
        modules.push_back(std::move(u_ptr));
    }


#if WITH_IMU
    {
        auto u_ptr       = std::make_unique<ImuStateSolver>();
        imu_state_solver = u_ptr.get();
        modules.push_back(std::move(u_ptr));
    }
#endif

    createGlobalThreadPool(4);

    if (settings.viewer_settings.enabled)
    {
        std::atomic_bool initDone = false;
        viewerThread              = ScopedThread(
            [&]()
            {
                Saiga::setThreadName("Viewer");
                // The Thread creating and running the GL app must be the same
                viewer = create_viewer();

                viewer->setSnakeImguiFunction([this]() { imgui(); });
                initDone = true;
                viewer->run();
                quit();

                while (running)
                {
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
                // This thread also has to delete the viewer because of OpenGL calls
                viewer = nullptr;
            });

        while (!initDone)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        Snake::viewer = viewer.get();
    }

    std::cout << ConsoleColor::GREEN << "All Modules Initialized. Starting Camera Tracking..." << ConsoleColor::RESET
              << std::endl;
}

System::~System()
{
    keyFrameDB.reset();
}

void System::run()
{
    running = true;

    input->run();
    tracking->run();

    tracking->waitForFinish();

    std::cout << "System: Tracking Finished!" << std::endl;


    loopClosing->Join();
    simplification->Join();
    lba->Join();
    deferredMapper->Join();

    {
        auto lock = map.LockReadOnly();
        if (map.MaxNumberOfKeyframes() > 0)
        {
            // mark last frew frames as bad by increase the culling probability
            auto last_kf = &map.getKeyframe(map.MaxNumberOfKeyframes() - 1);
            Keyframe* kf = last_kf;
            while (kf && kf->frame->id > last_kf->frame->id - 30)
            {
                kf->cull_factor = 5;
                kf              = kf->previousKF;
                simplification->Add(kf);
            }
        }
    }

    deferredMapper->ForceCleanQueue();
    simplification->ForceCleanQueue();



    std::cout << "System: All Modules Terminated!" << std::endl;

    gba->FullBA(5);


    if (has_imu)
    {
        imu_state_solver->IterateBaImu(10);
    }
    else
    {
        gba->FullBA(10);
    }

    {
        auto lock = map.LockFull();
        map.removeOutliers(0.75 * reprojectionErrorThresholdMono2, 0.75 * reprojectionErrorThresholdStereo2);
    }
    gba->FullBA(10);

    if (settings.inputType == InputType::RGBD)
    {
        gba->RealignIntermiediateFrames(true, true);
        gba->RealignIntermiediateFrames(true, true);
        RematchIntermiediate();
        gba->RealignIntermiediateFrames(true, true);
        gba->RealignIntermiediateFrames(true, true);
    }


    writeFrameTrajectory(settings.evalDir + "/frames/" + settings.out_file_prefix + "_frames_ba.tum");
    writeKeyFrameTrajectory(settings.evalDir + settings.out_file_prefix + "_keyframes_ba.tum");

    performance_stats.PrintStatistics();
    performance_stats.PrintTimings();

    if (settings.inputType == InputType::RGBD)
    {
        DepthProcessor2::Settings depth_settings = {0.0f, 2, 7.f, 9.f};
        depth_settings.cameraParameters = StereoCamera4(rgbd_intrinsics.depthModel.K, rgbd_intrinsics.bf).cast<float>();

        auto lock   = map.LockFull();
        auto frames = map.GetAllFrames();


        DepthProcessor2 dp(depth_settings);
        ProgressBar loadingBar(std::cout, "Preprocess Depth", frames.size());
#pragma omp parallel for
        for (int i = 0; i < (int)frames.size(); ++i)
        {
            dp.Process(frames[i]->depth_image.getImageView());
            loadingBar.addProgress(1);
        }
    }



    if (viewer)
    {
        // Update the viewer one last time.
        viewer->setMap(std::make_unique<ViewerMap>());
        viewer->InputFinished();
    }

    running  = false;
    finished = true;

    // Wait until viewer is closed.
    if (viewerThread.joinable())
    {
        viewerThread.join();
    }
}

void System::quit()
{
    pause = false;
    // quitting the input system will quit all other modules as well
    input->quit();
}

void System::RematchIntermiediate()
{
    std::vector<FramePtr> frames;
    {
        auto lock = map.LockReadOnly();
        frames    = map.GetAllFrames();
    }
    ProgressBar bar(std::cout, "RematchIntermiediate", frames.size());
    for (auto frame : frames)
    {
        if (!frame->validPose || frame->isKeyframe) continue;
        frame->trackingInliers = 0;

        tracking->updateLocalMap(*frame);
        frame->trackingInliers = tracking->computePose(*frame);


        frame->prediction_weight_translation = 0;
        frame->prediction_weight_rotation    = 0;


        //            int bef = frame->trackingInliers;
        int bef = 0;
        for (auto i : frame->featureRange())
        {
            if (frame->mvpMapPoints[i] && !frame->mvpMapPoints[i]->isBad()) bef++;
        }



        //        int aft = tracking->computePose(*frame);
        //        std::cout << *frame << " " << bef << " -> " << aft << std::endl;
        bar.addProgress(1);
    }
}

void System::SaveSublonet()
{
    std::string out_dir = "/ssd2/slam/saiga/sublo/";
    std::filesystem::create_directory(out_dir);
    std::filesystem::create_directory(out_dir + "/data/");

    auto frames = map.GetAllFrames();

    int i = 0;
    for (auto f : frames)
    {
        auto full_prefix = out_dir + "/data/" + std::to_string(i);
        auto depth_file  = full_prefix + ".depth.png";
        auto extr_file   = full_prefix + ".extrinsics.txt";
        auto intr_file   = full_prefix + ".intrinsics.txt";
        auto dis_file    = full_prefix + ".distortion.txt";


        auto& depth = f->depth_image;
        TemplatedImage<unsigned short> depth_short(depth.dimensions());
        for (auto k : depth.rowRange())
        {
            for (auto l : depth.colRange())
            {
                float& df = depth(k, l);
                auto d    = df * 5000.f;
                SAIGA_ASSERT(d < (1 << 16));
                depth_short(k, l) = iRound(d);
            }
        }
        depth_short.save(depth_file);


        {
            std::ofstream strm(extr_file);
            strm << std::setprecision(20);
            Mat4 V = f->Pose().matrix();
            for (int i = 0; i < 4; ++i)
            {
                for (int j = 0; j < 4; ++j)
                {
                    strm << V(i, j) << " ";
                }
                strm << std::endl;
            }
        }

        {
            std::ofstream strm(intr_file);
            strm << std::setprecision(20);
            Mat3 K1 = rgbd_intrinsics.depthModel.K.matrix();
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                {
                    strm << K1(i, j) << " ";
                }
                strm << std::endl;
            }
        }

        {
            std::ofstream strm(dis_file);
            strm << std::setprecision(20);
            Eigen::Matrix<double, 8, 1> dis = rgbd_intrinsics.depthModel.dis.Coeffs();

            for (int j = 0; j < dis.rows(); ++j)
            {
                strm << dis(j) << " ";
            }
        }


        ++i;
    }
}


void System::imgui()
{
    if (ImGui::Begin("System"))
    {
        static bool pa = pause;
        if (ImGui::Checkbox("pause", &pa))
        {
            pause = pa;
        }

        if (ImGui::Button("Crazy move"))
        {
            map.crazyMove();
        }

        if (ImGui::Button("Global BA"))
        {
            gba->FullBA(1, false);
            localMapping->UpdateViewer();
        }

        if (ImGui::Button("Global BA Rel"))
        {
            gba->FullBARel(1, false);
            localMapping->UpdateViewer();
        }

        if (ImGui::Button("RealignIntermiediateFrames imu"))
        {
            gba->RealignIntermiediateFrames(true, true);
            localMapping->UpdateViewer();
        }
        if (ImGui::Button("RealignIntermiediateFrames"))
        {
            gba->RealignIntermiediateFrames(true, false);
            localMapping->UpdateViewer();
        }

        if (ImGui::Button("Rematch intermiediate frames"))
        {
            RematchIntermiediate();
        }

        if (ImGui::Button("printTrajectory"))
        {
            map.printTrajectory(true);
        }


        if (ImGui::Button("Write Frames To File"))
        {
            writeFrameTrajectory("Frames.txt");
        }

        if (ImGui::Button("Write Frames To File with id"))
        {
            writeFrameTrajectory("Frames.txt", true);
        }


        map.imgui();



#if WITH_IMU
        imu_state_solver->solver.imgui();

        if (ImGui::Button("Init Gyro"))
        {
            imu_state_solver->InitGyroBias();
            localMapping->UpdateViewer();
        }
        if (ImGui::Button("InitGravityAndScale"))
        {
            imu_state_solver->InitGravityAndScale();
            localMapping->UpdateViewer();
        }


        if (ImGui::Button("Solve Scale"))
        {
            imu_state_solver->solver.Solve(1, 1, 1, 1, 1, false);
            imu_state_solver->RecomputeWeights();
        }
        if (ImGui::Button("Solve imu"))
        {
            imu_state_solver->solver.Solve(1, 1, 1, 1, 0, false);
            imu_state_solver->RecomputeWeights();
        }

        ImGui::InputDouble("current_gyro_weight", &current_gyro_weight);
        ImGui::InputDouble("current_acc_weight", &current_acc_weight);
#endif



        if (ImGui::Button("Save Scene"))
        {
            std::string out_dir = "scenes/" + CurrentTimeString("%Y_%d_%m_%H_%M_%S");
            SAIGA_ASSERT(!std::filesystem::exists(out_dir));
            std::filesystem::create_directory(out_dir);
            SAIGA_ASSERT(std::filesystem::exists(out_dir));

            std::string frame_out_dir = out_dir + "/frames/";
            std::filesystem::create_directory(frame_out_dir);

            auto intr = rgbd_intrinsics;
            intr.fromConfigFile(out_dir + "/camera.ini");

            auto frames = map.GetAllFrames();

            std::cout << "Saving scene to " << out_dir << std::endl;
            std::cout << "First frame " << *frames.front() << " t = " << frames.front()->timeStamp << std::endl;

            ProgressBar loadingBar(std::cout, "Saving", frames.size());

#pragma omp parallel for num_threads(4)
            for (int i = 0; i < frames.size(); ++i)
            {
                auto f = frames[i];



                auto str       = Saiga::leadingZeroString(i, 5);
                auto frame_dir = frame_out_dir + "/" + str + "/";
                std::filesystem::create_directory(frame_dir);
                frames[i]->Save(frame_dir);


                if (f->validPose)
                {
                    std::ofstream ostream(frame_dir + "/view_pose.txt");
                    ostream << frames[i]->Pose().params().transpose() << std::endl;
                }
                loadingBar.addProgress(1);
            }
        }
    }
    ImGui::End();

    for (auto& m : modules)
    {
        m->imgui();
    }
}  // namespace Snake

void System::writeFrameTrajectory(const std::string& file, bool write_id)
{
    //   - collect all frames using the prev pointer and starting at the last keyframe
    std::vector<int> keyframesi;
    map.GetAllKeyFrames(keyframesi);

    if (keyframesi.empty()) return;

    std::vector<FramePtr> snake_frames;
    auto c = map.getKeyframe(keyframesi.back()).frame;
    while (c)
    {
        if (c->validPose) snake_frames.push_back(c);
        c = c->previousFrame;
    }
    std::reverse(snake_frames.begin(), snake_frames.end());

    std::ofstream strm(file);
    strm.precision(15);
    for (auto f : snake_frames)
    {
        if (f->validPose)
        {
            double time = f->timeStamp;
            SE3 pose    = f->tmpPose.inverse();
            Vec3 t      = pose.translation();
            Quat q      = pose.unit_quaternion();

            if (write_id)
            {
                strm << f->id << " ";
            }

            strm << time << " " << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " " << q.z()
                 << " " << q.w() << std::endl;
        }
    }
}

void System::writeKeyFrameTrajectory(const std::string& file)
{
    std::vector<int> keyframesi;
    map.GetAllKeyFrames(keyframesi);

    if (keyframesi.empty()) return;

    std::ofstream strm(file);
    strm.precision(15);

    for (auto i : keyframesi)
    {
        auto& kf = map.getKeyframe(i);
        if (kf.isBad()) continue;

        double time = kf.frame->timeStamp;
        SE3 pose    = kf.PoseInv();
        Vec3 t      = pose.translation();
        Quat q      = pose.unit_quaternion();
        strm << time << " " << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " " << q.z() << " "
             << q.w() << std::endl;
    }
}



}  // namespace Snake
