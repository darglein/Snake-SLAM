/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */


#include "saiga/core/util/table.h"
#include "saiga/vision/cameraModel/MotionModel.h"
#include "saiga/vision/util/HistogramImage.h"

//#include "LocalMapping/Hashmap.h"
#include "Map/MapPoint.h"
#include "Optimizer/LocalBundleAdjustment.h"
#include "Tracking.h"
#include "LocalMapping/LocalMapping.h"

namespace Snake
{
std::tuple<bool, Keyframe*> Tracking::TrackFine(FramePtr framep)
{
    SAIGA_ASSERT(last_keyframe && !last_keyframe->isBad());
    SAIGA_ASSERT(lastTrackedFrame != framep);


#if REF_SWITCH
    if (trackingReferenceSwitch.need_switch_fine)
    {
        reference_keyframe = framep->referenceKF();
    }
#endif

    SAIGA_ASSERT(framep->validPose);



    if (!framep->validPose && last_keyframe->frame->id == framep->id - 1)
    {
        // invalid coarse tracking and we already tried making the last valid frame to a kf
        return {false, nullptr};
    }

    Keyframe* kf                = nullptr;
    int inliersPoseOptimization = 0;
    {
        //        Saiga::ScopedTimer timer(stats.timeMS);
        //    SAIGA_BLOCK_TIMER();
        Frame& frame = *framep;

        if (framep->validPose)
        {
            updateLocalMap(frame);
            TEST_MAP_SYNC;
            inliersPoseOptimization = computePose(frame);
            TEST_MAP_SYNC;
        }
        frame.trackingInliers = inliersPoseOptimization;

        {
            TEST_MAP_SYNC;
            auto [need, cull] = NeedNewKeyframe(framep);
            if (need)
            {
                kf = CreateNewKeyFrame(framep, cull);
            }


            TEST_MAP_SYNC;
        }


        // We allow points with high innovation (considererd outliers by the Huber Function)
        // pass to the new keyframe, so that bundle adjustment will finally decide
        // if they are outliers or not. We don't want next frame to estimate its position
        // with those points so we discard them in the frame.
        for (auto i : frame.featureRange())
        {
            if (frame.mvpMapPoints[i] && frame.mvbOutlier[i]) frame.mvpMapPoints[i] = nullptr;
        }
        SAIGA_ASSERT(frame.referenceKF());


        TEST_MAP_SYNC;
    }


    if (framep->validPose)
    {
        lastTrackedFrame = framep;
    }

    return {inliersPoseOptimization > 0, kf};
}



void Tracking::updateLocalMap(Frame& frame)
{
    TEST_MAP_SYNC;
    // - All keyframes that have a reference to one of the previously matched points
    // - Some neighbours of these

    // All world points that are referenced from any of the 'local keyframes'
    //    TEST_MAP_SYNC;

    lm.clear();
    //        mvpLocalMapPoints.clear();
    {
        //        std::shared_lock lock(map.mutexUpdate);
        auto lock = map.LockReadOnly();


        // accesses observations and graph
        UpdateLocalKeyFrames2(frame);
        //            SAIGA_BLOCK_TIMER();
        // critical section for building the local map
        referencePose = reference_keyframe->PoseGlobal();

        if (lastTrackedFrame && frame.id - lastTrackedFrame->id == 1 && lastTrackedFrame->validPose)
        {
            lastPose = lastTrackedFrame->getPoseFromReference();
        }

        auto pose = frame.getPoseFromReference();
        frame.setPose(pose);
        frame.setReference(reference_keyframe);
        UpdateLocalPoints(frame);
    }
    //    stats.localMapSize = lm.points.size();


    //    outConsole << "fine TrackingFine" << std::endl;

    TEST_MAP_SYNC;


    //        auto additionalPoints = SearchLocalPoints(frame);
}

int Tracking::computePose(Frame& frame)
{
    int additionalPoints;
    int inliersPoseOptimization;
    {
        int radius = settings.inputType == InputType::Mono ? fine_projection_search_radius_mono
                                                           : fine_projection_search_radius_stereo;


        additionalPoints = localMapMatcher.SearchByProjection2(frame, lm, radius, 0.8, settings.num_tracking_threads);
        TEST_MAP_SYNC;


        //        std::cout << "fine " << std::endl;
        //        std::cout << "prediction " << frame.prediction << std::endl;
        //        std::cout << "tmpPose " << frame.tmpPose << std::endl;

        //        std::cout << "motion_prediction " << frame.motion_prediction << std::endl;
        inliersPoseOptimization = poseRefinement.refinePose(frame, lm);
        //        std::cout << "tmpPose " << frame.tmpPose << std::endl;
        TEST_MAP_SYNC;
    }

    if (lastTrackedFrame && frame.id - lastTrackedFrame->id == 1 && lastTrackedFrame->validPose)
    {
        // Convert last and current camera pose to body space

        auto p1_body       = lastPose.inverse() * mono_intrinsics.camera_to_body;
        auto p2_body       = frame.tmpPose.inverse() * mono_intrinsics.camera_to_body;
        auto velocity_body = p1_body.inverse() * p2_body;
        //        std::cout << "Add motion " << frame.id << std::endl;
        predictor.SetVisualVelocity(frame, velocity_body);
    }

    {
        TEST_MAP_SYNC;
        auto relToRef = frame.Pose() * referencePose.inverse();
        TEST_MAP_SYNC;
        frame.setRelToRef(relToRef);
        TEST_MAP_SYNC;
    }


    TEST_MAP_SYNC;


    // Update MapPoints Statistics
    for (auto i : frame.featureRange())
    {
        if (frame.mvpMapPoints[i])
        {
            if (!frame.mvbOutlier[i])
            {
                frame.mvpMapPoints[i]->IncreaseFound();
            }
            else
            {
                frame.mvpMapPoints[i] = nullptr;
            }
        }
    }
    TEST_MAP_SYNC;


    if (inliersPoseOptimization < fine_min_pose_optimziation_inliers)
    {
        return 0;
    }


    // TrackingFine was a success
    frame.validPose = true;



    TEST_MAP_SYNC;
    return inliersPoseOptimization;
}



void Tracking::UpdateLocalKeyFrames2(Frame& frame)
{
    int num_direct_kfs        = 15;
    int num_direct_prob_kfs   = 5;
    int num_indirect_prob_kfs = 5;

    tmp_keyframes.clear();
    local_keyframes.clear();

    for (auto i : frame.featureRange())
    {
        MapPoint* mp = frame.mvpMapPoints[i];
        if (!mp) continue;

        if (mp->isBad())
        {
            frame.mvpMapPoints[i] = nullptr;
            continue;
        }

        for (auto obs : mp->GetObservationList())
        {
            auto kf = obs.first;
            perKeyframeData[kf->id()].point_counter++;

            if (perKeyframeData[kf->id()].in_local_keyframes != frame.id)
            {
                perKeyframeData[kf->id()].in_local_keyframes = frame.id;
                tmp_keyframes.push_back(kf);
            }
        }
    }

    if (tmp_keyframes.empty()) return;



    count_keyframe.clear();
    for (auto kf : tmp_keyframes)
    {
        auto count                              = perKeyframeData[kf->id()].point_counter;
        perKeyframeData[kf->id()].point_counter = 0;
        count_keyframe.emplace_back(count, kf);
    }
    SAIGA_ASSERT(count_keyframe.size() == tmp_keyframes.size());

    std::sort(count_keyframe.begin(), count_keyframe.end());
    reference_keyframe = count_keyframe.back().second;


    // Insert the best 15 keyframes from the count
    for (int i = 0; i < std::min<int>(num_direct_kfs, tmp_keyframes.size()); ++i)
    {
        local_keyframes.push_back(count_keyframe.back().second);
        count_keyframe.pop_back();
    }



    double prob = num_direct_prob_kfs / double(count_keyframe.size());
    indirect_neighbor.clear();
    while (!count_keyframe.empty())
    {
        auto kf = count_keyframe.back().second;
        if (Random::sampleDouble(0, 1) < prob)
        {
            local_keyframes.push_back(kf);
        }
        else
        {
            // give it a second change as indirect
            indirect_neighbor.push_back(kf);
        }
        count_keyframe.pop_back();
    }


    // Collect neighbors of direct KFs and store them in indirect_neighbor.
    for (auto kf : local_keyframes)
    {
        auto tmp_keyframes = kf->GetBestCovisibilityKeyFrames(5);
        for (auto [count, neighbor_kf] : tmp_keyframes)
        {
            if (*neighbor_kf)
            {
                if (perKeyframeData[neighbor_kf->id()].in_local_keyframes != frame.id)
                {
                    indirect_neighbor.push_back(neighbor_kf);
                    perKeyframeData[neighbor_kf->id()].in_local_keyframes = frame.id;
                }
            }
        }
    }

    // Pick up to 'num_indirect_prob_kfs' keyframes from the neighbors and include them in the local keyframes.
    double prob_ind = num_indirect_prob_kfs / double(indirect_neighbor.size());
    for (auto kf : indirect_neighbor)
    {
        if (Random::sampleDouble(0, 1) < prob_ind)
        {
            local_keyframes.push_back(kf);
        }
    }
}



int Tracking::UpdateLocalPoints(Frame& frame)
{
    //    auto pinv = last_keyframe->PoseGlobal().inverse();
    //    lm.addPoints(newLocalPoints, pinv);


    for (auto kf : local_keyframes)
    {
        for (auto pMP : kf->GetMapPointMatches())
        {
            if (!pMP || pMP->isBad()) continue;
            if (pMP->lastFrameSeen == frame.id) continue;
            lm.addPoint(pMP);
        }
    }

    for (auto& mp : frame.mvpMapPoints)
    {
        if (!mp || mp->isBad()) continue;
        // add points from the current frame to the LM, but
        // do not search them (valid = false)
        mp->IncreaseVisible();
        //        SAIGA_ASSERT(mp->lastFrameSeen == frame.id);
        auto id = lm.addPoint(mp);
        SAIGA_ASSERT(id != -1);
        lm.points[id].valid = false;
    }
    return lm.points.size();
}

}  // namespace Snake
