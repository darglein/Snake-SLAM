/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */


#include "saiga/core/image/ImageDraw.h"
#include "saiga/core/util/table.h"
#include "saiga/vision/cameraModel/MotionModel.h"
#include "saiga/vision/reconstruction/P3P.h"

//#include "ImuIntegration.h"
#include "LoopClosing/KeyframeDatabase.h"
#include "Map/LocalMap.h"
#include "Map/MapPoint.h"
#include "Tracking.h"
#include "LocalMapping/LocalMapping.h"
namespace Snake
{
void Tracking::PredictFramePosition(FramePtr frame) {}
void Tracking::BuildCoarseLocalMap(FramePtr frame)
{
#if REF_SWITCH
    if (trackingReferenceSwitch.need_switch_coarse)
    {
#    if 1
        //        SAIGA_EXIT_ERROR("sdf");
        //        std::shared_lock lock(map.mutexUpdate);
        auto lock = map.LockReadOnly();

        reference_keyframe = trackingReferenceSwitch.target_keyframe;

        SE3 target_kf_pose = trackingReferenceSwitch.target_keyframe->Pose();
        SE3 new_source_global =
            inverseMatchingSE3(trackingReferenceSwitch.source_to_target.inverse() * DSim3(target_kf_pose, 1.0));

        // global last pose
        lastPose = lastTrackedFrame->getPoseFromReference();

        // local in source space
        SE3 last_in_source = lastPose * trackingReferenceSwitch.source_keyframe->PoseGlobal().inverse();


        SE3 last_in_new_source = last_in_source;
        last_in_new_source.translation() *= trackingReferenceSwitch.source_to_target.scale();

        SE3 new_last = last_in_new_source * new_source_global;


        lastPose       = new_last;
        frame->tmpPose = lastPose;


        // clear if it is a retrack frame
        frame->clearMatches();
        predictor.motion_model_body->clear();

        lastTrackedFrame->validPose = false;
        lastTrackedFrame            = nullptr;
        validInARow                 = 1;
#    endif
        trackingReferenceSwitch.need_switch_coarse = false;
    }
#endif

    {
        //            std::shared_lock lock(map.mutexUpdate);
        auto lock = map.LockReadOnly();

        // ============== 1. Setup Reference Frame and predict new frame pose ==============

        while (reference_keyframe->isBad())
        {
            // the reference was removed. (from culling)
            // just use parent for now
            reference_keyframe = reference_keyframe->GetParent();
            SAIGA_ASSERT(reference_keyframe);
        }
        SAIGA_ASSERT(reference_keyframe && !reference_keyframe->isBad());



        // critical section for map computation
        referencePose = reference_keyframe->PoseGlobal();


        PredictFramePosition(frame);



        // ============== 2. Create local map ==============

        last_frame_lm.clear();
#if 1
        if (lastTrackedFrame && settings.inputType != InputType::Mono)
        {
            // === 2.1 with points seen in last tracked frame
            auto f = lastTrackedFrame;
            for (int i : f->featureRange())
            {
                auto mp = f->mvpMapPoints[i];
                if (!mp || f->mvbOutlier[i]) continue;
                int idx                              = last_frame_lm.addPoint(mp);
                last_frame_lm.points[idx].octave     = f->undistorted_keypoints[i].octave;
                last_frame_lm.points[idx].angle      = f->undistorted_keypoints[i].angle;
                last_frame_lm.points[idx].descriptor = f->descriptors[i];
            }
        }
#endif


#if 1
        // === 2.2 with points seen in reference keyframe
        for (int i : last_keyframe->frame->featureRange())
        {
            auto mp = last_keyframe->GetMapPoint(i);
            if (!mp || mp->isBad()) continue;
            int idx = last_frame_lm.addPoint(mp);
            if (idx + 1 == last_frame_lm.points.size())
            {
                last_frame_lm.points[idx].octave     = last_keyframe->frame->undistorted_keypoints[i].octave;
                last_frame_lm.points[idx].angle      = last_keyframe->frame->undistorted_keypoints[i].angle;
                last_frame_lm.points[idx].descriptor = last_keyframe->frame->descriptors[i];
            }
        }
#endif

#if 0

            // === 2.3 local stereo points from last keyframe
            for (auto&& np : referenceKF->newStereoPoints)
            {
                //                auto& i  = np.first;
                auto& mp = map.getMapPoint(np.second);
                SAIGA_ASSERT(mp.debugIsLocal);
                SAIGA_ASSERT(mp.referenceKF == referenceKF);
                int idx = last_frame_lm.addPoint(&mp);
                if (idx + 1 == last_frame_lm.points.size())
                {
                    last_frame_lm.points[idx].octave     = referenceKF->frame->undistorted_keypoints[np.first].octave;
                    last_frame_lm.points[idx].angle      = referenceKF->frame->undistorted_keypoints[np.first].angle;
                    last_frame_lm.points[idx].descriptor = referenceKF->frame->descriptors[np.first];
                }
            }
#endif
    }
}

bool Tracking::TrackCoarse(FramePtr frame)
{
    bool tracking_result = false;
    {
        BuildCoarseLocalMap(frame);
        TEST_MAP_SYNC;



        TEST_MAP_SYNC;

        frame->clearMatches();
        auto radius = settings.inputType == InputType::Mono ? coarse_projection_search_radius_mono
                                                            : coarse_projection_search_radius_stereo;
        tracking_result = TrackWithPrediction(*frame, radius);

        //        std::cout << "[TrackWithPrediction] " << *frame << " " << frame->trackingInliers << std::endl;

        TEST_MAP_SYNC;

        if (!tracking_result)
        {
            frame->clearMatches();
            frame->tmpPose  = lastPose;
            tracking_result = TrackBruteForce(*frame, reference_keyframe, coarse_min_inliers_last_keyframe, true);
            TEST_MAP_SYNC;
        }


        if (tracking_result)
        {
            frame->validPose = true;
            TEST_MAP_SYNC;
        }
        TEST_MAP_SYNC;
    }



    return tracking_result;
}


bool Tracking::TrackWithPrediction(Frame& frame, float radius)
{
    //    std::cout << "track " << frame << std::endl;
    // Try to match all points that were found in lastframe to the current frame
    int projectionMatches;

    // Feature threshold
#ifdef SNAKE_OS_MODE
    auto feature_th = 100;
#else
    auto feature_th = 75;
#endif


#if 0
    if (false && gyro_initialized)
    {
        // test
        frame.clearMatches();

        frame.tmpPose = frame.prediction;
        auto m1       = localMapMatcher.SearchByProjectionFrameFrame2(frame, last_frame_lm, radius, feature_th,
                                                                settings.num_tracking_threads);

        frame.clearMatches();
        frame.tmpPose = frame.motion_prediction;
        auto m2       = localMapMatcher.SearchByProjectionFrameFrame2(frame, last_frame_lm, radius, feature_th,
                                                                settings.num_tracking_threads);

        frame.clearMatches();

        if (m1 != m2)
        {
            std::cout << frame.prediction << std::endl;
            std::cout << frame.motion_prediction << std::endl;
            std::cout << "Gyro: " << m1 << " Motion " << m2 << std::endl;
            std::cout << std::endl;
        }
    }
#endif
    // orbslam is 100 mine was preiviously 75
    projectionMatches = localMapMatcher.SearchByProjectionFrameFrame2(frame, last_frame_lm, radius, feature_th,
                                                                      settings.num_tracking_threads);

    //    SAIGA_ASSERT(projectionMatches > 20);
#if WITH_IMU345
    if (projectionMatches < 50)
    {
        std::cout << "num points " << last_frame_lm.points.size() << std::endl;
        std::cout << referencePose << std::endl;
        std::cout << frame.tmpPose << std::endl;
        std::cout << "matches: " << projectionMatches << std::endl;
    }
#endif
    //    projectionMatches = localMapMatcher.SearchByProjectionFrameFrameOmp(frame, last_frame_lm, radius, feature_th);
    TEST_MAP_SYNC;

    if (projectionMatches < coarse_min_inliers_last_frame)
    {
        auto additional_matches = localMapMatcher.SearchByProjectionFrameFrame2(
            frame, last_frame_lm, 2 * radius, feature_th, settings.num_tracking_threads);
        projectionMatches += additional_matches;
        TEST_MAP_SYNC;
    }

    if (projectionMatches < coarse_min_inliers_last_frame)
    {
        std::cout << frame << " TrackWithPrediction only " << projectionMatches << " inliers." << std::endl;
        return false;
    }


    TEST_MAP_SYNC;



    int inliersPoseOptimization;
    inliersPoseOptimization = poseRefinement.refinePose(frame, last_frame_lm);

    TEST_MAP_SYNC;


    if (inliersPoseOptimization < coarse_min_inliers_last_frame)
    {
        std::cout << frame << " refinePose only " << inliersPoseOptimization << "/" << projectionMatches
                  << " inliers. Weights: " << frame.prediction_weight_rotation << "/"
                  << frame.prediction_weight_translation << std::endl;
        return false;
    }

    //    depth_vector.clear();

    auto matches = projectionMatches;
    for (int i = 0; i < frame.N; ++i)
    {
        if (!frame.mvpMapPoints[i]) continue;

        MapPoint* mp = frame.mvpMapPoints[i];
        if (frame.mvbOutlier[i])
        {
            frame.mvpMapPoints[i] = nullptr;
            frame.mvbOutlier[i]   = false;
            matches--;
        }
        else if (*mp)
        {
            auto lid = last_frame_lm.localIndex(mp);
            auto z   = (frame.tmpPose * last_frame_lm.points[lid].position).z();

            if (z <= 0)
            {
                frame.mvpMapPoints[i] = nullptr;
                matches--;
                continue;
            }

            //            depth_vector.push_back(z);
            mp->lastFrameSeen = frame.id;
        }
    }

    TEST_MAP_SYNC;


    TEST_MAP_SYNC;
    auto relToRef = frame.tmpPose * referencePose.inverse();
    TEST_MAP_SYNC;
    frame.setReference(reference_keyframe, relToRef);
    TEST_MAP_SYNC;


    // Add to motion model, if last frame is valid and they next to each other
    if (lastTrackedFrame && frame.id - lastTrackedFrame->id == 1)
    {
        // Convert last and current camera pose to body space
        auto p1_body       = lastPose.inverse() * mono_intrinsics.camera_to_body;
        auto p2_body       = frame.tmpPose.inverse() * mono_intrinsics.camera_to_body;
        auto velocity_body = p1_body.inverse() * p2_body;
        //        motion_model_body->addRelativeMotion(velocity_body, frame.id);
        predictor.SetVisualVelocity(frame, velocity_body);
        TEST_MAP_SYNC;
    }

    frame.trackingInliers = inliersPoseOptimization;

    TEST_MAP_SYNC;

    return true;
}

bool Tracking::TrackBruteForce(Frame& frame, Keyframe* ref, int min_matches, bool has_local_map)
{
    frame.prediction_weight_rotation *= 0.01;
    frame.prediction_weight_translation *= 0.01;


    TEST_MAP_SYNC;
    BruteForceMatcher<FeatureDescriptor> matcher;
    matcher.matchKnn2_omp(frame.descriptors, ref->frame->descriptors, settings.num_tracking_threads);
    auto matches = matcher.filterMatches(60, 0.8);


    TEST_MAP_SYNC;
    if (matches < 30)
    {
        return false;
    }

    std::vector<std::pair<int, int>> matchesIds;
    AlignedVector<Vec3> wps;
    AlignedVector<Vec2> ips;

    SE3 reference_pose;

    TEST_MAP_SYNC;
    {
        //        std::shared_lock lock(map.mutexUpdate);
        auto lock        = map.LockReadOnly();
        int matchesInMap = 0;
        // find all matches that are also currently in the map
        for (auto m : matcher.matches)
        {
            auto mp = ref->GetMapPoint(m.second);
            if (mp && *mp)
            {
                frame.mvpMapPoints[m.first] = mp;
                matchesIds.push_back(m);
                wps.push_back(mp->getPosition());


                auto ip  = frame.undistorted_keypoints[m.first].point;
                auto nip = mono_intrinsics.model.K.unproject2(ip);
                ips.push_back(nip);
                matchesInMap++;
            }
        }
        //        stats.localMapMatches = matchesInMap;
        //        trackingConsole << "Map: " << matchesInMap << " ";

        reference_pose = ref->Pose();

        if (matchesInMap < 20)
        {
            return false;
        }
    }
    TEST_MAP_SYNC;



    std::vector<int> inlierMatches;
    std::vector<char> inlierMask;
    SE3 pose;

    int inliers;

    {
        RansacParameters params;
        params.maxIterations     = 250;
        auto chi1                = 2 * reprojectionErrorThresholdMono / K.fx;
        params.residualThreshold = chi1 * chi1;
        params.threads           = settings.num_tracking_threads;

        P3PRansac pnp2(params);
#pragma omp parallel num_threads(settings.num_tracking_threads)
        {
            inliers = pnp2.solve(wps, ips, pose, inlierMatches, inlierMask);
        }
        frame.tmpPose = pose;
    }


    TEST_MAP_SYNC;
    auto relToRef = pose * reference_pose.inverse();
    TEST_MAP_SYNC;
    frame.setReference(ref, relToRef);
    TEST_MAP_SYNC;

    SAIGA_ASSERT((int)inlierMatches.size() == inliers);

    auto cpy = frame.mvpMapPoints;
    for (auto& m : frame.mvpMapPoints) m = nullptr;

    for (auto i : inlierMatches)
    {
        int j                 = matchesIds[i].first;
        frame.mvpMapPoints[j] = cpy[j];
    }
    std::fill(frame.mvbOutlier.begin(), frame.mvbOutlier.end(), false);


    TEST_MAP_SYNC;
    {
        //        std::shared_lock lock(map.mutexUpdate);
        auto lock = map.LockReadOnly();

        reference_pose = ref->Pose();
        frame.setPose(frame.getPoseFromReference());

        inliers = poseRefinement.RefinePoseWithMatches(frame);
        if (inliers < 10)
        {
            return false;
        }

        // search a few more points by projection

        if (has_local_map && ref == reference_keyframe)
        {
            // relative pose to local map
            auto rel      = frame.tmpPose * reference_pose.inverse();
            frame.tmpPose = rel * referencePose;

            localMapMatcher.SearchByProjectionFrameFrame2(frame, last_frame_lm, 5, 75, settings.num_tracking_threads);



            // back to new local space
            frame.tmpPose = rel * reference_pose;
        }
        else
        {
            localMapMatcher.SearchByProjectionFrameToKeyframe(frame, *ref, 8, 75);
        }
        inliers = poseRefinement.RefinePoseWithMatches(frame);

        // still less than 50 inliers after triple search?
        if (inliers < min_matches)
        {
            return false;
        }
    }

    TEST_MAP_SYNC;


    reference_keyframe = ref;

    for (int i = 0; i < frame.N; ++i)
    {
        if (!frame.mvpMapPoints[i]) continue;
        MapPoint* mp = frame.mvpMapPoints[i];
        if (frame.mvbOutlier[i])
        {
            frame.mvpMapPoints[i] = nullptr;
            frame.mvbOutlier[i]   = false;
            matches--;
        }
        else if (*mp)
        {
            mp->lastFrameSeen = frame.id;
        }
    }

    TEST_MAP_SYNC;

    frame.trackingInliers = inliers;

    return true;
}

bool Tracking::try_localize(FramePtr frame)
{
    frame->computeBoW();

    // Relocalization is performed when tracking is lost
    // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
    std::vector<std::pair<Keyframe*, float>> candidates =
        keyFrameDB->DetectRelocalizationCandidates(&frame->bow_vec, 0, 2);

    if (candidates.empty()) return false;

    {
        for (auto kfid : candidates)
        {
            auto kf = kfid.first;
            if (TrackBruteForce(*frame, kf, 60, false))
            {
                auto lock        = map.LockReadOnly();
                frame->validPose = true;
                frame->setReference(kf);
                return true;
            }
        }
    }
    return false;
}


}  // namespace Snake
