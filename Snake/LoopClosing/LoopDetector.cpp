/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */
//#define ENABLE_MAP_SYNC_TEST
#include "LoopDetector.h"

#include "saiga/core/imgui/imgui.h"

#include "LocalMapping/LocalMapping.h"
#include "LoopORBMatcher.h"
#include "Tracking/PoseRefinement.h"


namespace Snake
{
LoopDetector::LoopDetector()
{
    mnLoopPointForKF.resize(maxPoints, -1);
}

Loop LoopDetector::Detect(Keyframe* kf)
{
    loop = Loop();

    bool found_initial = DetectLoop(kf);
    loop.candidates    = candidateKFs.size();

    if (found_initial)
    {
        TEST_MAP_SYNC;
        // currently the computesim3 does a lock during the complete execution.
        if (ComputeSim3(kf))
        {
            loop.found_loop = true;
            CHECK_VALID_MAP;
            TEST_MAP_SYNC;
        }
    }
    CHECK_VALID_MAP;
    return loop;
}

bool LoopDetector::DetectLoop(KeyFrame* new_kf)
{
    // Avoid that a keyframe can be erased while it is being process by this thread

    TEST_MAP_SYNC;
    vector<Keyframe*> all_connected_keyframes;
    {
        // we read the graph only
        auto lock = map.LockReadOnly();

        // Compute reference BoW similarity score
        // This is the lowest score to a connected keyframe in the covisibility graph
        // We will impose loop candidates to have a higher similarity than this
        //        new_kf->GetVectorCovisibleKeyFrames(all_connected_keyframes);
        new_kf->GetCovisiblesByWeight(all_connected_keyframes, 5);
    }


    TEST_MAP_SYNC;

    // This is the bottleneck of this function
    // it's at least 99% of the runtime
    // Note: we don't need to lock the map during that :)

    if (!all_connected_keyframes.empty())
    {
        auto center_keyframe = all_connected_keyframes[int(all_connected_keyframes.size() * 0.75)];

        float score = vocabulary.score(new_kf->frame->bow_vec, center_keyframe->frame->bow_vec);

        if (average_min_score == -1)
        {
            average_min_score = score;
        }
        else
        {
            average_min_score = min_score_alpha * average_min_score + (1.0 - min_score_alpha) * score;
        }
    }

    {
        auto cands =
            keyFrameDB->DetectLoopCandidates(&new_kf->frame->bow_vec, all_connected_keyframes, average_min_score, 10);


        initial_candidates.clear();

        // Shared lock because we access the keyframe connections
        auto lock = map.LockReadOnly();
        for (auto& ca : cands)
        {
            initial_candidates.emplace_back(ca.first, ca.second);
        }
        SAIGA_ASSERT(
            std::is_sorted(initial_candidates.begin(), initial_candidates.end(), std::greater<LoopCandidate>()));
    }

    return CheckConsistency(new_kf);
}

bool LoopDetector::CheckConsistency(KeyFrame* new_kf)
{
    candidateKFs.clear();
    for (auto& candidate : initial_candidates)
    {
        // go over all previous kf which are not consistent yet
        for (auto& previous_candidate : previous_candidates)
        {
            if (previous_candidate.InGroup(candidate.kf))
            {
                candidate.consistency_count =
                    std::max(candidate.consistency_count, previous_candidate.consistency_count + 1);
            }
        }

        // Add consistent kfs to candidates, if there are no overlapping groups in there yet.
        if (candidate.consistency_count >= 2)
        {
            bool found = false;
            for (auto& c : candidateKFs)
            {
                if (c.InGroup(candidate.kf))
                {
                    found = true;
                    break;
                }
            }
            if (!found)
            {
                candidateKFs.push_back(candidate);
            }
        }
    }
    previous_candidates = initial_candidates;
    SAIGA_ASSERT(std::is_sorted(candidateKFs.begin(), candidateKFs.end(), std::greater<LoopCandidate>()));


    candidateKFs.resize(std::min(candidateKFs.size(), max_match_keyframes));

    return !candidateKFs.empty();
}


std::tuple<SE3, double, int> LoopDetector::solve(KeyFrame* pKF1, KeyFrame* pKF2,
                                                 const std::vector<MapPoint*>& vpMatched12, vector<bool>& vbInliers,
                                                 bool compute_scale)
{
    solver.clear();
    solver.pose1 = pKF1->Pose();
    solver.pose2 = pKF2->Pose();
    //    solver.threshold = 9.210;
    solver.threshold = 12;
    solver.N         = 0;
    solver.camera1   = K;
    solver.camera2   = K;

    auto& tmp_matches = pKF1->GetMapPointMatches();

    for (int i1 = 0; i1 < (int)vpMatched12.size(); i1++)
    {
        MapPoint* pMP2 = vpMatched12[i1];
        if (!pMP2 || pMP2->isBad())
        {
            //            SAIGA_EXIT_ERROR("sdf");
            continue;
        }

        MapPoint* pMP1 = tmp_matches[i1];
        if (!pMP1 || pMP1->isBad())
        {
            SAIGA_EXIT_ERROR("sdf");
            continue;
        }


        int indexKF1 = pMP1->GetIndexInKeyFrame(pKF1);
        int indexKF2 = pMP2->GetIndexInKeyFrame(pKF2);

        if (indexKF1 < 0 || indexKF2 < 0) continue;

        auto& kp1 = pKF1->frame->undistorted_keypoints[indexKF1];
        auto& kp2 = pKF2->frame->undistorted_keypoints[indexKF2];


        solver.ips1.push_back(kp1.point);
        solver.ips2.push_back(kp2.point);

        // Keep points in view-space
        solver.points1.push_back(solver.pose1 * pMP1->getPosition());
        solver.points2.push_back(solver.pose2 * pMP2->getPosition());


        solver.N++;
    }

    vbInliers.clear();
    vbInliers.resize(solver.N, false);

    int its = RansacIterationsFromProbability(solver.N, 0.999, 15, 100);

    return solver.solve(its, compute_scale);
}



bool LoopDetector::ComputeSim3(KeyFrame* source_kf, Keyframe* target_kf)
{
    if (source_kf->isBad() || target_kf->isBad()) return false;

    vector<MapPoint*> vvpMapPointMatches;

    // Bruteforce feature matching.
    // We use relatively relaxed filter criteria, because there might so some large changes between the images.
    //
    // This can also have race-conditions, but the system can't crash from it. An actual race might only invalidate a
    // match, but the remaining computatains are robust anyways.

#if 0
    int nmatches = LoopORBmatcher::MatchBruteforce(source_kf, target_kf, vvpMapPointMatches, 120, 0.9);
#else
    int nmatches = LoopORBmatcher::MatchBoW(source_kf, target_kf, vvpMapPointMatches, 50, 0.75);
#endif


    // Note: Even with IMU small scale drift can occure.
    // The Sim3 loop closure is robust anyways so optimizing the additional scale parameters also doesn't hurt.
    bool compute_scale = settings.inputType == InputType::Mono;


    loop.matches = std::max(loop.matches, nmatches);


    if (nmatches < 20)
    {
        return false;
    }

    vector<bool> vbInliers;

    //    pSolver.SetRansacParameters(nmatches, 0.999, 15, 100);

    {
        // load the mappoints into local arrays
        //        pSolver.init();
    }
    auto [T, scale, nInliers] = solve(source_kf, target_kf, vvpMapPointMatches, vbInliers, compute_scale);
    DSim3 T_target_csource    = DSim3(T, scale);


    DSim3 corrected_source_sim3 = T_target_csource.inverse() * DSim3(target_kf->Pose(), 1.0);
    DSim3 T_w_corrected_source  = corrected_source_sim3.inverse();

    VLOG(1) << "scale: " << scale << " inliers: " << nInliers << "/" << nmatches;


    loop.inliers = std::max(loop.inliers, nInliers);

    // If RANSAC returns a Sim3, perform a guided matching and optimize with all correspondences
    if (nInliers < 10) return false;

    vector<MapPoint*> vpMapPointMatches(vvpMapPointMatches.size(), static_cast<MapPoint*>(NULL));
    for (size_t j = 0, jend = vbInliers.size(); j < jend; j++)
    {
        if (vbInliers[j]) vpMapPointMatches[j] = vvpMapPointMatches[j];
    }


    SnakeORBMatcher matcher2;
    auto source_frame = source_kf->frame;

    auto old_matches = source_frame->mvpMapPoints;
    auto old_pose    = source_frame->tmpPose;
    source_frame->clearMatches();
    source_frame->tmpPose          = T_w_corrected_source.se3().inverse();
    auto projection_search_inliers = matcher2.SearchByProjectionFrameToKeyframe(*source_frame, *target_kf, 5, 50);
    VLOG(1) << "projection search: " << projection_search_inliers << " inliers.";


    PoseRefinement poseRefinement;
    auto pose_ref_inliers = poseRefinement.RefinePoseWithMatches(*source_frame);

    if (pose_ref_inliers < 30)
    {
        return false;
    }
    VLOG(1) << "pose ref inliers: " << pose_ref_inliers << " inliers.";

    int far_points   = 0;
    int close_points = 0;
    for (int i = 0; i < source_frame->N; ++i)
    {
        auto& mp = source_frame->mvpMapPoints[i];
        if (!mp || source_frame->mvbOutlier[i]) continue;

        if (mp->far_stereo_point || source_frame->depth[i] > th_depth)

        {
            far_points++;
        }
        else
        {
            close_points++;
        }
    }

    if (close_points < 30)
    {
        VLOG(1) << "not enough close points: " << close_points << "/" << far_points;
        return false;
    }


    if (compute_scale)
    {
        // recompute scale
        double scale_sum  = 0;
        int scale_inliers = 0;
        for (int i = 0; i < source_frame->N; ++i)
        {
            auto& mp = source_frame->mvpMapPoints[i];
            if (!mp || source_frame->mvbOutlier[i]) continue;

            // check if there is also a point in they kf
            auto kf_mp = source_kf->GetMapPoint(i);
            if (kf_mp)
            {
                // project both an compare depth
                auto z1            = (source_frame->tmpPose * mp->getPosition()).norm();
                auto z2            = (source_kf->Pose() * kf_mp->getPosition()).norm();
                double point_scale = z1 / z2;

                double absolute_difference = fabs(point_scale - scale);
                double relative_difference = absolute_difference / std::max(fabs(point_scale), fabs(scale));

                if (relative_difference < 0.1)
                {
                    scale_sum += point_scale;
                    scale_inliers++;
                }
            }
        }

        auto mean_scale = scale_sum / scale_inliers;
        VLOG(1) << "mean scale " << mean_scale << " from " << scale_inliers << " inliers";


        if (scale_inliers < 15)
        {
            return false;
        }
        T_w_corrected_source.se3()   = source_frame->tmpPose.inverse();
        T_w_corrected_source.scale() = mean_scale;
    }
    else
    {
        T_w_corrected_source.se3()   = source_frame->tmpPose.inverse();
        T_w_corrected_source.scale() = 1;
    }

    loop.mvpCurrentMatchedPoints = source_frame->mvpMapPoints;

    source_frame->mvpMapPoints = old_matches;
    source_frame->tmpPose      = old_pose;

    loop.scale             = scale;
    loop.source_keyframe   = source_kf;
    loop.target_keyframe   = target_kf;
    loop.T_w_correctSource = T_w_corrected_source;
    loop.T_target_source   = T_w_corrected_source * DSim3(source_kf->PoseInv().inverse(), 1);
    return true;
}

bool LoopDetector::ComputeSim3(KeyFrame* new_kf)
{
    auto lock = map.LockReadOnly();


    KeyFrame* target_kf = nullptr;

    for (auto& candidate : candidateKFs)
    {
        if (ComputeSim3(new_kf, candidate.kf))
        {
            target_kf = candidate.kf;
            break;
        }
    }

    if (!target_kf) return false;


    {
        // Retrieve MapPoints seen in Loop Keyframe and neighbors
        vector<KeyFrame*> target_connected_kfs = loop.target_keyframe->GetVectorCovisibleKeyFrames();
        target_connected_kfs.push_back(loop.target_keyframe);
        loop.target_map_points.clear();
        for (auto pKF : target_connected_kfs)
        {
            for (auto mp : pKF->GetMapPointMatches())
            {
                if (!mp || mp->isBad()) continue;

                if (mnLoopPointForKF[mp->id()] != new_kf->id())
                {
                    if (mp->IsInKeyFrame(new_kf))
                    {
                        continue;
                    }
                    loop.target_map_points.push_back(mp);
                    mnLoopPointForKF[mp->id()] = new_kf->id();
                }
            }
        }

        // Retrieve MapPoints seen in Loop Keyframe and neighbors
        vector<KeyFrame*> source_connected_kfs = loop.source_keyframe->GetVectorCovisibleKeyFrames();
        source_connected_kfs.push_back(loop.source_keyframe);
        loop.source_map_points.clear();
        for (auto pKF : source_connected_kfs)
        {
            for (auto mp : pKF->GetMapPointMatches())
            {
                if (!mp || mp->isBad()) continue;

                if (mnLoopPointForKF[mp->id()] != new_kf->id())
                {
                    if (mp->IsInKeyFrame(new_kf))
                    {
                        continue;
                    }
                    loop.source_map_points.push_back(mp);
                    mnLoopPointForKF[mp->id()] = new_kf->id();
                }
            }
        }
    }

    // If enough matches accept Loop
    int nTotalMatches = 0;
    for (size_t i = 0; i < loop.mvpCurrentMatchedPoints.size(); i++)
    {
        if (loop.mvpCurrentMatchedPoints[i]) nTotalMatches++;
    }
    std::cout << "Found Loop with " << nTotalMatches << " matches. " << std::endl;
    return nTotalMatches >= 40;
}

}  // namespace Snake
