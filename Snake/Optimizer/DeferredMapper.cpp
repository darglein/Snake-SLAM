/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */
#include "DeferredMapper.h"

#include "saiga/core/imgui/imgui.h"
#include "saiga/core/util/FileSystem.h"



namespace Snake
{
DeferredMapper::DeferredMapper()
    : DelayedParallelMapOptimization("PMO", pmo_enable, pmo_delay, settings.async), Module(ModuleType::DEFERRED_MAPPER)
{
    CreateTable({7, 7, 7, 7, 7, 7}, {"KF", "Link", "Remove", "Fuse", "Tri.", "Time (ms)"});
}


void DeferredMapper::Process(Keyframe* kf)
{
    if (kf->isBad()) return;

    int obs_relinked, obs_removed;
    int fused;
    int new_points, accepted_points, late_erased_points;

    {
        auto timer                                                = ModuleTimer();
        std::tie(obs_relinked, obs_removed)                       = Relink(kf);
        fused                                                     = MapSearch(kf);
        std::tie(new_points, accepted_points, late_erased_points) = Retriangulate(kf);
    }
    (*output_table) << kf->id() << obs_relinked << obs_removed << fused << new_points << LastTime();
}

std::pair<int, int> DeferredMapper::Relink(KeyFrame* kf)
{
    static constexpr auto relink_reprojection_error_threshold = 0.8;
    static constexpr auto relink_outlier_threshold            = reprojectionErrorThresholdMono;
    static constexpr auto relink_feature_threshold            = 25;

    // full lock. we modify inside the loop
    auto lock = map.LockFull();

    if (!kf || kf->isBad())
    {
        return {0, 0};
    }

    auto outlier_threshold_squared = relink_outlier_threshold * relink_outlier_threshold;

    int obs_relinked = 0;
    int obs_removed  = 0;

    auto pose = kf->Pose();

    for (auto i : kf->frame->featureRange())
    {
        auto&& kp = kf->frame->undistorted_keypoints[i];
        auto mp   = kf->GetMapPoint(i);
        if (!mp || mp->isBad()) continue;

        auto idx = mp->GetIndexInKeyFrame(kf);
        SAIGA_ASSERT(idx == i);


        Vec3 wp                = mp->getPosition();
        Vec3 np                = pose * wp;
        double z               = np(2);
        Vec2 ip                = K.project(np);
        auto rep_error_squared = (ip - kp.point).squaredNorm();

        if (np.z() <= 0 || rep_error_squared > outlier_threshold_squared)
        {
            kf->EraseMapPointMatch(mp);
            mp->EraseObservation(kf);
            obs_removed++;
            continue;
        }

        auto dKF         = kf->frame->descriptors[i];
        auto& dMP        = mp->descriptor;
        auto featureDist = distance(dMP, dKF);

        if (featureDist == 0)
        {
            // find other descriptor
            for (auto o : mp->GetObservationList())
            {
                if (o.first != kf)
                {
                    //                    std::cout << "use other desc" << std::endl;
                    dKF         = o.first->frame->descriptors[o.second];
                    featureDist = distance(dMP, dKF);
                    break;
                }
            }
        }

        // Let's search only in a radius which is in the error threshold
        tmp_indices.clear();
        kf->frame->GetFeaturesInArea(tmp_indices, ip, relink_reprojection_error_threshold);


        // If there are multiple candidates, we pick the point with the smallest feature distance
        int bestDist = featureDist;
        int bestIdx  = -1;

        for (auto j : tmp_indices)
        {
            if (i == j) continue;

            auto&& kp2 = kf->frame->undistorted_keypoints[j];
            auto& dKF2 = kf->frame->descriptors[j];

            auto error_squared = (ip - kp2.point).squaredNorm();
            if (error_squared > rep_error_squared) continue;

            if (kf->frame->right_points[j] > 0)
            {
                auto disp = stereo_cam.LeftPointToRight(ip.x(), z);
                auto er   = (disp - kf->frame->right_points[j]);
                if (er * er > rep_error_squared * 2.0)
                {
                    continue;
                }
            }

            auto featureDist2 = distance(dMP, dKF2);
            if (featureDist2 < relink_feature_threshold && featureDist2 < bestDist)
            {
                bestDist = featureDist2;
                bestIdx  = j;
            }
        }

        if (bestIdx != -1)
        {
            auto oldMp = kf->GetMapPoint(bestIdx);
            if (oldMp && !oldMp->isBad())
            {
                // well, we found a better point which is already linked
                // so one of them is probably an outlier.
                // let's just remove this match. the other match is maybe corrected in the future.
                kf->EraseMapPointMatch(mp);
                mp->EraseObservation(kf);
                obs_removed++;
                continue;
            }

            // Actual relinking
            kf->EraseMapPointMatch(mp);
            kf->addMappoint(mp, bestIdx);
            mp->RelinkObservation(kf, i, bestIdx);
            obs_relinked++;

            mp->ComputeDistinctiveDescriptors();
            mp->UpdateNormal();
        }
    }
    return {obs_relinked, obs_removed};
}

int DeferredMapper::MapSearch(Keyframe* kf)
{
    MapSearchParams params;
    params.fuse_threshold =
        settings.inputType == InputType::Mono ? reprojectionErrorThresholdMono : reprojectionErrorThresholdStereo;
    params.fuse_threshold *= 1;
    params.two_match_factor     = 1.5;
    params.feature_error        = 50;
    params.only_older_keyframes = true;
    params.num_threads          = 1;
    return ns.Process(params, kf);
}

std::tuple<int, int, int> DeferredMapper::Retriangulate(Keyframe* kf)
{
    //
    // Triangulate new points
    TriangulationParams params;
    float factor               = 0.8;
    params.errorMono           = reprojectionErrorThresholdMono * factor;
    params.errorStereo         = reprojectionErrorThresholdStereo * factor;
    params.epipolarDistance    = 1.2;
    params.feature_distance    = 40;
    params.only_past_keyframes = true;
    params.num_neighbors       = 5;
    auto new_points            = tri.Process(params, kf, &recently_created_points);

    int accepted_points    = 0;
    int late_erased_points = 0;

    //
    // Remove points with less than 3 matches
    auto lock = map.LockFull();
    recently_created_points.erase(std::remove_if(recently_created_points.begin(), recently_created_points.end(),
                                                 [&](MapPoint* mp) {
                                                     if (mp->isBad()) return true;
                                                     auto timeSinceCreation = kf->id() - mp->mnFirstKFid;
                                                     auto observations      = mp->GetNumObservations();

                                                     if (timeSinceCreation > params.num_neighbors)
                                                     {
                                                         if (observations <= 2)
                                                         {
                                                             late_erased_points++;
                                                             mp->SetBadFlag();
                                                             mp = nullptr;
                                                             return true;
                                                         }
                                                         else
                                                         {
                                                             accepted_points++;
                                                             return true;
                                                         }
                                                     }
                                                     return false;
                                                 }),
                                  recently_created_points.end());
    return {new_points, accepted_points, late_erased_points};
}


}  // namespace Snake
