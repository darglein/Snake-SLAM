/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

//#define ENABLE_MAP_SYNC_TEST

#include "NeighbourSearch.h"

#include "saiga/core/imgui/imgui.h"

#include "Optimizer/LocalBundleAdjustment.h"

namespace Snake
{
MapSearcher::MapSearcher()
{
    perKeyframeData.resize(maxKeyframes);
    perPointData.resize(maxPoints);
}

int MapSearcher::Process(const MapSearchParams& params, Keyframe* kf)
{
    if (kf->isBad()) return 0;
    SAIGA_ASSERT(kf->id() >= 0);

    int neighbors       = 0;
    int fused_points    = 0;
    int search_points   = 0;
    int fuse_candidates = 0;
    float time;
    {
        Saiga::ScopedTimer timer(time);
        TEST_MAP_SYNC;
        {
            auto lock = map.LockReadOnly();
            neighbors = FindNeighbours(kf, params.only_older_keyframes);
            if (neighbors == 0)
            {
                return 0;
            }
            search_points = FindPoints(kf);
        }

        fuse_candidates = FindFuseCandidates(kf, params);
        {
            auto lock    = map.LockFull();
            fused_points = FuseCandidatesIntoKf(kf);
        }
    }
    return fused_points;
}

int MapSearcher::FindNeighbours(Keyframe* kf, bool only_older_keyframes)
{
    neighbour_keyframes.clear();
    kf->GetBestCovisibilityKeyFrames(neighbour_keyframes, 15);



    neighbour_keyframes.erase(std::remove_if(neighbour_keyframes.begin(), neighbour_keyframes.end(),
                                             [](auto other_kf) { return other_kf->isBad(); }),
                              neighbour_keyframes.end());

    kf_pose = kf->Pose();

    opt_id++;

    for (int i = 0; i < neighbour_keyframes.size(); ++i)
    {
        auto kf2 = neighbour_keyframes[i];
        SAIGA_ASSERT(kf2);
        SAIGA_ASSERT(!kf2->isBad());

        auto& other_data = perKeyframeData[kf2->id()];
        SAIGA_ASSERT(other_data.mnFuseTargetForKF < opt_id);
        other_data.mnFuseTargetForKF = opt_id;
    }

    for (int i = 0, last = neighbour_keyframes.size(); i < last && i < 100; ++i)
    {
        auto kf2 = neighbour_keyframes[i];
        tmp_keyframes.clear();
        kf2->GetBestCovisibilityKeyFrames(tmp_keyframes, 10);
        for (auto pKFi2 : tmp_keyframes)
        {
            auto& other_data2 = perKeyframeData[pKFi2->id()];
            if (pKFi2->isBad() || other_data2.mnFuseTargetForKF == opt_id || pKFi2->id() == kf->id()) continue;
            other_data2.mnFuseTargetForKF = opt_id;


            neighbour_keyframes.push_back(pKFi2);
        }
    }

    if (only_older_keyframes)
    {
        neighbour_keyframes.erase(std::remove_if(neighbour_keyframes.begin(), neighbour_keyframes.end(),
                                                 [kf](auto other_kf) { return other_kf->id() > kf->id(); }),
                                  neighbour_keyframes.end());
    }

    neighbours_pose.resize(neighbour_keyframes.size());
    for (int i = 0; i < neighbour_keyframes.size(); ++i)
    {
        neighbours_pose[i] = neighbour_keyframes[i]->Pose();
    }

    return neighbour_keyframes.size();
}

int MapSearcher::FindPoints(Keyframe* kf)
{
    int points_to_search = 0;

    tmp_points.clear();
    kf->GetMapPointMatches(tmp_points);
    tmp_points.erase(std::remove_if(tmp_points.begin(), tmp_points.end(), [](MapPoint* p) { return !p || p->isBad(); }),
                     tmp_points.end());
    kf_points.clear();
    kf_points.addPoints(tmp_points);

    point_mask.clear();
    point_mask.resize(neighbour_keyframes.size());

    for (int k = 0; k < neighbour_keyframes.size(); ++k)
    {
        point_mask[k].resize(kf_points.points.size());
        for (int p = 0; p < kf_points.points.size(); ++p)
        {
            point_mask[k][p] = !map.getMapPoint(kf_points.points[p].id).IsInKeyFrame(neighbour_keyframes[k]);
        }
    }

    // Search matches by projection from target KFs in current KF
    neighbour_points.clear();
    for (auto pKFi : neighbour_keyframes)
    {
        tmp_points.clear();
        pKFi->GetMapPointMatches(tmp_points);

        for (auto mp : tmp_points)
        {
            if (!mp || mp->isBad()) continue;

            auto& data = perPointData[mp->id()];
            if (data.mnFuseCandidateForKF == kf->id()) continue;
            if (mp->IsInKeyFrame(kf)) continue;

            neighbour_points.addPoint(mp);
            data.mnFuseCandidateForKF = kf->id();
        }
    }

    points_to_search += kf_points.points.size();
    points_to_search += neighbour_points.points.size();
    return points_to_search;
}

int MapSearcher::FindFuseCandidates(Keyframe* kf, const MapSearchParams& params)
{
    fuse_candidates_neighbours.resize(neighbour_keyframes.size());

    std::atomic<int> total_candidates = 0;
    thread_data.resize(params.num_threads);

    // Search matches by projection from current KF in target KFs
#pragma omp parallel for num_threads(params.num_threads) schedule(dynamic)
    for (int j = 0; j < (int)neighbour_keyframes.size() + 1; ++j)
    {
        int tid                    = OMP::getThreadNum();
        auto& triangulationMatcher = thread_data[tid].triangulationMatcher;
        if (j == 0)
        {
            total_candidates +=
                triangulationMatcher.Fuse(kf, kf_pose, nullptr, neighbour_points, fuse_candidates,
                                          params.fuse_threshold, params.two_match_factor, params.feature_error);
        }
        else
        {
            int i    = j - 1;
            auto kf2 = neighbour_keyframes[i];
            SAIGA_ASSERT(kf2 != kf);

            auto& fc = fuse_candidates_neighbours[i];
            total_candidates +=
                triangulationMatcher.Fuse(kf2, neighbours_pose[i], &point_mask[i], kf_points, fc, params.fuse_threshold,
                                          params.two_match_factor, params.feature_error);
        }
    }
    return total_candidates;
}

int MapSearcher::FuseCandidatesIntoKf(Keyframe* kf)
{
    int fusedPoints = 0;

    for (int i = 0; i < (int)neighbour_keyframes.size(); ++i)
    {
        auto kf  = neighbour_keyframes[i];
        auto& fc = fuse_candidates_neighbours[i];
        fusedPoints += fuseCandidatesIntoKf(fc, kf);
    }

    fusedPoints += fuseCandidatesIntoKf(fuse_candidates, kf);

    for (auto& mp : kf->MapPointRef())
    {
        if (mp)
        {
            if (mp->isBad())
            {
                mp = nullptr;
                continue;
            }
            SAIGA_ASSERT(!mp->isBad());
            mp->ComputeDistinctiveDescriptors();
            mp->UpdateNormal();
        }
    }
    kf->UpdateConnections();
    return fusedPoints;
}

}  // namespace Snake
