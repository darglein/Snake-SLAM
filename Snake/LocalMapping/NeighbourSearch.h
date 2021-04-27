/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once
#include "LocalMapping/MappingORBMatcher.h"
#include "Map/Map.h"
#include "System/DelayedParallelMapOptimization.h"
namespace Snake
{
struct MapSearchParams
{
    float fuse_threshold          = 3;
    FeatureDistance feature_error = 50;
    float two_match_factor        = 2.0;
    bool only_older_keyframes     = false;
    int num_threads               = 1;
};

class MapSearcher
{
   public:
    MapSearcher();
    int Process(const MapSearchParams& params, Keyframe* kf);

   private:
    //  Compute the neighbouring keyframes in which we want to search additional matches. The result is stored in the
    //  member neighbour_keyframes.
    int FindNeighbours(Keyframe* kf, bool only_older_keyframes);

    // Compute the points of this keyframe and the points of the neighbours and stores them in the respective member
    // variables.
    int FindPoints(Keyframe* kf);

    int FindFuseCandidates(Keyframe* kf, const MapSearchParams& params);

    int FuseCandidatesIntoKf(Keyframe* kf);

    vector<KeyFrame*> neighbour_keyframes;
    //    vector<MapPoint*> points_of_kf;

    LocalMap<FusionPoint> neighbour_points, kf_points;
    AlignedVector<SE3> neighbor_poses;
    SE3 kf_pose;


    // ====== tmp variables to save multiple allocs =======

    vector<MapPoint*> tmp_points;
    vector<KeyFrame*> tmp_keyframes;

    struct PerThreadData
    {
        MappingORBMatcher triangulationMatcher;
    };
    std::vector<PerThreadData> thread_data;


    std::vector<std::vector<bool>> point_mask;
    std::vector<std::vector<std::pair<int, int>>> fuse_candidates_neighbours;
    std::vector<SE3> neighbours_pose;

    std::vector<std::pair<int, int>> fuse_candidates;


    // For each keyframe one of these objects is allocated.
    // The keyframe id references directly into the array.
    struct PerKeyframeData
    {
        // Start with -1 so we can fuse the 0th keyframe as well
        int mnFuseTargetForKF = -1;
    };
    std::vector<PerKeyframeData> perKeyframeData;

    struct PerPointData
    {
        int mnFuseCandidateForKF = 0;
    };
    int opt_id = 0;
    std::vector<PerPointData> perPointData;
};



}  // namespace Snake
