/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once
#include "LocalMapping/NeighbourSearch.h"
#include "LocalMapping/Triangulator.h"
#include "Map/Map.h"
#include "System/DelayedParallelMapOptimization.h"
#include "System/Module.h"
#include "Tracking/SnakeORBMatcher.h"
namespace Snake
{
class DeferredMapper : public DelayedParallelMapOptimization, public Module
{
   public:
    DeferredMapper();

    virtual void Process(Keyframe* kf) override;

   private:
    // Many keypoints are very close to each other.
    // This function projects the current mappoint matches and checks if an other keypoint
    // in that area might fit better than the current point.
    // If yes only the link is replaced.
    //
    // Return:
    // Pair   <relinked_observations, removed_observations>
    //
    std::pair<int, int> Relink(KeyFrame* kf);

    // Find new matches between keyframes and mappoints by reprojection.
    // Same algorithm as in keyframe insertion.
    //
    // Return:
    //    <New Connections>
    int MapSearch(Keyframe* kf);

    // Retriangulates new points between kf and older other keyframes.
    // Uses the same triangulation algorithm as during keyframe insertion.
    //
    // Return:
    //    <new_points, accepted_points, erased_points>
    std::tuple<int, int, int> Retriangulate(Keyframe* kf);

    MapSearcher ns;
    Triangulator tri;
    std::vector<MapPoint*> recently_created_points;
    vector<int> tmp_indices;

    static constexpr bool pmo_enable = true;
    static constexpr int pmo_delay   = 9;
};

// Owned by System. Global variable to enable easy access from different modules.
inline DeferredMapper* deferredMapper;

}  // namespace Snake
