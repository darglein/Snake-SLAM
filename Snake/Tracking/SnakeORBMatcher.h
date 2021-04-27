/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */


#pragma once

#include "Map/Frame.h"
#include "Map/LocalMap.h"
#include "System/SnakeGlobal.h"
namespace Snake
{
class SnakeORBMatcher
{
   public:
    SnakeORBMatcher();

    // BF Tracking and Loop Closing
    int SearchByProjectionFrameToKeyframe(Frame& CurrentFrame, const Keyframe& kf, float th,
                                          FeatureDistance featureError);

    // Fine Tracking
    int SearchByProjection2(Frame& CurrentFrame, LocalMap<FineTrackingPoint>& lm, const float th, float ratio,
                            int num_threads);

    // Coarse Tracking
    int SearchByProjectionFrameFrame2(Frame& CurrentFrame, const LocalMap<CoarseTrackingPoint>& lm, const float th,
                                      FeatureDistance featureError, int num_threads);


   private:
    static constexpr int TH_HIGH               = 100;
    static constexpr int TH_MEDIUM             = 75;
    static constexpr int TH_LOW                = 50;
    static constexpr int HISTO_LENGTH          = 30;
    static constexpr float disparity_threshold = 4;

    vector<int> tmp_indices;
    vector<bool> tmp_flags;

    std::array<std::vector<int>, HISTO_LENGTH> rotHist;

    // OMP variables
    vector<vector<int>> tmp_indices_omp;
    vector<std::pair<int, int>> tmp_matches_world_image;
    //    vector<vector<std::pair<int, int>>> tmp_fuseCandidates;
    //    SE3 currentPose;
    //    Vec3 cameraPosition;
    //    int N;
    //    int matches;
};

}  // namespace Snake
