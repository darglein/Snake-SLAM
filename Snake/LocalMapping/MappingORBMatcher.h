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
class MappingORBMatcher
{
   public:
    int SearchForTriangulation2(const Keyframe* kf1, const Keyframe* kf2, const Mat3& E,
                                std::vector<std::pair<int, int>>& vMatchedPairs, float epipolarDistance,
                                int featureDistance);

    int SearchForTriangulationBF(const SE3& pose1, const SE3& pose2, const Keyframe* pKF1, const Keyframe* pKF2,
                                 const Mat3& F12, std::vector<std::pair<int, int>>& vMatchedPairs,
                                 float epipolarDistance, int featureDistance);

    int SearchForTriangulationProject(const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>& grid, const SE3& pose1,
                                      const SE3& pose2, const Keyframe* pKF1, const Keyframe* pKF2, const Mat3& F12,
                                      std::vector<std::pair<int, int>>& vMatchedPairs, float epipolarDistance,
                                      int featureDistance);


    int Fuse(Keyframe* kf, const vector<MapPoint*>& vpMapPoints, std::vector<std::pair<int, int>>& fuseCandidates,
             float th, float obs_factor = 2.0, int feature_th = 50);

    int Fuse(Keyframe* kf, const SE3& pose, std::vector<bool>* point_mask, const LocalMap<FusionPoint>& points,
             std::vector<std::pair<int, int>>& fuseCandidates, float th, float obs_factor, int feature_th);

   private:
    static constexpr int TH_HIGH      = 100;
    static constexpr int TH_MEDIUM    = 75;
    static constexpr int TH_LOW       = 50;
    static constexpr int HISTO_LENGTH = 30;

    vector<int> tmp_indices;
    vector<bool> tmp_flags;
};


int fuseCandidatesIntoKf(const std::vector<std::pair<int, int>>& fcs, Keyframe* kf);

}  // namespace Snake
