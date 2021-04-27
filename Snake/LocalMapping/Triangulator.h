/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once

#include "saiga/vision/g2o/g2oBA2.h"
#include "saiga/vision/recursive/BARecursive.h"
#include "saiga/vision/scene/Scene.h"

#include "Map/Map.h"
#include "MappingORBMatcher.h"

namespace Snake
{
struct ImageTriangulationResult
{
    struct NewPoint
    {
        int featureId1, featureId2;
        Vec3 worldPosition;
        bool far_away = false;
    };

    Keyframe *kf1, *kf2;
    AlignedVector<NewPoint> newPoints;
};

struct TriangulationParams
{
    float errorMono;
    float errorStereo;
    float epipolarDistance;
    FeatureDistance feature_distance;
    bool only_past_keyframes = true;
    int num_neighbors        = 10;
    bool check_third         = false;
    int num_threads          = 1;
};

class Triangulator
{
   public:
    ~Triangulator();
    int Process(TriangulationParams params, Keyframe* kf, std::vector<MapPoint*>* out_points);

   private:
    ImageTriangulationResult triangulate(TriangulationParams params, Keyframe* kf1, Keyframe* kf2, bool precise);
    void ComputeDepthMap(Keyframe* kf);
    std::vector<ImageTriangulationResult> newPointsa;
    std::vector<KeyFrame*> tmp_keyframes;
    SE3 pose1;

    Eigen::Matrix<double, -1, -1, Eigen::RowMajor> grid;
    Eigen::Matrix<int, -1, -1, Eigen::RowMajor> grid_bool;

    struct PerThreadData
    {
        vector<std::pair<int, int>> tmp_matches;
        MappingORBMatcher triangulationMatcher;
    };
    std::vector<PerThreadData> thread_data;
};



}  // namespace Snake
