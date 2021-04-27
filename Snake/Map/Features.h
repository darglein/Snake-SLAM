/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once
#include "saiga/core/Core.h"
#include "saiga/vision/slam/FrameBase.h"
#include "saiga/vision/util/ScalePyramid.h"

#include "System/SnakeGlobal.h"

namespace Snake
{
inline ScalePyramid scalePyramid;

struct Features
{
    int N = 0;

    std::vector<KeyPoint> keypoints, keypoints_right;
    std::vector<FeatureDescriptor> descriptors, descriptors_right;

    AlignedVector<Vec2> normalized_points;
    std::vector<KeyPoint> undistorted_keypoints;
    std::vector<float> right_points;
    std::vector<float> depth;

    auto featureRange() const { return Range(0, N); }

    inline bool hasDepth(int id) { return depth[id] > 0; }


    void GetFeaturesInArea(vector<int>& indices, const Vec2& position, float r);
    void GetFeaturesInAreaAndCheckScale(vector<int>& indices, const Vec2& position, double r, double predicted_scale);
    void GetFeaturesInAreaAndCheckScale(vector<int>& indices, const Vec2& position, double r, int min_octave,
                                        int max_octave);

    Saiga::FeatureGrid2 grid;
};
}  // namespace Snake
