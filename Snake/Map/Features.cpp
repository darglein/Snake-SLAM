/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */
#include "Frame.h"
#include "Map/Keyframe.h"
#include "Map/MapPoint.h"


namespace Snake
{
void Features::GetFeaturesInArea(vector<int>& indices, const Vec2& position, float r)
{
    indices.clear();
    auto [cellMin, cellMax] = featureGridBounds.minMaxCellWithRadius(position, r);
    auto r2                 = r * r;
    for (auto cx : Range(cellMin.first, cellMax.first + 1))
    {
        for (auto cy : Range(cellMin.second, cellMax.second + 1))
        {
            for (auto pid : grid.cellIt({cx, cy}))
            {
                auto& kp = undistorted_keypoints[pid];
                if ((kp.point - position).squaredNorm() < r2) indices.push_back(pid);
            }
        }
    }
}

void Features::GetFeaturesInAreaAndCheckScale(vector<int>& indices, const Vec2& position, double r,
                                              double predicted_scale)
{
    indices.clear();
    auto r2                 = r * r;
    auto [cellMin, cellMax] = featureGridBounds.minMaxCellWithRadius(position, r);
    for (auto cx : Range(cellMin.first, cellMax.first + 1))
    {
        for (auto cy : Range(cellMin.second, cellMax.second + 1))
        {
            for (auto pid : grid.cellIt({cx, cy}))
            {
                auto& kp = undistorted_keypoints[pid];
                if (!scalePyramid.PredictionConsistent(predicted_scale, kp.octave)) continue;
                if ((kp.point - position).squaredNorm() < r2)
                {
                    indices.push_back(pid);
                }
            }
        }
    }
}


void Features::GetFeaturesInAreaAndCheckScale(vector<int>& indices, const Vec2& position, double r, int min_octave,
                                              int max_octave)
{
    indices.clear();
    auto r2                 = r * r;
    auto [cellMin, cellMax] = featureGridBounds.minMaxCellWithRadius(position, r);
    for (auto cx : Range(cellMin.first, cellMax.first + 1))
    {
        for (auto cy : Range(cellMin.second, cellMax.second + 1))
        {
            for (auto pid : grid.cellIt({cx, cy}))
            {
                auto& kp = undistorted_keypoints[pid];
                if (kp.octave < min_octave || kp.octave > max_octave) continue;
                if ((kp.point - position).squaredNorm() < r2)
                {
                    indices.push_back(pid);
                }
            }
        }
    }
}

}  // namespace Snake
