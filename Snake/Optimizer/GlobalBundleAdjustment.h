/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once

#include "saiga/vision/ba/BAWrapper.h"
#include "saiga/vision/recursive/BAPointOnly.h"
#include "saiga/vision/recursive/BARecursive.h"
#include "saiga/vision/recursive/BARecursiveRel.h"
#include "saiga/vision/scene/Scene.h"

#include "Map/Map.h"
#include "System/DelayedParallelMapOptimization.h"
#include "System/Module.h"

#define USE_TIGHT_SOLVER 0
namespace Snake
{
class GlobalBundleAdjustment : public Module
{
   public:
    GlobalBundleAdjustment();
    ~GlobalBundleAdjustment() {}

    void FullBA(int iterations = 5, bool remove_outliers = true);
    void FullBARel(int iterations, bool remove_outliers);

    void PointBA(int iterations = 4, bool remove_outliers = true);

    // Optimize pose of all frames except the keyframes.
    // The points are fixed.
    void RealignIntermiediateFrames(bool ceres, bool with_imu);

   private:
    void MakeGlobalScene();
    void UpdateGlobalScene(bool remove_outliers);

    OptimizationOptions global_op_options;
    OptimizationOptions global_op_options_points;
    BAOptions global_ba_options;


    // ======== tmp variables to not allocate every frame =====
    Scene scene;

#if WITH_IMU


#    if USE_TIGHT_SOLVER
    Imu::ImuFullSolver solver;
    Imu::CeresBAWithIMU cba;
#    else
    // eresBA cbaceres;
    BARecRel cba;
#    endif
#else
    BARec cba;
#endif

    vector<std::pair<KeyFrame*, MapPoint*>> outliers;
    vector<KeyFrame*> keyFrames;
    std::vector<MapPoint*> mapPoints;
    std::vector<int> keyframesi, pointsi;
    std::vector<KeyFrame*> kfmapinv;
    std::vector<MapPoint*> mpmapinv;

    struct PerKeyframeData
    {
        int id_in_scene = -1;
    };
    std::vector<PerKeyframeData> perKeyframeData;

    // gba is used from the loop closer and IMU solver.
    // this lock guarantees that it's not executed twice
    std::mutex gba_lock;
};


inline GlobalBundleAdjustment* gba;


}  // namespace Snake
