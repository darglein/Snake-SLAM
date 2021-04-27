/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once
#include "saiga/vision/ba/BAWrapper.h"
#include "saiga/vision/ceres/CeresBA.h"
#include "saiga/vision/recursive/BARecursive.h"
#include "saiga/vision/recursive/BARecursiveRel.h"
#include "saiga/vision/scene/Scene.h"

#include "Map/Map.h"
#include "System/DelayedParallelMapOptimization.h"
#include "System/Module.h"

namespace Snake
{
class LocalBundleAdjustment : public DelayedParallelMapOptimization, public Module
{
   private:
    struct Statistics
    {
        double timeInit     = 0;
        double timeSolve    = 0;
        double timeFinalize = 0;

        friend std::ostream& operator<<(std::ostream& strm, Statistics stat)
        {
            strm << "Times Init/Solve/Finish " << stat.timeInit << " " << stat.timeSolve << " " << stat.timeFinalize;
            return strm;
        }
    };


   public:
    LocalBundleAdjustment();
    virtual ~LocalBundleAdjustment();


    void lba(KeyFrame* kf);


    void Process(Keyframe* kf) override { lba(kf); }

    void printStatistics(std::ostream& strm);

   private:
    void MakeLocalScene(KeyFrame* kf);
    std::tuple<int, double, double> SolveLocalScene(KeyFrame* kf);
    void UpdateLocalScene(KeyFrame* kf);


    // this is incremented after every ba to quickly identify
    // which keyframes are already included in the scene
    int currentOptimizationId = 0;

    std::vector<Statistics> statistics;

    OptimizationOptions local_op_options;
    BAOptions local_ba_options;

    int stateBefore;

    // For each keyframe one of these objects is allocated.
    // The keyframe id references directly into the array.
    struct PerKeyframeData
    {
        int mnBAFixedForKF = 0;
        int mnBALocalForKF = 0;
        int id_in_scene    = -1;
    };
    std::vector<PerKeyframeData> perKeyframeData;

    struct PerPointData
    {
        int mnBALocalForKF = 0;
        int id_in_scene    = -1;
    };
    std::vector<PerPointData> perPointData;

    // ======== tmp variables to not allocate every frame =====
    Scene scene;

#if WITH_IMU
    //    CeresBA cba;
    BARecRel cba;
#else
    BARec cba;
#endif



    vector<std::pair<KeyFrame*, MapPoint*>> outliers;
    vector<KeyFrame*> keyFrames, tmp_keyframes;
    std::vector<MapPoint*> mapPoints;
    std::vector<int> keyframesi, pointsi;

    std::vector<KeyFrame*> kfmapinv;
    std::vector<MapPoint*> mpmapinv;

    float chi1Mono, chi1Stereo;
    float chi2Mono, chi2Stereo;


    static constexpr bool lba_enable = true;

    static constexpr int num_last_keyframes     = 20;
    static constexpr int num_neighbor_keyframes = 15;
};

// Owned by System. Global variable to enable easy access from different modules.
inline LocalBundleAdjustment* lba;

}  // namespace Snake
