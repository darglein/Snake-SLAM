/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once

#include "saiga/vision/scene/PoseGraph.h"

#include "KeyframeDatabase.h"
#include "LoopDetector.h"
#include "Map/Map.h"
#include "System/DelayedParallelMapOptimization.h"
#include "System/Module.h"
namespace Snake
{
class LocalMapping;
class Optimizer;



class LoopClosing : public DelayedParallelMapOptimization, public Module
{
   public:
    LoopClosing();
    ~LoopClosing();

    void Process(Keyframe* kf) override;

   private:
    void SearchAndFuse(const std::vector<KeyFrame*>& source_keyframes, const std::vector<MapPoint*>& points);

    void OptimizeEssentialGraph();

    void ConstructPGO(std::set<Keyframe*> constant_kfs,
                      const std::map<Keyframe*, std::pair<SE3, DSim3>>& transformed_poses, bool fix_scale);



    void CorrectLoop(const Loop& _loop);
    void CorrectLoopold(const Loop& _loop);

    LoopDetector loopDetector;
    int last_loop_close_kf = -1;

    static constexpr bool lc_enable = true;
    static constexpr int lc_delay   = 0;


    struct LC_PerKeyframeData
    {
        KeyFrame* kf;
        SE3 before_T_w_i;
        DSim3 after_T_w_i;
        int index_in_pg = -1;
    };
    PoseGraph pg;
    std::vector<LC_PerKeyframeData> per_keyframe_data;
};

// Owned by System. Global variable to enable easy access from different modules.
inline LoopClosing* loopClosing;

}  // namespace Snake
