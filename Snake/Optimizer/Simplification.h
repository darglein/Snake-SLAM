/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once

#include "saiga/core/imgui/imgui_saiga.h"
#include "saiga/vision/slam/WeightedUndirectedGraph.h"

#include "DelayedParallelMapOptimization.h"
#include "Map/Map.h"
#include "System/Module.h"

namespace Snake
{
class Simplification : public DelayedParallelMapOptimization, public Module
{
   public:
    Simplification();
    virtual ~Simplification();
    void Process(Keyframe* kf);

   private:
    float Redundancy(KeyFrame* kf);
    std::tuple<bool, const char*, double> KeyFrameCullingGraph(KeyFrame* kf);
    void EraseKeyframe(Keyframe* kf);

    using GraphType = Saiga::WeightedUndirectedGraph<int>;
    GraphType graph;
    std::vector<Keyframe*> tmp_keyframes;

    // Settings which are not supposed to be changed by the user.
    static constexpr bool simpl_enable              = true;
    static constexpr int simpl_delay                = 8;
    static constexpr float simpl_border_angle       = 1.0;
    static constexpr float simpl_border_redundancy  = 0.8;
    static constexpr int simpl_border_matches       = 80;
    static constexpr int simpl_min_matches_in_graph = 20;
};

// Owned by System. Global variable to enable easy access from different modules.
inline Simplification* simplification;

}  // namespace Snake
