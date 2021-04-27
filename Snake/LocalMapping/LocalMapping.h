/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once
#include "saiga/core/imgui/imgui_saiga.h"
#include "saiga/vision/slam/MiniBow.h"

#include "Map/Map.h"
#include "MappingORBMatcher.h"
#include "Optimizer/DeferredMapper.h"
#include "Optimizer/Simplification.h"
#include "Triangulator.h"
namespace Snake
{
class LocalMapping : public Module
{
   private:
   public:
    LocalMapping();
    ~LocalMapping();


    virtual void Process(Keyframe* kf);


    void setLocalMapInitial();


    void UpdateViewer();

   private:
    void ProcessNewKeyFrame(KeyFrame* kf);
    void MapPointCulling(KeyFrame* kf);
    void CreateNewMapPoints(KeyFrame* kf);

    std::vector<MapPoint*> recentMapPoints;
    Triangulator triangulator;
    MapSearcher map_search;

    ThreadPool depth_process = {4, "Depth Process"};
};

// Owned by System. Global variable to enable easy access from different modules.
inline LocalMapping* localMapping;


}  // namespace Snake
