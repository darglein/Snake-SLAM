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
class LoopORBmatcher
{
   public:
    // Project MapPoints into KeyFrame using a given Sim3 and search for duplicated MapPoints.
    int Fuse(KeyFrame* pKF, const std::vector<MapPoint*>& vpPoints, float th);

    static int MatchBruteforce(KeyFrame* pKF1, KeyFrame* pKF2, vector<MapPoint*>& vpMatches12, int threshold,
                               float ratio);
    static int MatchBoW(KeyFrame* pKF1, KeyFrame* pKF2, vector<MapPoint*>& vpMatches12, int threshold, float ratio);

   private:
    static constexpr int TH_HIGH      = 100;
    static constexpr int TH_LOW       = 50;
    static constexpr int HISTO_LENGTH = 30;

    vector<int> tmp_indices;
};

}  // namespace Snake
