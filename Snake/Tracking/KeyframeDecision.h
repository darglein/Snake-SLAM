/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once
#include "saiga/core/Core.h"
#include "saiga/core/util/table.h"

#include "System/SnakeGlobal.h"

namespace Snake
{
enum class TrackingQuality
{
    SUPER_BAD = 0,
    BAD       = 1,
    MEDIUM    = 2,
    GOOD      = 3,
    VERY_GOOD = 4,
    NEED      = 5,
};

struct TrackingQualityEvaluator
{
    TrackingQualityEvaluator();
    std::pair<bool, double> NeedNewKeyframe(Frame& frame, Keyframe* last_keyframe);

   private:
    std::pair<TrackingQuality, const char*> ComputeTrackingQuality(Frame& frame);
    std::pair<bool, const char*> Decision(TrackingQuality quality, Frame& frame, Keyframe* last_keyframe);
    Table tab;
};

}  // namespace Snake
