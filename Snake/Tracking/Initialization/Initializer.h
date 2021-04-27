/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once
#include "System/SnakeGlobal.h"

namespace Snake
{
struct InitializerResult
{
    bool success = false;

    // first kf in mono init. (RGBD init sets both these variables to the new KF)
    Keyframe* kfFirst = nullptr;
    // second kf in stereo
    Keyframe* kfLast = nullptr;
};

// Interface for stereo and mono initialization
class SAIGA_ALIGN_CACHE Initializer
{
   public:
    Initializer(int quality) : quality(quality) {}
    virtual ~Initializer() {}
    virtual InitializerResult Initialize(FramePtr frame) = 0;

   protected:
    int quality;
};

}  // namespace Snake
