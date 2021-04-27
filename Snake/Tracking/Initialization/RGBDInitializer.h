/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once
#include "Initializer.h"
namespace Snake
{
class SAIGA_ALIGN_CACHE RGBDInitializer : public Initializer
{
   public:
    RGBDInitializer(int quality) : Initializer(quality) {}
    virtual InitializerResult Initialize(FramePtr frame) override;

   private:
    InitializerResult result;

    // Set this to true to get some randomness into the initialization.
    // Otherwise it is completely deterministic.
    static constexpr bool remove_random_points = true;
};



}  // namespace Snake
