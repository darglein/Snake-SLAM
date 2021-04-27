/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once

#include "Map/Features.h"
#include "System/Module.h"
#include "System/SnakeGlobal.h"

namespace Saiga
{
class ORBExtractor;
class ORBExtractorGPU;
}  // namespace Saiga

namespace Snake
{
class FeatureDetector : public Module
{
   public:
    FeatureDetector();
    ~FeatureDetector();

    FramePtr GetFrame() { return output_buffer.get(); }

   private:
    void Detect(Frame& frame);

    std::unique_ptr<ORBExtractor> extractor;

#ifdef SNAKE_CUDA
    std::unique_ptr<ORBExtractorGPU> extractorGPU2;
#endif

    ScopedThread featureDetectionThread;
    SynchronizedSlot<FramePtr> output_buffer;

    std::string tmpDir = "features/";
    int start_frame    = 0;
};

// Owned by System. Global variable to enable easy access from different modules.
inline FeatureDetector* featureDetector;

}  // namespace Snake
