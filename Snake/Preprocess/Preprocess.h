/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once
#include "Map/Frame.h"
#include "System/Module.h"
#include "System/SnakeGlobal.h"
namespace Snake
{
/**
 * Takes as input the Frame and features and computes:
 * - Undistorted keypoints
 * - Grid-based accelaration structure for projection
 * - Reorder keypoints to for cache performance
 * - Initialized the frame
 * - Stereo matches and depth
 */
class Preprocess : public Module
{
   public:
    Preprocess();


    FramePtr GetFrame() { return output_buffer.get(); }

   private:
    void Process(Frame& frame);
    int ComputeStereoFromRGBD(Frame& frame);
    void undistortKeypoints(Frame& frame);
    int StereoMatching(Frame& frame);
    void computeFeatureGrid(Frame& frame);

    SynchronizedSlot<FramePtr> output_buffer;
    Thread thread;
};

// Owned by System. Global variable to enable easy access from different modules.
inline Preprocess* preprocess;

}  // namespace Snake
