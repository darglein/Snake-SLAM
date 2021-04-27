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
class Input : public Module
{
   public:
    Input();
    virtual ~Input() {}

    FramePtr GetFrame() { return output_buffer.get(); }

    void run();
    void quit() { running = false; }
    void SetPause(bool pause) { this->pause = pause; }

   private:
    FramePtr ReadNextFrame();
    void CreateCamera();
    void computeGrayscaleImage(Frame& frame);

    std::unique_ptr<CameraBase> camera_rgbd;
    std::unique_ptr<CameraBase> camera_mono;
    std::unique_ptr<CameraBase> camera_stereo;
    ScopedThread camera_thread;
    ScopedThread process_image_thread;

    double start_timestamp;
    bool first_image     = true;
    atomic<bool> running = false;
    bool pause           = false;

    int frameId = 0;


    SynchronizedSlot<FramePtr> camera_slot;

    // Output of the camera module
    SynchronizedBuffer<FramePtr> output_buffer = {2};
};

// Owned by System. Global variable to enable easy access from different modules.
inline Input* input;


}  // namespace Snake
