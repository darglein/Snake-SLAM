/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once
#include "saiga/core/Core.h"

#include "LocalMapping/LocalMapping.h"
#include "LoopClosing/LoopClosing.h"
#include "Preprocess/Input.h"
#include "Tracking/Tracking.h"

namespace Snake
{
class System
{
   public:
    System(const Settings& _settings);
    System(const Settings& settings, std::function<std::unique_ptr<ViewerInterface>(void)> create_viewer);

    ~System();
    void run();

    // early quit for example when the window is closed
    void quit();

    void waitForFinish();

   private:
    std::vector<std::unique_ptr<Module>> modules;

    void RematchIntermiediate();

    void SaveSublonet();



    bool finished            = false;
    std::atomic_bool running = false;
    std::unique_ptr<ViewerInterface> viewer;

    ScopedThread viewerThread;

    void imgui();

    void writeFrameTrajectory(const std::string& file, bool write_id = false);
    void writeKeyFrameTrajectory(const std::string& file);
};

}  // namespace Snake
