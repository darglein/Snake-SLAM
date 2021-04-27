/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once
#include "saiga/core/Core.h"
#include "saiga/core/imgui/imgui_saiga.h"
#include "saiga/core/time/TimerBase.h"
namespace Snake
{
enum class ModuleType : int
{
    INPUT              = 0,
    FEATURE_DETECTOR   = 1,
    PREPROCESS         = 2,
    TRACKING           = 3,
    KEYFRAME_INSERTION = 4,
    OPTIMIZER          = 5,
    SIMPLIFICATION     = 6,
    DEFERRED_MAPPER    = 7,
    LOOP_CLOSING       = 8,
    IMU_SOLVER         = 9,
    OTHER              = 10
};

constexpr const char* module_str_list[] = {
    "Input",     "Feature Detector", "Preprocess",      "Tracking",     "Keyframe Insertion",
    "Optimizer", "Simplification",   "Deferred Mapper", "Loop Closing", "IMU Solver",
    "Other"};

// Modules except "OTHER".
constexpr int knum_modules = 10;


// A global struct, where each module can write its stats in the destructor.
struct PerformanceStats
{
    // For each module a vector of times
    // Everything is given in ms.
    std::vector<std::vector<float>> timings;


    PerformanceStats() : timings(knum_modules)
    {
        for (auto& v : timings)
        {
            v.reserve(10000);
        }
    }

    void AddTime(ModuleType type, float time)
    {
        int id = (int)type;
        timings[id].push_back(time);
    }
    void PrintStatistics();
    void PrintTimings();
};
inline PerformanceStats performance_stats;

class Module
{
   public:
    Module(ModuleType type);
    virtual ~Module() {}

    void CreateTable(const std::vector<int>& colum_width, const std::vector<std::string>& colum_name);

    void imgui();

    auto ModuleTimer() { return ScopedModuleTimer(type); }
    auto LastTime() { return performance_stats.timings[id].back(); }

    struct ScopedModuleTimer : public Saiga::TimerBase
    {
        ScopedModuleTimer(ModuleType type) : type(type) { start(); }
        ~ScopedModuleTimer()
        {
            auto time =
                std::chrono::duration_cast<std::chrono::duration<float, typename std::chrono::milliseconds::period>>(
                    stop())
                    .count();
            performance_stats.AddTime(type, time);
        }
        ModuleType type;
    };

   protected:
    ModuleType type;
    std::string name;
    int id;
    std::unique_ptr<ImGui::IMTable> output_table;
};



}  // namespace Snake
