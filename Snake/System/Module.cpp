/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */
#include "Module.h"

#include "saiga/core/imgui/imgui.h"

#include "Map/Map.h"
#include "System/Settings.h"
namespace Snake
{
void Snake::PerformanceStats::PrintStatistics()
{
    {
        auto error_sim3 = map.TrajectoryError(true);
        //        auto error_sim3_2 = map.TrajectoryError2(true);
        auto error_se3 = map.TrajectoryError(false);
        auto rep_stats = map.ReprojectionStats();

        std::cout << ConsoleColor::RED;
        Table table({2, 30, 15, 2});
        std::cout << "================ Map Statistics ================" << std::endl;
        table << "|"
              << "Frames" << timings.front().size() << "|";
        table << "|"
              << "Keyframes"
              << std::to_string(rep_stats.num_keyframes) + "/" + std::to_string(rep_stats.num_inserted_keyframes)
              << "|";
        table << "|"
              << "Points" << std::to_string(rep_stats.num_points) + "/" + std::to_string(rep_stats.num_inserted_points)
              << "|";
        table << "|"
              << "Observations" << rep_stats.num_observations << "|";
        table << "|"
              << "Obs/KF" << double(rep_stats.num_observations) / rep_stats.num_keyframes << "|";
        table << "|"
              << "Obs/Point" << double(rep_stats.num_observations) / rep_stats.num_points << "|";
        table << "|"
              << "Reprojection RMSE (px)" << rep_stats.rms << "|";

        table << "|"
              << "Abs. Trans. RMSE Sim3 (m)" << error_sim3.ate_rmse << "|";
        table << "|"
              << "Scale (GT==1)" << error_sim3.scale_error << "|";


        //        table << "|"
        //              << "Abs. Trans. RMSE Sim3 (m)" << error_sim3_2.ate_rmse << "|";
        //        table << "|"
        //              << "Scale (GT==1)" << error_sim3_2.scale_error << "|";
        table << "|"
              << "Total Predicted Scale:" << map.total_scale << "|";
        table << "|"
              << "Total GT Scale:" << map.total_scale * error_sim3.scale_error << "|";

        if (settings.inputType != InputType::Mono || has_imu)
        {
            double rel_scale_error = error_sim3.scale_error;
            if (rel_scale_error < 1)
            {
                rel_scale_error = 1.0 / rel_scale_error;
            }
            rel_scale_error = (rel_scale_error - 1.0) * 100;
            table << "|"
                  << "Rel Scale Error (%)" << rel_scale_error << "|";


            table << "|"
                  << "Abs. Trans. RMSE SE3 (m)" << error_se3.ate_rmse << "|";
        }

        std::cout << "================================================" << std::endl;
        std::cout << ConsoleColor::RESET;
    }
}
void Snake::PerformanceStats::PrintTimings()
{
    {
        std::cout << ConsoleColor::RESET;
        Table table({2, 30, 15, 10, 2});
        std::cout << "===================== Module Timings =====================" << std::endl;

        table << "|"
              << "       Module"
              << "Time (ms)"
              << "Hz (1/s)"
              << "|";

        for (int i = 0; i < knum_modules; ++i)
        {
            auto stats = Saiga::Statistics(timings[i]);
            table << "|" << module_str_list[i] << stats.mean << 1.0 / (stats.mean / 1000.0) << "|";
        }

        std::cout << "==========================================================" << std::endl;
        std::cout << ConsoleColor::RESET;
    }
}

Module::Module(ModuleType type) : type(type)
{
    id   = (int)type;
    name = module_str_list[id];
}

void Module::CreateTable(const std::vector<int>& colum_width, const std::vector<std::string>& colum_name)
{
    output_table = std::make_unique<ImGui::IMTable>(std::to_string(id) + " " + name, colum_width, colum_name);
}

void Module::imgui()
{
    if (output_table)
    {
        auto ws                           = ImGui::GetIO().DisplaySize;
        int x_id                          = id % 2;
        int y_id                          = id / 2;
        output_table->console.size        = Saiga::ivec2(350, 150);
        int start_x                       = ws.x - (2 * output_table->console.size(0));
        int start_y                       = 0;
        output_table->console.position(0) = start_x + x_id * output_table->console.size(0);
        output_table->console.position(1) = start_y + y_id * output_table->console.size(1);


        output_table->Render();
    }
}

}  // namespace Snake
