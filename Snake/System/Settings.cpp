/**
 * Copyright (c) 2017 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#include "Settings.h"

#include "saiga/core/Core.h"
#include "saiga/core/imgui/imgui.h"

#include "System/SnakeGlobal.h"

namespace Snake
{
void ViewerSettings::fromConfigFile(const std::string& file)
{
    Saiga::SimpleIni ini;
    ini.LoadFile(file.c_str());

    INI_GETADD_BOOL(ini, "Viewer", enabled);
    INI_GETADD_DOUBLE(ini, "Viewer", KeyFrameSize);
    INI_GETADD_DOUBLE(ini, "Viewer", KeyFrameLineWidth);
    INI_GETADD_DOUBLE(ini, "Viewer", GraphLineWidth);
    INI_GETADD_DOUBLE(ini, "Viewer", PointSize);
    INI_GETADD_DOUBLE(ini, "Viewer", CameraLineWidth);

    INI_GETADD_BOOL(ini, "Viewer", renderInput);
    INI_GETADD_BOOL(ini, "Viewer", renderPoints);
    INI_GETADD_BOOL(ini, "Viewer", renderCurrentCam);
    INI_GETADD_BOOL(ini, "Viewer", renderKeyframes);
    INI_GETADD_BOOL(ini, "Viewer", renderFrames);
    INI_GETADD_BOOL(ini, "Viewer", renderEdges);
    INI_GETADD_BOOL(ini, "Viewer", renderVelocity);

    INI_GETADD_BOOL(ini, "Viewer", smoothCamera);
    INI_GETADD_BOOL(ini, "Viewer", followCamera);

    if (ini.changed()) ini.SaveFile(file.c_str());
}

void ViewerSettings::imgui()
{
    ImGui::InputFloat("KeyFrameSize", &KeyFrameSize);
    ImGui::InputFloat("KeyFrameLineWidth", &KeyFrameLineWidth);
    ImGui::InputFloat("GraphLineWidth", &GraphLineWidth);
    ImGui::InputFloat("PointSize", &PointSize);
    ImGui::InputFloat("CameraLineWidth", &CameraLineWidth);

    ImGui::ColorEdit3("color_points", color_points.data());
    ImGui::ColorEdit3("color_keyframes", color_keyframes.data());
    ImGui::ColorEdit3("color_graph", color_graph.data());

    ImGui::Checkbox("renderInput", &renderInput);
    ImGui::Checkbox("renderPoints", &renderPoints);
    ImGui::Checkbox("renderEdges", &renderEdges);
    ImGui::Checkbox("renderKeyframes", &renderKeyframes);
    ImGui::Checkbox("renderFrames", &renderFrames);
    ImGui::Checkbox("renderVelocity", &renderVelocity);
}


Settings::Settings(const std::string& file)
{
    Saiga::SimpleIni ini;
    ini.LoadFile(file.c_str());

    config_file = file;
    {
        auto group = "Global";
        INI_GETADD_LONG_COMMENT(ini, group, randomSeed, "# 0 == Time based random");


        if (randomSeed == 0)
        {
            randomSeed = (uint32_t)Saiga::Random::generateTimeBasedSeed();
        }


        INI_GETADD_STRING(ini, group, evalDir);
        INI_GETADD_STRING(ini, group, out_file_prefix);
        INI_GETADD_LONG(ini, group, num_tracking_threads);
        INI_GETADD_BOOL(ini, group, async);
        INI_GETADD_BOOL(ini, group, async_lba);
        INI_GETADD_BOOL(ini, group, start_paused);
        INI_GETADD_BOOL(ini, group, keep_valid_depth_image);
        INI_GETADD_BOOL(ini, group, keep_all_frame_data);
    }

    {
        auto group = "Input";

        int inputTypeId = static_cast<int>(inputType);
        inputTypeId     = ini.GetAddLong(group, "inputType", inputTypeId,
                                     "# 0 = Mono\n"
                                     "# 1 = RGBD\n"
                                     "# 2 = Stereo");
        inputType       = static_cast<InputType>(inputTypeId);

        int sensorTypeId = static_cast<int>(sensorType);
        sensorTypeId     = ini.GetAddLong(group, "sensorTypeId", sensorTypeId,
                                      "# 0 = PRIMESENSE\n"
                                      "# 1 = RAW_DATASET\n"
                                      "# 2 = TUM-RGBD\n"
                                      "# 3 = ZJU\n"
                                      "# 4 = EUROC\n"
                                      "# 5 = KITTI\n"
                                      "# 6 = SCANNET\n"
                                      "# 7 = Azure");
        sensorType       = static_cast<SensorType>(sensorTypeId);


        INI_GETADD_STRING(ini, group, voc_file);
    }

    {
        auto group = "FeatureDetector";
        INI_GETADD_LONG(ini, group, fd_features);
        INI_GETADD_LONG(ini, group, fd_levels);
        INI_GETADD_LONG(ini, group, fd_iniThFAST);
        INI_GETADD_LONG(ini, group, fd_minThFAST);
        INI_GETADD_DOUBLE(ini, group, fd_scale_factor);
        INI_GETADD_LONG(ini, group, fd_threads);
        INI_GETADD_BOOL(ini, group, fd_bufferToFile);
        INI_GETADD_BOOL(ini, group, fd_drawDebugImage);
        INI_GETADD_BOOL(ini, group, fd_gpu);
        INI_GETADD_BOOL(ini, group, fd_relaxed_stereo);
    }


    {
        auto group = "Tracking";
        INI_GETADD_LONG(ini, group, kfi_target_matches);
        INI_GETADD_LONG(ini, group, th_map);
        INI_GETADD_LONG(ini, group, initialization_quality);
    }


    {
        auto group = "IMU";
        INI_GETADD_BOOL(ini, group, enable_imu);
        INI_GETADD_DOUBLE(ini, group, weight_gyro_optimization);
        INI_GETADD_DOUBLE(ini, group, weight_acc_optimization);
    }

    if (ini.changed()) ini.SaveFile(file.c_str());

    datasetParameters.fromConfigFile(file);
    viewer_settings.fromConfigFile(file);


#if __has_feature(thread_sanitizer)
    settings.datasetParameters.multiThreadedLoad = false;
#endif
}

void Settings::SetDefaultParametersForDataset()
{
    if (sensorType == SensorType::EUROC)
    {
        kfi_target_matches = 160;
        th_map             = 140;

        weight_gyro_optimization   = 1000;
        weight_acc_optimization    = 400;
        weight_gyro_initialization = 0.3;
        weight_gyro_tracking       = 0.2;

    }

    if (sensorType == SensorType::KINECT_AZURE || sensorType == SensorType::SAIGA_RAW)
    {
        kfi_target_matches = 160;
        th_map             = 150;

        weight_gyro_optimization   = 4000;
        weight_acc_optimization    = 400;
        weight_gyro_initialization = 0.5;
    }

    if (sensorType == SensorType::ZJU)
    {
        kfi_target_matches = 150;
        th_map             = 100;


        initial_bias_gyro          = Vec3(-0.0170773, 0.0117907, -0.00879683);
        weight_gyro_optimization   = 1000;
        weight_acc_optimization    = 200;
        weight_acc_tracking        = 0.2;
        weight_gyro_initialization = 0.5;
    }
    if (sensorType == SensorType::KITTI)
    {
        kfi_target_matches = 175;
        th_map             = 140;
    }
}


void Settings::Print()
{
    auto type_str   = input_str_list[(int)inputType];
    auto sensor_str = sensor_str_list[(int)sensorType];


    std::cout << Saiga::ConsoleColor::GREEN;
    Saiga::Table table({2, 18, 41, 2});
    std::cout << "========================= Snake-SLAM =========================" << std::endl;
    table << "|"
          << "Input Type" << type_str << "|";
    table << "|"
          << "Sensor Type" << sensor_str << "|";
    table << "|"
          << "FPS Limit" << datasetParameters.playback_fps << "|";
    table << "|"
          << "Dataset Dir" << datasetParameters.dir << "|";
    table << "|"
          << "Num Features" << fd_features << "|";
    table << "|"
          << "GPU" << fd_gpu << "|";
    table << "|"
          << "Th Map" << th_map << "|";
    table << "|"
          << "Target Matches" << kfi_target_matches << "|";
    table << "|"
          << "Async" << async << "|";
    table << "|"
          << "Async LBA" << async_lba << "|";
    table << "|"
          << "Viewer" << viewer_settings.enabled << "|";
    table << "|"
          << "Tracking Threads" << num_tracking_threads << "|";
    table << "|"
          << "Feature Threads" << fd_threads << "|";
    table << "|"
          << "Random Seed" << randomSeed << "|";

    int total_number_of_threads = 1 + fd_threads + 1 + num_tracking_threads;
    if (async)
    {
        total_number_of_threads += 4;
    }
    table << "|"
          << "Total Threads" << total_number_of_threads << "|";

    table << "|"
          << "Max Keyframes" << maxKeyframes << "|";
    table << "|"
          << "Max Points" << maxPoints << "|";

#if WITH_IMU
    table << "|"
          << "Weight Gyro" << weight_gyro_optimization << "|";
    table << "|"
          << "Weight Gyro Init" << weight_gyro_initialization << "|";
    table << "|"
          << "Weight Acc" << weight_acc_optimization << "|";
#endif


    std::cout << "==============================================================" << std::endl;
    std::cout << Saiga::ConsoleColor::RESET;
}


}  // namespace Snake
