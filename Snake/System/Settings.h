/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */
#pragma once

#include "saiga/core/Core.h"
#include "saiga/vision/camera/CameraBase.h"
#include "saiga/vision/cameraModel/MotionModel.h"

#include "buildconfig.h"

#include <string>

namespace Snake
{
enum class InputType : int
{
    Mono   = 0,
    RGBD   = 1,
    Stereo = 2
};
constexpr const char* input_str_list[] = {"Mono", "RGBD", "Stereo"};

enum class SensorType : int
{
    PRIMESENSE   = 0,
    SAIGA_RAW    = 1,
    TUM_RGBD     = 2,
    ZJU          = 3,
    EUROC        = 4,
    KITTI        = 5,
    SCANNET      = 6,
    KINECT_AZURE = 7,
};

constexpr const char* sensor_str_list[] = {"PRIMESENSE", "SAIGA_RAW", "TUM_RGBD", "ZJU",
                                           "EUROC",      "KITTI",     "SCANNET",  "Azure"};

struct ViewerSettings
{
    bool enabled = true;

    float KeyFrameSize      = 0.02;
    float KeyFrameLineWidth = 2;
    float GraphLineWidth    = 2;
    float PointSize         = 4;
    float CameraLineWidth   = 4;

    Saiga::vec3 color_points    = Saiga::vec3(1, 0, 0);
    Saiga::vec3 color_keyframes = Saiga::vec3(0, 1, 0);

    Saiga::vec3 color_graph = Saiga::vec3(0, 0, 1);


    bool renderBoundingbox = false;
    bool renderFloor       = true;
    bool renderInput       = true;
    bool renderPoints      = true;
    bool renderCurrentCam  = true;
    bool renderKeyframes   = true;
    bool renderFrames      = false;
    bool renderEdges       = true;
    bool renderVelocity    = false;
    bool smoothCamera      = true;
    bool followCamera      = true;

    void fromConfigFile(const std::string& file);
    void imgui();
};


/**
 * Important settings used all over snake.
 * These are loaded before everything else, so they can be used by the modules during construction.
 *
 * @brief The GlobalSettings struct
 */
struct Settings
{
    // ====== General ======

    // 0 means a new seed is generated at program start
    long randomSeed             = 0;
    std::string evalDir         = "eval_out/";
    std::string out_file_prefix = "trajectory";
    bool async                  = false;
    bool async_lba              = false;
    int num_tracking_threads    = 4;
    bool start_paused           = false;

    // keep the depth image of frames with a valid pose.
    // required for afterwards dense reconstruction
    bool keep_valid_depth_image = false;

    // keep the complete frame data for all frames
    // required to save the input stream
    bool keep_all_frame_data = false;

    // Points to this file
    std::string config_file;


    // ====== Input ======

    InputType inputType   = InputType::Mono;
    SensorType sensorType = SensorType::EUROC;
    std::string voc_file  = "ORBvoc.minibow";
    Saiga::DatasetParameters datasetParameters;

    // ====== Feature Detector ======

    int fd_features        = 1000;
    float fd_scale_factor  = 1.2;
    int fd_levels          = 4;
    int fd_iniThFAST       = 20;
    int fd_minThFAST       = 7;
    int fd_threads         = 2;
    bool fd_bufferToFile   = false;
    bool fd_drawDebugImage = false;
    bool fd_gpu            = false;
    bool fd_relaxed_stereo = true;


    // ====== Tracking ======

    // Keyframes are inserted so that target_matches are reached.
    //  -> A higher value means more keyframes are inserted.
    int kfi_target_matches = 180;

    // Simplification threshold. All keyframes above th_map are removed.
    //   -> A higher value means less keyframes are removed.
    int th_map = 140;

    // Initilization quality for monocular input.
    // Higher numbers means higher quality (and more required feature matches)
    // Range [0,2]
    int initialization_quality = 1;



    // ======== IMU Settings =========
    bool enable_imu = true;


    Saiga::Vec3 initial_bias_gyro = Saiga::Vec3::Zero();
    Saiga::Vec3 initial_bias_acc  = Saiga::Vec3::Zero();

    // relative to optimization weight
    double weight_gyro_initialization = 0.3;
    double weight_gyro_optimization   = 100;
    double weight_gyro_tracking       = 0.2;


    double weight_acc_optimization = 10;
    double weight_acc_tracking     = 0.1;


    ViewerSettings viewer_settings;


    Settings() {}
    Settings(const std::string& file);

    void SetDefaultParametersForDataset();

    void Print();
};

inline Settings settings;


}  // namespace Snake
