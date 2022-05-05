/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */
#pragma once

#include "buildconfig.h"

/**
 *  This file is included everywhere
 */
#include "saiga/core/util/Thread/SpinLock.h"
#include "saiga/core/util/Thread/SynchronizedBuffer.h"
#include "saiga/core/util/Thread/SynchronizedSlot.h"
#include "saiga/core/util/Thread/omp.h"
#include "saiga/core/util/Thread/threadName.h"
#include "saiga/core/util/easylogging++.h"
#include "saiga/vision/VisionIncludes.h"
#include "saiga/vision/camera/CameraBase.h"
#include "saiga/vision/cameraModel/Distortion.h"
#include "saiga/vision/features/Features.h"
#include "saiga/vision/util/FeatureGrid2.h"

#include "Settings.h"

#include <iostream>
#include <thread>

namespace Snake
{
using namespace Saiga;
// Math types
using Scalar = double;
using Vec4   = Saiga::Vec4;
using Vec3   = Saiga::Vec3;
using Vec2   = Saiga::Vec2;
using Vec4f  = Saiga::vec4;
using Vec3f  = Saiga::vec3;
using Vec2f  = Saiga::vec2;
using Mat4   = Saiga::Mat4;
using Mat3   = Saiga::Mat3;
using SE3    = Saiga::SE3;
using std::vector;

// Types which are used throughout SnakeSLAM.
// You can change these here if you want to use your own implementations.
using Intrinsics = Saiga::MonocularIntrinsics;

// Features
using FeatureDistance   = int;
using FeatureDescriptor = Saiga::DescriptorORB;
using KeyPoint          = Saiga::KeyPoint<double>;

// Threading stuff
using Thread = Saiga::ScopedThread;
using Saiga::SynchronizedBuffer;
using Saiga::SynchronizedSlot;
using std::atomic;


using std::cout;
using std::endl;


#if __has_feature(thread_sanitizer)
// With TSAN everything is slow anyways and std::mutex works better for
// detecting race conditions
// using SmallMutex = std::mutex;
#    define SNAKE_FULL_SYNC
using SmallMutex    = Saiga::DummyLock;
using TSANOnlyMutex = std::mutex;
#else
// A mutex for low-contention sections
using SmallMutex    = Saiga::DummyLock;
using TSANOnlyMutex = Saiga::DummyLock;
#endif



// Forward Declarations
class MapPoint;
class Keyframe;
class Frame;
class LocalMapping;
using FramePtr = std::shared_ptr<Frame>;

}  // namespace Snake


namespace Snake
{
// Set by the input module depending on the input sensor.
inline MonocularIntrinsics mono_intrinsics;
inline StereoIntrinsics stereo_intrinsics;
inline RGBDIntrinsics rgbd_intrinsics;


inline IntrinsicsPinholed & K = mono_intrinsics.model.K;
inline StereoCamera4 stereo_cam;

inline Rectification rect_left;
inline Rectification rect_right;

inline bool is_dataset = true;
inline bool has_imu    = false;
inline Imu::Sensor imu;



// 20 Px diameter of the cells
#define FEATURE_GRID_ROWS 24
#define FEATURE_GRID_COLS 32
// inline Saiga::FeatureGridBounds<double, FEATURE_GRID_ROWS, FEATURE_GRID_COLS> featureGridBounds;
inline Saiga::FeatureGridBounds2<double, 20> featureGridBounds;

// inline std::atomic_bool trackingRunning      = false;
// inline std::atomic_bool localmappringRunning = false;

constexpr int maxFeatures = 1500;

#ifdef SNAKE_MOBILE
constexpr bool store_previous_frame = true;
constexpr int maxKeyframes          = 1000;
constexpr int maxPoints             = 100000;
#else
constexpr bool store_previous_frame = true;
constexpr int maxKeyframes          = 10000;
constexpr int maxPoints             = 10000000;
#endif


#ifdef SNAKE_OS_MODE
constexpr float reprojectionErrorThresholdMono   = 2.44765;    // sqrt(5.991)
constexpr float reprojectionErrorThresholdStereo = 2.7955321;  // sqrt(7.815)
constexpr float trackingErrorFactorCoarse        = 1;
constexpr float trackingErrorFactorFine          = 1;
constexpr float lbaErrorFactor                   = 1.0;
inline double th_depth                           = 3;
#else

// constexpr float reprojectionErrorThresholdMono   = 2.44765;
// constexpr float reprojectionErrorThresholdStereo = 2.7955321;

constexpr float reprojectionErrorThresholdMono   = 2.1;
constexpr float reprojectionErrorThresholdStereo = 2.3;

constexpr float trackingErrorFactorCoarse = 1;
constexpr float trackingErrorFactorFine   = 1;
constexpr float lbaErrorFactor            = 1;
inline double th_depth                    = 20;
#endif

constexpr float reprojectionErrorThresholdMono2   = reprojectionErrorThresholdMono * reprojectionErrorThresholdMono;
constexpr float reprojectionErrorThresholdStereo2 = reprojectionErrorThresholdStereo * reprojectionErrorThresholdStereo;


inline std::atomic_bool pause       = false;
inline std::atomic_bool stop_camera = false;


inline CameraBase* globalCamera;

//#define MATCHING_CHECK_SCALE_CONSISTENCY
//#define MATCHING_MIN_MAX_DISTANCE

// Most evals were run with these 2 enabled.
#define MATCHING_CHECK_SCALE_CONSISTENCY2
#define MATCHING_MIN_MAX_DISTANCE2

// Free the image after tracking to reduce memory consumption.
// A dense-reconstruction requires the depth image to be kept in memory.
#define FREE_COLOR_IMAGE 0
#define FREE_DEPTH_IMAGE 0


#define WITH_IMU 1

#if WITH_IMU

// Global weights multiplied to gyro and acc error.
// 0 at the start and then increased after state estimation gets better.
inline double current_gyro_weight        = 0;
inline double current_acc_weight         = 0;
inline double acc_position_interpolation = 0;
inline bool map_clear                    = false;

// Current bias estimates
// inline Vec3 imu_bias_gyro = Vec3::Zero();
// inline Vec3 imu_bias_acc = Vec3::Zero();
inline Imu::Gravity imu_gravity;



// gyro + acc
// inline bool imu_initialized = false;

// simplification (can change)
inline float max_time_between_kf_map = 0.5;

// for keyframe insertion
constexpr float max_time_between_kf_tracking = 0.5;
#endif



}  // namespace Snake
