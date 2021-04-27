/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */
#pragma once
#include "IMU/FullSolver.h"
#include "Map/Map.h"
#include "System/DelayedParallelMapOptimization.h"
#include "SnakeGlobal.h"
#include "System/Module.h"



namespace Snake
{
class ImuStateSolver : public DelayedParallelMapOptimization, public Module
{
   public:
    ImuStateSolver();
    virtual ~ImuStateSolver();

    // Called for every new keyframe (before LBA!!)
    void ProcessNewKeyframe(Keyframe* kf);

    void Process(Keyframe* kf);


    // Called sometimes from local mapping. Usually after LBA for every keyframe.
    // The system should be stable even if this is not or only sometimes called.
    void UpdateMap();


    bool InitGyroBias();
    bool InitGravityAndScale();

    void RecomputeWeights();

    void IterateBaImu(int k);

    // This enum also shows the order in which we initialize the IMU states.
    // Note that, during ACC_BIAS initializaion the gravity and scale is recomputed.
    enum class InitializationState
    {
        INITIALIZING_GYRO_BIAS,
        INITIALIZING_GRAVITY_SCALE,
        INITIALIZING_ACC_BIAS,
        INITIALIZING_VELOCITIES,
        OPTIMIZING,
        OPTIMIZING2,
        DONE
    };
    InitializationState current_init_state = InitializationState::INITIALIZING_GYRO_BIAS;

    // Reset initialzion after map clear.
    void clear();

    int x                            = 0;
    double start_time                = -1;
    double min_error                 = 1;
    bool use_initial_gyro_bias_guess = false;
    bool use_initial_acc_bias_guess  = false;

    // Variables only used during gyro initialization.
    Vec3 init_gyro_bias      = Vec3::Zero();
    int gyro_init_iterations = 0;

    int acc_init_iterations = 0;
    double init_scale       = 1;
    double init_done_time   = 0;
    int init_optimize_state = 0;
    Vec3 init_acc_bias      = Vec3::Zero();
    Vec3 init_gravity       = Vec3::Zero();


    ImuFullSolver solver;
};

// Owned by System. Global variable to enable easy access from different modules.
inline ImuStateSolver* imu_state_solver;


}  // namespace Snake
