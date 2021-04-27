/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once


#include "saiga/vision/imu/all.h"

#include "SnakeGlobal.h"
#define IMU_SOLVE_IN_BODY_SPACE 1
namespace Snake
{
class ImuFullSolver
{
   public:
    void Solve(bool bg, bool ba, bool v, bool g, bool s, bool add_initial_edge);

    std::vector<Keyframe*> keyframes_in_solver;

    Imu::DecoupledImuScene::SolverOptions solver_options;
    Imu::DecoupledImuScene decoupled_scene;
    void FillSolverFromMap(bool add_initial_edge);
    void UpdateStateFromSolver();


    void imgui();

    Imu::Preintegration dummy_preint;
    Imu::ImuSequence dummy_sequence;
};


}  // namespace Snake
