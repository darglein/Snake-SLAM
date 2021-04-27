/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#include "FullSolver.h"

#include "saiga/core/imgui/imgui.h"

#include "Map/Map.h"
#include "Optimizer/GlobalBundleAdjustment.h"
#include "Tracking/Tracking.h"
namespace Snake
{
void ImuFullSolver::FillSolverFromMap(bool add_initial_edge)
{
    auto lock = map.LockReadOnly();

    decoupled_scene.Clear();
    keyframes_in_solver.clear();

    keyframes_in_solver = map.GetAllKeyFrames();

    // remove all keyframe which have no valid imu state
    keyframes_in_solver.erase(std::remove_if(keyframes_in_solver.begin(), keyframes_in_solver.end(),
                                             [](auto kf) { return kf->imu_state == 0; }),
                              keyframes_in_solver.end());

    int N = keyframes_in_solver.size();


    for (auto kf : keyframes_in_solver)
    {
        SAIGA_ASSERT(kf->imu_state);
    }
    decoupled_scene.gravity = imu_gravity;
    decoupled_scene.scale   = 1;

    //    decoupled_scene.weight_change_a = 10;
    //    decoupled_scene.weight_change_g = 10;


    for (int i = 0; i < N; ++i)
    {
        auto kf = keyframes_in_solver[i];

        if (!kf->preint_valid)
        {
            kf->PreintegrateFromPrevious(false);
        }

        Imu::NavState s;
#if IMU_SOLVE_IN_BODY_SPACE
        s.pose = kf->PoseInv() * mono_intrinsics.camera_to_body;
#else
        s.pose = keyframes_in_solver[i]->PoseInv();
#endif
        s.time              = kf->frame->timeStamp;
        s.velocity_and_bias = kf->velocity_and_bias;
        decoupled_scene.states.push_back(s);
    }


    if (add_initial_edge)
    {
        // Initial
        dummy_sequence.time_begin = decoupled_scene.states.front().time;
        dummy_sequence.time_end   = dummy_sequence.time_begin + 1.0;
        dummy_sequence.data.clear();



        Imu::Data s1;
        s1.omega.setZero();
        s1.acceleration.setZero();
        s1.timestamp = dummy_sequence.time_begin;

        Imu::Data s2;
        s2.omega.setZero();
        s2.acceleration.setZero();
        s2.timestamp = dummy_sequence.time_end;

        dummy_sequence.data.push_back(s1);
        dummy_sequence.data.push_back(s2);

        //        std::cout << dummy_sequence << std::endl;

        dummy_preint = Imu::Preintegration();
        dummy_preint.IntegrateMidPoint(dummy_sequence, false);

        //        std::cout << dummy_preint.delta_R.unit_quaternion() << std::endl;
        //        std::cout << dummy_preint.delta_x.transpose() << std::endl;

        //        std::cout << dummy_preint.delta_R.unit_quaternion() << std::endl;
        //        std::cout << dummy_preint.delta_x.transpose() << std::endl;
        //        std::cout << std::endl;
        //        exit(0);

        // Add initial bias edge
        Imu::NavState initial_state;
        initial_state                             = decoupled_scene.states.front();
        initial_state.velocity_and_bias.gyro_bias = settings.initial_bias_gyro;
        initial_state.velocity_and_bias.acc_bias  = settings.initial_bias_acc;

        //        initial_state.velocity_and_bias.gyro_bias = Vec3(0.2, -0.2, 0);
        //        initial_state.velocity_and_bias.acc_bias  = -Vec3(0.2, -0.2, 0);

        //        std::cout << "Add initial (gyro/acc) " << initial_state.velocity_and_bias.gyro_bias.transpose() << " /
        //        "
        //                  << initial_state.velocity_and_bias.acc_bias.transpose() << std::endl;

        initial_state.constant = true;
        initial_state.time     = dummy_sequence.time_end;
        decoupled_scene.states.push_back(initial_state);



        Imu::NavEdge initial_edge;
        initial_edge.from       = 0;
        initial_edge.to         = decoupled_scene.states.size() - 1;
        initial_edge.preint     = &dummy_preint;
        initial_edge.data       = &dummy_sequence;
        initial_edge.weight_pvr = 0;

        SAIGA_ASSERT(settings.initial_bias_gyro.squaredNorm() > 0);
        SAIGA_ASSERT(settings.initial_bias_acc.squaredNorm() > 0);


        decoupled_scene.edges.push_back(initial_edge);
    }


    for (int i = 1; i < N; ++i)
    {
        Imu::NavEdge e;
        e.data   = &keyframes_in_solver[i]->imudata;
        e.preint = &keyframes_in_solver[i]->preint;
        e.from   = i - 1;
        e.to     = i;

        double dt_preint = e.preint->delta_t;
        double dt_frames = decoupled_scene.states[e.to].time - decoupled_scene.states[e.from].time;
        SAIGA_ASSERT(std::abs(dt_preint - dt_frames) < 0.0001);

        if (dt_preint > 2)
        {
            e.weight_pvr = 0;
        }

        decoupled_scene.edges.push_back(e);
    }
    decoupled_scene.SanityCheck();

    //    solver.chi2Print(0);
    //    exit(0);
}

void ImuFullSolver::UpdateStateFromSolver()
{
    auto lock   = map.LockFull();
    imu_gravity = decoupled_scene.gravity;


    Vec3 mean_gyro = Vec3::Zero();
    Vec3 mean_acc  = Vec3::Zero();

    for (int i = 0; i < keyframes_in_solver.size(); ++i)
    {
        auto& s = decoupled_scene.states[i];
        auto kf = keyframes_in_solver[i];

        kf->velocity_and_bias = s.velocity_and_bias;
        kf->velocity_and_bias.acc_bias += s.delta_bias.acc_bias;
        kf->velocity_and_bias.gyro_bias += s.delta_bias.gyro_bias;

        mean_gyro += kf->velocity_and_bias.gyro_bias;
        mean_acc += kf->velocity_and_bias.acc_bias;


        // SAIGA_ASSERT(decoupled_scene.states[i].delta_bias.gyro_bias.squaredNorm() == 0);

        //        SAIGA_ASSERT(keyframes_in_solver[i]->velocity.squaredNorm() > 0);
    }

    mean_gyro /= keyframes_in_solver.size();
    mean_acc /= keyframes_in_solver.size();

    lock.unlock();
    if (decoupled_scene.scale != 1)
    {
        map.Transform(SE3(), decoupled_scene.scale);
        tracking->SignalRescale(decoupled_scene.scale);
    }
}

void ImuFullSolver::imgui()
{
    ImGui::InputDouble("weight_P", &decoupled_scene.weight_P);
    ImGui::InputDouble("weight_V", &decoupled_scene.weight_V);
    ImGui::InputDouble("weight_R", &decoupled_scene.weight_R);

    ImGui::InputDouble("weight_change_a", &decoupled_scene.weight_change_a);
    ImGui::InputDouble("weight_change_g", &decoupled_scene.weight_change_g);
}

void ImuFullSolver::Solve(bool bg, bool ba, bool v, bool g, bool s, bool add_initial_edge)
{
    if (settings.inputType != InputType::Mono)
    {
        s = false;
    }

    if (settings.initial_bias_acc.squaredNorm() == 0)
    {
        add_initial_edge = false;
    }

    FillSolverFromMap(add_initial_edge);


    solver_options.solver_flags = 0;

    if (ba) solver_options.solver_flags |= Imu::IMU_SOLVE_BA;
    if (bg) solver_options.solver_flags |= Imu::IMU_SOLVE_BG;
    if (v) solver_options.solver_flags |= Imu::IMU_SOLVE_VELOCITY;
    if (g) solver_options.solver_flags |= Imu::IMU_SOLVE_GRAVITY;
    if (s) solver_options.solver_flags |= Imu::IMU_SOLVE_SCALE;



    decoupled_scene.weight_R = 1000;
    decoupled_scene.weight_P = 100;
    decoupled_scene.weight_V = 10;


    if (!v)
    {
        decoupled_scene.weight_V = 0;
    }


    if (solver_options.solver_flags == Imu::IMU_SOLVE_BG)
    {
        decoupled_scene.weight_P = 0;
        decoupled_scene.weight_V = 0;
    }


    solver_options.use_global_bias              = false;
    solver_options.bias_recompute_delta_squared = std::numeric_limits<double>::infinity();
    solver_options.final_recompute              = false;


    {
        OptimizationOptions oopts;
        oopts.maxIterations = solver_options.max_its;
        oopts.debugOutput   = false;
        oopts.solverType    = OptimizationOptions::SolverType::Direct;

        // decoupled_scene.PreintAll();
        //    double before = solver.chi2();
        Imu::DecoupledImuSolver so;
        so.optimizationOptions = oopts;
        so.Create(decoupled_scene, solver_options);
        auto r = so.initAndSolve();
    }
    UpdateStateFromSolver();
}



}  // namespace Snake
