/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#include "ImuStateSolver.h"

#include "saiga/vision/imu/all.h"

#include "Map/Map.h"
#include "Optimizer/GlobalBundleAdjustment.h"
#include "Optimizer/LocalBundleAdjustment.h"
#include "Optimizer/Simplification.h"
#include "Tracking/Tracking.h"

#if WITH_IMU
namespace Snake
{
ImuStateSolver::ImuStateSolver()
    : DelayedParallelMapOptimization("ImuStateSolver", true, 0, settings.async), Module(ModuleType::IMU_SOLVER)
{
    // imu_gravity.unit_gravity = Vec3(0,-9.81,0);

    if (settings.initial_bias_gyro.squaredNorm() > 0)
    {
        //        imu_bias_gyro               = settings.initial_bias_gyro;
        use_initial_gyro_bias_guess = true;
        current_gyro_weight         = settings.weight_gyro_optimization * settings.weight_gyro_initialization;
    }

    if (settings.initial_bias_acc.squaredNorm() > 0)
    {
        //        imu_bias_acc               = settings.initial_bias_acc;
        use_initial_acc_bias_guess = true;
    }

    acc_position_interpolation = 0.5;
    CreateTable({5, 7}, {"KF", "Time (ms)"});
}

ImuStateSolver::~ImuStateSolver() {}

void ImuStateSolver::ProcessNewKeyframe(Keyframe* kf)
{
    auto lock = map.LockFull();

    if (kf->previousKF == nullptr)
    {
        // The first keyframe is initialized with the bias presets.
        // (no integration has to be done)
        kf->velocity_and_bias.gyro_bias = settings.initial_bias_gyro;
        kf->velocity_and_bias.acc_bias  = settings.initial_bias_acc;
        kf->imu_state                   = 1;
        return;
    }


    // Initialize from previous KF and integrate state.
    kf->PreintegrateFromPrevious(true);
}

void ImuStateSolver::Process(Keyframe* kf)
{
    {
        auto timer = ModuleTimer();
        UpdateMap();
    }
    (*output_table) << kf->id() << LastTime();
}


void ImuStateSolver::UpdateMap()
{
    if (current_init_state == InitializationState::INITIALIZING_GYRO_BIAS)
    {
        InitGyroBias();
    }
    else if (current_init_state == InitializationState::INITIALIZING_GRAVITY_SCALE)
    {
        InitGravityAndScale();
    }
    else
    {
        auto keyframes       = map.GetAllKeyFrames();
        auto time_since_init = keyframes.back()->frame->timeStamp - init_done_time;

        if (time_since_init > 5 && init_optimize_state == 0)
        {
            acc_position_interpolation = 0;
            current_acc_weight         = 0.5 * settings.weight_acc_optimization;
            solver.Solve(0, 1, 1, 0, 1, true);
            RecomputeWeights();
            gba->FullBA(3, false);
            init_optimize_state++;
        }
        else if (time_since_init > 15 && init_optimize_state == 1)
        {
            // max_time_between_kf_map    = 1.1;
            acc_position_interpolation = 0.3;
            current_acc_weight         = settings.weight_acc_optimization;
            solver.Solve(0, 1, 1, 0, 1, false);
            RecomputeWeights();
            gba->FullBA(3, false);
            init_optimize_state++;
        }
        else if (time_since_init > 25 && init_optimize_state == 2)
        {
            if (1)
            {
                auto lock    = map.LockReadOnly();
                auto all_kfs = map.GetAllKeyFrames();
                for (auto kf : all_kfs)
                {
                    simplification->Add(kf);
                }
            }

            acc_position_interpolation = 0.5;
            solver.Solve(1, 1, 1, 0, 1, false);
            RecomputeWeights();
            gba->FullBA(1, false);
            init_optimize_state++;
        }
        else if (time_since_init > 50 && init_optimize_state == 3)
        {
            solver.Solve(1, 1, 1, 1, 1, false);
            RecomputeWeights();
            gba->FullBA(1, false);
            init_optimize_state++;
        }
        else if (time_since_init > 75 && init_optimize_state == 4)
        {
            solver.Solve(1, 1, 1, 1, 1, false);
            RecomputeWeights();
            gba->FullBA(1, false);
            init_optimize_state++;
        }
        else
        {
            solver.Solve(1, 1, 1, 1, 0, time_since_init < 5);
            RecomputeWeights();
        }
    }

}


void ImuStateSolver::RecomputeWeights()
{
    auto lock      = map.LockFull();
    auto keyframes = map.GetAllKeyFrames();
    for (int i = 0; i < keyframes.size(); ++i)
    {
        auto& kf = keyframes[i];
        kf->PreintegrateFromPrevious(false);


        int e                      = std::min<int>(i, keyframes.size() - i - 1);
        double w                   = clamp(e * 0.3, 0, 1);
        w                          = 1;
        kf->rpc.weight_translation = w;
        kf->rpc.weight_rotation    = 1;
        //        std::cout << i << " " << *kf << " " << e << " " << w << std::endl;
    }
}



bool ImuStateSolver::InitGyroBias()
{
    bool remove_kfs          = true;
    bool use_robust_function = true;

    std::vector<Imu::ImuPosePair> solver_data;
    std::vector<SE3> imu_poses;
    std::vector<Imu::Preintegration> preints;
    int N = 0;


    std::vector<Keyframe*> keyframes;
    {
        auto lock = map.LockReadOnly();

        // Remove border keyframes because they are usually tracked with less precision.
        int border = 0;
        keyframes  = map.GetAllKeyFrames();
        if (keyframes.size() < 8 + border * 2)
        {
            return false;
        }
        keyframes.erase(keyframes.begin(), keyframes.begin() + border);
        for (int i = 0; i < border; ++i) keyframes.pop_back();


        N = keyframes.size();

        imu_poses.resize(N);
        preints.resize(N);


        for (int i = 0; i < N; ++i)
        {
            auto kf      = keyframes[i];
            imu_poses[i] = kf->PoseInv() * mono_intrinsics.camera_to_body;
        }


        for (int i = 1; i < N; ++i)
        {
            auto kf1 = keyframes[i - 1];
            auto kf2 = keyframes[i];

            SAIGA_ASSERT(!kf1->isBad());
            SAIGA_ASSERT(kf2->imudata.time_begin == kf1->frame->timeStamp &&
                         kf2->imudata.time_end == kf2->frame->timeStamp);
            SAIGA_ASSERT(kf2->imudata.complete());

            preints[i] = Imu::Preintegration(init_gyro_bias, Vec3::Zero());
            preints[i].IntegrateMidPoint(kf2->imudata, true);



            Imu::ImuPosePair ipp;
            ipp.pose1     = &imu_poses[i - 1];
            ipp.pose2     = &imu_poses[i];
            ipp.preint_12 = &preints[i];
            solver_data.push_back(ipp);
        }
    }

    constexpr double gyro_init_th = 0.008;
    min_error                     = std::min(min_error, gyro_init_th * 2);

    if (!use_robust_function)
    {
        min_error = 10000;
    }

    auto bias_error = Imu::SolveGlobalGyroBias(solver_data, min_error * 0.75);


    init_gyro_bias += bias_error.first;
    min_error = std::min(min_error, bias_error.second);


    if (remove_kfs && gyro_init_iterations > 3)
    {
        auto lock = map.LockFull();
        for (int i = 1; i < N; ++i)
        {
            auto kf1 = keyframes[i - 1];
            //            auto kf2 = keyframes[i];

            auto& ipp = solver_data[i - 1];
            auto th   = std::max(min_error * 1.5, gyro_init_th);
            auto err  = sqrt(ipp.chi2_residual);
            if (err > th)
            {
                std::cout << "Removing bad init keyframe: " << *kf1 << " Error: " << err << " > " << th << std::endl;
                kf1->SetBadFlag();
                gyro_init_iterations--;
            }
            else
            {
                break;
            }
        }
    }

    gyro_init_iterations++;
    std::cout << "Gyro It " << gyro_init_iterations << " (N=" << N << "). Bias: " << init_gyro_bias.transpose()
              << " RMSE: " << bias_error.second << std::endl;



    if (gyro_init_iterations > 15)
    {
        map_clear = true;
    }


    if ((gyro_init_iterations >= 7 && bias_error.second < gyro_init_th) || gyro_init_iterations > 30)
    {
        std::cout << "Gyro done. With Error: " << bias_error.second << std::endl;

        // Set bias of every keyframe
        auto lock      = map.LockFull();
        auto keyframes = map.GetAllKeyFrames();
        for (auto kf : keyframes)
        {
            kf->velocity_and_bias.gyro_bias = init_gyro_bias;
        }

        lock.unlock();

        RecomputeWeights();

        lock.lock();

        if (remove_kfs)
        {
            // from the beginning remove all outlier keyframes
            for (int i = 0; i < keyframes.size(); ++i)
            {
                auto kf1 = keyframes[i];
                auto kf2 = kf1->nextKF;
                if (kf2 == nullptr) continue;

                auto p1 = kf1->PoseInv() * mono_intrinsics.camera_to_body;
                auto p2 = kf2->PoseInv() * mono_intrinsics.camera_to_body;



                Vec3 res = kf2->preint.RotationalError(p1.so3(), p2.so3());
                auto th  = bias_error.second * 3;
                if (res.norm() > th)
                {
                    std::cout << "Removing bad init keyframe: " << *kf1 << " Error: " << res.norm() << " > " << th
                              << std::endl;
                    kf1->SetBadFlag();
                }
                else
                {
                    break;
                }

                //            std::cout << kf1->id() << " -> " << kf2->id() << ": " << res.norm() << std::endl;
            }
        }


        lock.unlock();

        current_gyro_weight = settings.weight_gyro_optimization;


        // solver.FillSolverFromMap(false);
        // solver.decoupled_scene.chi2Print(0);
        //     solver.Solve(1, 0, 0, 0, 0, use_initial_gyro_bias_guess);
        gba->FullBA(3, true);
        current_init_state = InitializationState::INITIALIZING_GRAVITY_SCALE;

        return true;
    }

    return false;
}



bool ImuStateSolver::InitGravityAndScale()
{
    auto lock = map.LockReadOnly();


    // Remove border keyframes because they are usually tracked with less precision.
    int border     = 1;
    auto keyframes = map.GetAllKeyFrames();

    if (keyframes.size() < border * 2 + 3) return false;

    keyframes.erase(keyframes.begin(), keyframes.begin() + border);
    for (int i = 0; i < border; ++i) keyframes.pop_back();

    int N = keyframes.size();

    std::vector<SE3> imu_poses(N);

    for (int i = 0; i < N; ++i)
    {
        SE3 p = keyframes[i]->PoseInv();
        p.translation() *= init_scale;


        imu_poses[i] = p;
    }


    std::vector<Imu::ImuPoseTriplet> data(N - 2);
    for (int i = 0; i < N - 2; ++i)
    {
        Imu::ImuPoseTriplet ipt;
        ipt.pose1     = &imu_poses[i];
        ipt.pose2     = &imu_poses[i + 1];
        ipt.pose3     = &imu_poses[i + 2];
        ipt.preint_12 = &(keyframes[i + 1]->preint);
        ipt.preint_23 = &(keyframes[i + 2]->preint);
        data[i]       = ipt;
    }



    // Preintegrate with the estimated gyro bias (except the first kf)
    for (int i = 1; i < keyframes.size(); ++i)
    {
        auto kf    = keyframes[i];
        kf->preint = Imu::Preintegration(kf->velocity_and_bias.gyro_bias, init_acc_bias);
        kf->preint.IntegrateMidPoint(kf->imudata, true);
    }

    // Initial Graviity  solver, because the bias-gravity solution only computes delta rotations, which are not
    // valid for large differences.
    if (acc_init_iterations == 0)
    {
        init_gravity = Imu::SolveScaleGravityLinear(data, mono_intrinsics.camera_to_body).second;
    }

    double scale_delta;
    Vec3 bias_delta;
    double error;
    std::tie(scale_delta, init_gravity, bias_delta, error) =
        Imu::SolveScaleGravityBiasLinear(data, init_gravity, mono_intrinsics.camera_to_body);


    init_acc_bias += bias_delta;
    init_scale *= scale_delta;


    if (settings.inputType != InputType::Mono)
    {
        init_scale = 1;
    }


    acc_init_iterations++;
    std::cout << "Acc It " << acc_init_iterations << " (N=" << N << "). Scale: " << init_scale
              << " Gravity: " << init_gravity.transpose() << " Bias: " << init_acc_bias.transpose()
              << " RMSE: " << error << std::endl;

    if (acc_init_iterations == 1)
    {
        lock.unlock();
        Quat q = Quat::FromTwoVectors(init_gravity, imu_gravity.unit_gravity);
        map.Transform(SE3(q, Vec3::Zero()));
        return false;
    }

    if (acc_init_iterations >= 20)
    {
        auto keyframes = map.GetAllKeyFrames();
        for (auto kf : keyframes)
        {
            kf->velocity_and_bias.acc_bias = init_acc_bias;
        }


        lock.unlock();

        Quat q = Quat::FromTwoVectors(init_gravity, imu_gravity.unit_gravity);
        map.Transform(SE3(q, Vec3::Zero()), init_scale);
        imu_gravity.Set(imu_gravity.unit_gravity);
        tracking->SignalRescale(init_scale);

        // Compute velocities
        solver.Solve(0, 0, 1, 0, 0, true);
        solver.Solve(0, 0, 1, 1, 1, true);

        gba->FullBA(1, false);
        current_init_state = InitializationState::OPTIMIZING;
        init_done_time     = keyframes.back()->frame->timeStamp;
        std::cout << "Acc done." << std::endl;
        return true;
    }
    return false;
}


void ImuStateSolver::IterateBaImu(int k)
{
    for (int i = 0; i < 10; ++i)
    {
        solver.Solve(1, 1, 1, 1, 0, false);
        RecomputeWeights();
        gba->FullBA(1, false);
    }
    solver.Solve(1, 1, 1, 0, 1, false);
    for (int i = 0; i < 10; ++i)
    {
        solver.Solve(1, 1, 1, 0, 0, false);
        RecomputeWeights();
        gba->FullBA(1, false);
    }
}

void ImuStateSolver::clear()
{
    acc_init_iterations  = 0;
    gyro_init_iterations = 0;

    current_gyro_weight = 0;
    current_acc_weight  = 0;
    //    imu_initialized     = false;
    current_init_state = InitializationState::INITIALIZING_GYRO_BIAS;
}



}  // namespace Snake
#endif
