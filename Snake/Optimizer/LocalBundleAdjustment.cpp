/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */
//#define ENABLE_MAP_SYNC_TEST
#include "LocalBundleAdjustment.h"

#include "saiga/core/imgui/imgui.h"
#include "saiga/core/util/table.h"
#include "saiga/vision/ceres/CeresBA.h"
#include "saiga/vision/ceres/CeresBARS.h"
#include "saiga/vision/g2o/g2oBA2.h"
#include "saiga/vision/recursive/BAPointOnly.h"
#include "saiga/vision/recursive/BAPoseOnly.h"

#include "Map/Map.h"

//#define BA_MP

namespace Snake
{
LocalBundleAdjustment::LocalBundleAdjustment()
    : DelayedParallelMapOptimization("LBA", lba_enable, 0, settings.async_lba), Module(ModuleType::OPTIMIZER)
{
    perKeyframeData.resize(maxKeyframes);
    perPointData.resize(maxPoints);

    outliers.reserve(maxFeatures);
    keyFrames.reserve(maxKeyframes);
    mapPoints.reserve(maxPoints);

    kfmapinv.reserve(100);
    mpmapinv.reserve(10000);


    scene.reserve(maxKeyframes, maxPoints, maxFeatures);
    statistics.reserve(maxKeyframes);

    //    cba.reserve(maxKeyframes, maxPoints);

    chi1Mono   = reprojectionErrorThresholdMono * lbaErrorFactor;
    chi1Stereo = reprojectionErrorThresholdStereo * lbaErrorFactor;

    chi2Mono                                = chi1Mono * chi1Mono;
    chi2Stereo                              = chi1Stereo * chi1Stereo;
    local_op_options.debugOutput            = false;
    local_op_options.maxIterations          = 3;
    local_op_options.maxIterativeIterations = 30;
    local_op_options.iterativeTolerance     = 1e-10;
    local_op_options.solverType             = OptimizationOptions::SolverType::Iterative;

#ifdef BA_MP
    local_op_options.numThreads = num_tracking_threads;
#else
    local_op_options.numThreads = 1;
#endif

    local_op_options.buildExplizitSchur = true;
    local_op_options.simple_solver      = true;

    local_ba_options.huberMono      = chi1Mono;
    local_ba_options.huberStereo    = chi1Stereo;
    local_ba_options.helper_threads = 1;



    CreateTable({5, 6, 7, 4, 10, 7}, {"KF", "Outl.", "Cost1", " -> ", "Cost2", "Time (ms)"});
    //    outConsole.setWriteToCout(true);
}

LocalBundleAdjustment::~LocalBundleAdjustment() {}


void LocalBundleAdjustment::lba(KeyFrame* kf)
{
    if (!kf || kf->isBad()) return;


    int outlierPoints;
    double cost_initial;
    double cost_final;

    {
        auto timer = ModuleTimer();
        MakeLocalScene(kf);
        std::tie(outlierPoints, cost_initial, cost_final) = SolveLocalScene(kf);
        UpdateLocalScene(kf);
    }

    (*output_table) << kf->id() << outlierPoints << cost_initial << " -> " << cost_final << LastTime();
}

void LocalBundleAdjustment::MakeLocalScene(KeyFrame* kf)
{
    //    std::unique_lock l(map.mutexUpdate);
    //    outConsole << "Running LBA for " << *kf << " " << kf->isBad() << std::endl;
    currentOptimizationId++;

    int maxKFid = 0;

    mapPoints.clear();
    kfmapinv.clear();
    mpmapinv.clear();
    scene.clear();


    TEST_MAP_SYNC;
    {
        //        std::shared_lock l(map.mutexUpdate);
        auto lock = map.LockReadOnly();

        if (map.KeyFramesInMap() < 2)
        {
            return;
        }
        stateBefore = map.mapState;



        keyFrames.clear();


        // 15 best neighbors
        tmp_keyframes.clear();
        kf->GetBestCovisibilityKeyFrames(tmp_keyframes, num_neighbor_keyframes);
        tmp_keyframes.push_back(kf);


        // + 15 last keyframes
        auto k = kf->previousKF;
        for (int i = 0; k && i < num_last_keyframes; k = k->previousKF, ++i)
        {
            tmp_keyframes.push_back(k);
        }

        // -> max 30 keyframes

        for (auto other_kf : tmp_keyframes)
        {
            auto& other_data = perKeyframeData[other_kf->id()];
            if (other_data.mnBALocalForKF == currentOptimizationId) continue;
            other_data.mnBALocalForKF = currentOptimizationId;

            if (!other_kf->isBad())
            {
                keyFrames.push_back(other_kf);
            }
        }

        std::sort(keyFrames.begin(), keyFrames.end(), [](auto a, auto b) { return a->id() < b->id(); });


        for (auto other_kf : keyFrames)
        {
            for (auto mp : other_kf->GetMapPointMatches())
            {
                if (!mp || mp->isBad()) continue;

                auto& data = perPointData[mp->id()];
                if (data.mnBALocalForKF != currentOptimizationId)
                {
                    mapPoints.push_back(mp);
                    data.mnBALocalForKF = currentOptimizationId;
                }
            }
        }

        // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
        std::vector<KeyFrame*> lFixedCameras;
        for (auto mp : mapPoints)
        {
            for (auto obs : mp->GetObservationList())
            {
                KeyFrame* kf     = obs.first;
                auto& other_data = perKeyframeData[kf->id()];
                if (other_data.mnBALocalForKF != currentOptimizationId &&
                    other_data.mnBAFixedForKF != currentOptimizationId)
                {
                    other_data.mnBAFixedForKF = currentOptimizationId;
                    if (!kf->isBad()) lFixedCameras.push_back(kf);
                }
            }
        }


        // =========== Build Saiga::Scene ============

        if (settings.inputType != InputType::Mono)
        {
            SAIGA_ASSERT(stereo_cam.fx != 1);
            scene.bf = stereo_cam.bf;
        }

        IntrinsicsPinholed intr = K;
        scene.intrinsics.push_back(intr);



        for (size_t i = 0; i < keyFrames.size(); i++)
        {
            auto* kf = keyFrames[i];

            SAIGA_ASSERT(kf && !kf->isBad());
            //            if (pKF->isBad()) continue;

            int internalId = scene.images.size();

            SceneImage si;
            si.intr        = 0;
            si.se3         = kf->Pose();
            si.validPoints = 0;
            scene.images.push_back(si);

            perKeyframeData[kf->id()].id_in_scene = internalId;
            kfmapinv.push_back(kf);



            if (kf->id() > maxKFid) maxKFid = kf->id();
        }

        for (size_t i = 0; i < lFixedCameras.size(); i++)
        {
            auto* pKF = lFixedCameras[i];
            if (pKF->isBad()) continue;

            int internalId = scene.images.size();

            SceneImage si;
            si.intr = 0;

            si.se3         = pKF->Pose();
            si.constant    = true;
            si.validPoints = 0;
            scene.images.push_back(si);

            perKeyframeData[pKF->id()].id_in_scene = internalId;
            kfmapinv.push_back(pKF);



            if (pKF->id() > maxKFid) maxKFid = pKF->id();
        }

        for (auto pMP : mapPoints)
        {
            int wpid                            = scene.worldPoints.size();
            perPointData[pMP->id()].id_in_scene = wpid;
            mpmapinv.push_back(pMP);
            scene.worldPoints.push_back(WorldPoint());
            WorldPoint& wp = scene.worldPoints.back();
            wp.p           = pMP->getPosition();
            wp.constant    = pMP->constant;

            int nEdges = 0;

            for (auto obs : pMP->GetObservationList())
            {
                KeyFrame* pKFi = obs.first;
                if (pKFi->isBad()) continue;

                nEdges++;

                auto& kpUn   = pKFi->frame->undistorted_keypoints[obs.second];
                float weight = scalePyramid.InverseSquaredScale(kpUn.octave);
                int iid      = perKeyframeData[pKFi->id()].id_in_scene;

                {
                    StereoImagePoint ip;
                    ip.point = kpUn.point;
                    ip.depth = pKFi->frame->depth[obs.second];

                    if (ip.depth > 0)
                    {
                        SAIGA_ASSERT(pKFi->frame->right_points[obs.second] > -500,
                                     std::to_string(pKFi->frame->right_points[obs.second]));
                        //                        ip.stereo_x = pKFi->frame->right_points[obs.second];
                    }

                    ip.wp     = wpid;
                    ip.weight = weight;

                    auto& img = scene.images[iid];

                    if (img.constant && wp.constant) continue;
                    wp.stereoreferences.emplace_back(iid, img.stereoPoints.size());
                    img.stereoPoints.push_back(ip);
                    img.validPoints++;
                }
            }
            wp.valid = nEdges > 0;
        }

#if WITH_IMU
        if (has_imu && current_gyro_weight > 0)
        {
            for (int i = 1; i < keyFrames.size(); ++i)
            {
                auto kf1 = keyFrames[i - 1];
                auto kf2 = keyFrames[i];

                auto& imu_data = kf2->imudata;
                if (kf1->frame->timeStamp != imu_data.time_begin || kf2->frame->timeStamp != imu_data.time_end)
                {
                    //                    std::cout << "Skip gyro constraint " << *kf1 << " - " << *kf2 << std::endl;
                    continue;
                }
                if (!kf2->preint_valid)
                {
                    continue;
                }
                RelPoseConstraint rpc = kf2->rpc;

                //                std::cout << kf2->id() << " preint bias: " << kf2->preint.GetBiasGyro().transpose() <<
                //                " "
                //                          << kf2->preint.GetBiasAcc().transpose() << std::endl;

                //                SAIGA_ASSERT(kf2->preint_valid);

                SAIGA_ASSERT(rpc.img1 == kf1->id());
                SAIGA_ASSERT(rpc.img2 == kf2->id());

                SAIGA_ASSERT(kf1->frame->timeStamp == imu_data.time_begin &&
                             kf2->frame->timeStamp == imu_data.time_end);

                rpc.img1 = perKeyframeData[kf1->id()].id_in_scene;
                rpc.img2 = perKeyframeData[kf2->id()].id_in_scene;


                double dt              = kf2->frame->timeStamp - kf1->frame->timeStamp;
                rpc.weight_rotation    = current_gyro_weight / dt;
                rpc.weight_translation = rpc.weight_translation * current_acc_weight / dt;

                if (dt > 2)
                {
                    rpc.weight_rotation    = 0;
                    rpc.weight_translation = 0;
                }

                scene.images[rpc.img1].rel_constraints++;
                scene.images[rpc.img2].rel_constraints++;

                scene.rel_pose_constraints.push_back(rpc);
            }
        }
#endif
    }

    //    std::cout << "LBA Scene: " << scene.images.size() << " " << scene.worldPoints.size() << std::endl;
}

std::tuple<int, double, double> LocalBundleAdjustment::SolveLocalScene(KeyFrame* kf)
{
    auto optOp = local_op_options;

    cba.optimizationOptions = local_op_options;
    cba.baOptions           = local_ba_options;
    cba.create(scene);

#ifdef BA_MP
    cba.initOMP();
    auto res = cba.solveOMP();
#else
    auto res                    = cba.initAndSolve();
#endif


    int outlierPoints = 0;
    {
        // Check inlier observations
        for (SceneImage& im : scene.images)
        {
            for (auto& o : im.stereoPoints)
            {
                if (!o) continue;
                if (o.depth > 0)
                {
                    Vec3 e = scene.residual3(im, o);
                    if (e.squaredNorm() > chi2Stereo)
                    {
                        o.outlier = true;
                        outlierPoints++;
                    }
                }
                else
                {
                    Vec2 e = scene.residual2(im, o);
                    if (e.squaredNorm() > chi2Mono)
                    {
                        o.outlier = true;
                        outlierPoints++;
                    }
                }
            }
        }
    }

    if (outlierPoints > 0)
    {
        // Removing a few outliers doesn't change much so 1 iteration should be enough
        optOp.maxIterations                   = 1;
        cba.optimizationOptions.maxIterations = 1;
#ifdef BA_MP
        auto res = cba.solveOMP();
#else
        auto res = cba.solve();
//            auto res = cba.solve(optOp, local_ba_options);
#endif
    }

    return {outlierPoints, res.cost_initial, res.cost_final};
}

void LocalBundleAdjustment::UpdateLocalScene(KeyFrame* kf)
{
    outliers.clear();
    {
        // Check inlier observations
        int outlierPoints = 0;

        //        for (SceneImage& im : scene.images)
        for (int iid = 0; iid < (int)scene.images.size(); ++iid)
        {
            auto& im = scene.images[iid];
            for (auto& o : im.stereoPoints)
            {
                if (o)
                {
                    if (o.depth > 0)
                    {
                        Vec3 e = scene.residual3(im, o);
                        if (e.squaredNorm() > chi2Stereo)
                        {
                            o.outlier = true;
                            outlierPoints++;
                        }
                    }
                    else
                    {
                        Vec2 e = scene.residual2(im, o);
                        if (e.squaredNorm() > chi2Mono)
                        {
                            o.outlier = true;
                            outlierPoints++;
                        }
                    }
                }

                if (o.outlier)
                {
                    auto kf = kfmapinv[iid];
                    auto mp = mpmapinv[o.wp];
                    outliers.emplace_back(kf, mp);
                }
            }
        }
        //            outConsole << "LBA Outliers: " << outlierPoints << std::endl;
    }



    {
        // Get Map Mutex
        //        VLOG(4) << "waiting for map mutex";
        //        std::unique_lock lock(map.mutexUpdate);
        auto lock = map.LockFull();


        if (stateBefore != map.mapState)
        {
            std::cout << "The map was modified during Local BA -> Dropping update..." << std::endl;
            return;
        }

        for (auto [kf, mp] : outliers)
        {
            kf->EraseMapPointMatch(mp);
            mp->EraseObservation(kf);
        }



        // Keyframes
        for (int i = 0; i < kfmapinv.size(); ++i)
        {
            KeyFrame* kf = kfmapinv[i];
            auto SE3quat = scene.images[i].se3;
            kf->SetPose(SE3quat);
        }

        // Points
        for (int i = 0; i < mpmapinv.size(); ++i)
        {
            MapPoint* pMP = mpmapinv[i];
            pMP->SetWorldPos(scene.worldPoints[i].p);
            pMP->UpdateNormal();
        }
    }
}



void LocalBundleAdjustment::printStatistics(std::ostream& strm)
{
    std::vector<double> timeInit, timeSolve, timeFinalize, timeTotal;


    int i = 0;
    for (auto s : statistics)
    {
        timeInit.push_back(s.timeInit);
        timeSolve.push_back(s.timeSolve);
        timeFinalize.push_back(s.timeFinalize);
        timeTotal.push_back(s.timeInit + s.timeSolve + s.timeFinalize);
        ++i;
    }

    Saiga::Statistics sti(timeInit);
    Saiga::Statistics sts(timeSolve);
    Saiga::Statistics stf(timeFinalize);
    Saiga::Statistics stt(timeTotal);

    strm << std::endl;
    strm << "[BundleAdjustment]" << std::endl;
    Saiga::Table table({16, 16, 16, 16}, strm);
    table << " "
          << "Mean"
          << "Min"
          << "Max";
    table << "Time Init (ms)" << sti.mean << sti.min << sti.max;
    table << "Time Solve (ms)" << sts.mean << sts.min << sts.max;
    table << "Time Final (ms)" << stf.mean << stf.min << stf.max;
    table << "Time Total (ms)" << stt.mean << stt.min << stt.max;

    strm << "Keyframes Per Second: " << 1000.0 / stt.mean << std::endl;
}


}  // namespace Snake
