/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */
//#define ENABLE_MAP_SYNC_TEST
#include "GlobalBundleAdjustment.h"

#include "saiga/core/imgui/imgui.h"
#include "saiga/core/util/table.h"
#include "saiga/vision/recursive/BAPointOnly.h"
#include "saiga/vision/recursive/BAPoseOnly.h"

#include "Map/Map.h"

namespace Snake
{
GlobalBundleAdjustment::GlobalBundleAdjustment() : Module(ModuleType::OTHER)
{
    outliers.reserve(maxFeatures);
    keyFrames.reserve(maxKeyframes);
    mapPoints.reserve(maxPoints);

    perKeyframeData.resize(maxKeyframes);

    kfmapinv.reserve(maxKeyframes);
    mpmapinv.reserve(maxPoints);

    scene.reserve(maxKeyframes, maxPoints, maxFeatures);


    global_op_options.debugOutput            = false;
    global_op_options.maxIterations          = 4;
    global_op_options.maxIterativeIterations = 40;
    global_op_options.iterativeTolerance     = 1e-10;
    global_op_options.solverType             = OptimizationOptions::SolverType::Iterative;
    global_op_options.numThreads             = 1;

    global_op_options_points = global_op_options;

    global_ba_options.huberMono      = reprojectionErrorThresholdMono;
    global_ba_options.huberStereo    = reprojectionErrorThresholdStereo;
    global_ba_options.helper_threads = 1;
}


void GlobalBundleAdjustment::FullBA(int iterations, bool remove_outliers)
{
    std::unique_lock gba_l(gba_lock);
    OptimizationResults res;
    float time = 0;
    {
        ScopedTimer tim(time);
        MakeGlobalScene();

        //        CeresBA cba;
        cba.create(scene);
#if USE_TIGHT_SOLVER
        cba.full_solver_data = solver;
#endif
        global_op_options.maxIterations = iterations;
        cba.optimizationOptions         = global_op_options;
        cba.baOptions                   = global_ba_options;
        res                             = cba.initAndSolve();
#if USE_TIGHT_SOLVER
        solver = cba.full_solver_data;
#endif
        UpdateGlobalScene(remove_outliers);
    }

    std::cout << "[GBA Full] " << res.cost_initial << " -> " << res.cost_final << " in " << time << " ms.  ("
              << res.total_time << ")" << std::endl;
}

void GlobalBundleAdjustment::FullBARel(int iterations, bool remove_outliers)
{
    std::unique_lock gba_l(gba_lock);
    OptimizationResults res;
    float time = 0;
    {
        ScopedTimer tim(time);
        MakeGlobalScene();

        BARecRel cba;
        cba.create(scene);
#if USE_TIGHT_SOLVER
        cba.full_solver_data = solver;
#endif
        global_op_options.maxIterations = iterations;
        cba.optimizationOptions         = global_op_options;
        cba.baOptions                   = global_ba_options;
        res                             = cba.initAndSolve();
#if USE_TIGHT_SOLVER
        solver = cba.full_solver_data;
#endif
        UpdateGlobalScene(remove_outliers);
    }

    std::cout << "[GBA Full] " << res.cost_initial << " -> " << res.cost_final << " in " << time << " ms.  ("
              << res.total_time << ")" << std::endl;
}

void GlobalBundleAdjustment::PointBA(int iterations, bool remove_outliers)
{
    std::unique_lock gba_l(gba_lock);
    OptimizationResults res;

    float time = 0;
    {
        ScopedTimer tim(time);
        MakeGlobalScene();
        BAPointOnly sepBA;
        global_op_options_points.maxIterations = iterations;
        sepBA.optimizationOptions              = global_op_options_points;
        sepBA.baOptions                        = global_ba_options;
        sepBA.create(scene);
        res = sepBA.initAndSolve();
        UpdateGlobalScene(remove_outliers);
    }

    std::cout << "[GBA Point] " << res.cost_initial << " -> " << res.cost_final << " in " << time << " ms.  ("
              << res.total_time << ")" << std::endl;
}
void GlobalBundleAdjustment::RealignIntermiediateFrames(bool ceres, bool with_imu)
{
    SAIGA_ASSERT(ceres == false);
    std::unique_lock gba_l(gba_lock);
    auto lock = map.LockFull();

    map.GetAllKeyFrames(keyframesi);
    map.GetAllMapPoints(pointsi);

    if (keyframesi.empty()) return;

    scene.clear();
    if (settings.inputType != InputType::Mono)
    {
        SAIGA_ASSERT(stereo_cam.fx != 1);
        scene.bf = stereo_cam.bf;
    }

    IntrinsicsPinholed intr = K;
    scene.intrinsics.push_back(intr);

    int maxKFid = 0;


    std::map<Frame*, int> kfmap;
    std::map<int, Frame*> kfmapinv;
    std::map<MapPoint*, int> mpmap;
    kfmap.clear();
    kfmapinv.clear();
    mpmap.clear();
    mpmapinv.clear();



    auto first_kf = map.GetAllKeyFrames().front();
    auto last_kf  = map.GetAllKeyFrames().back();

    //   - collect all frames using the prev pointer and starting at the last keyframe
    std::vector<FramePtr> all_frames = map.GetAllFrames();
    std::vector<FramePtr> snake_frames;


    for (auto f : all_frames)
    {
        if (f->id < first_kf->frame->id || f->id > last_kf->frame->id)
        {
            f->validPose = false;
            continue;
        }

        snake_frames.push_back(f);
    }


    for (auto i : pointsi)
    {
        MapPoint* pMP = &map.getMapPoint(i);

        int wpid       = scene.worldPoints.size();
        mpmap[pMP]     = wpid;
        mpmapinv[wpid] = pMP;
        scene.worldPoints.push_back(WorldPoint());
        WorldPoint& wp = scene.worldPoints.back();
        wp.p           = pMP->getPosition();
        wp.constant    = true;

        int nEdges = 0;
        wp.valid   = nEdges > 0;
    }

    for (auto f : snake_frames)
    {
        int internalId = scene.images.size();

        SceneImage si;
        si.intr        = 0;
        si.validPoints = 0;
        si.se3         = f->getPoseFromReference();

        if (f->isKeyframe)
        {
            si.constant = true;
        }

        scene.images.push_back(si);

        kfmap[f.get()]       = internalId;
        kfmapinv[internalId] = f.get();



        if (f->id > maxKFid) maxKFid = f->id;


        // Add links
        for (int i : f->featureRange())
        {
            if (f->mvbOutlier[i]) continue;
            auto mp = f->mvpMapPoints[i];
            if (!mp || mp->isBad()) continue;
            auto wpid = mpmap[mp];
            auto& wp  = scene.worldPoints[wpid];

            auto& kpUn   = f->undistorted_keypoints[i];
            auto depth   = f->depth[i];
            float weight = scalePyramid.InverseSquaredScale(kpUn.octave);
            {
                StereoImagePoint ip;
                ip.point  = kpUn.point;
                ip.depth  = depth;
                ip.wp     = wpid;
                ip.weight = weight;

                auto& img = scene.images[internalId];
                wp.stereoreferences.emplace_back(internalId, img.stereoPoints.size());
                img.stereoPoints.push_back(ip);
                img.validPoints++;
            }
        }
    }



#if WITH_IMU

    if (has_imu && with_imu)
    {
        if (current_gyro_weight > 0)
        {
            for (int i = 1; i < snake_frames.size(); ++i)
            {
                FramePtr f1 = snake_frames[i - 1];
                FramePtr f2 = snake_frames[i];


                auto& imu_data = f2->imu_data;
                if (f1->timeStamp != imu_data.time_begin || f2->timeStamp != imu_data.time_end)
                {
                    //                std::cout << "Skip gyro constraint (realignint.) " << *kf1 << " - " << *kf2 <<
                    //                std::endl;
                    continue;
                }

                auto ref = f1->referenceKF();

                Imu::Preintegration preint(ref->velocity_and_bias);
                preint.IntegrateMidPoint(imu_data, false);

                SO3 delta_R_body   = preint.delta_R;
                SO3 delta_R_camera = (mono_intrinsics.camera_to_body.so3() * delta_R_body *
                                      mono_intrinsics.camera_to_body.so3().inverse());
                delta_R_camera     = delta_R_camera.inverse();



                RelPoseConstraint rpc;
                rpc.img1            = kfmap[f1.get()];
                rpc.img2            = kfmap[f2.get()];
                rpc.rel_pose        = SE3(delta_R_camera, Vec3::Zero());
                rpc.weight_rotation = current_gyro_weight / (f2->timeStamp - f1->timeStamp);

                SAIGA_ASSERT(std::isfinite(rpc.weight_rotation));
                rpc.weight_translation = 0;

                scene.images[rpc.img1].rel_constraints++;
                scene.images[rpc.img2].rel_constraints++;

                scene.rel_pose_constraints.push_back(rpc);
            }
        }
    }

#endif

    //    SAIGA_ASSERT(scene);

    //    scene.save("scene_gba.scene");

    {
        //        SAIGA_BLOCK_TIMER();

        {
            BAPoseOnly sepBA;
            sepBA.optimizationOptions               = global_op_options;
            sepBA.baOptions                         = global_ba_options;
            sepBA.optimizationOptions.maxIterations = 4;
            sepBA.optimizationOptions.debug         = false;
            sepBA.optimizationOptions.debugOutput   = false;
            sepBA.create(scene);
            auto res = sepBA.initAndSolve();
            //            std::cout << "RealignIntermiediateFrames " << res.cost_initial << " -> " << res.cost_final <<
            //            std::endl;
        }
    }


    for (auto f : snake_frames)
    {
        auto iid     = kfmap[f.get()];
        auto SE3quat = scene.images[iid].se3;
        f->tmpPose   = SE3quat;
        f->UpdateRelFromTmpPose();

        f->removeOutliers(reprojectionErrorThresholdMono, reprojectionErrorThresholdStereo);
    }
}

void GlobalBundleAdjustment::MakeGlobalScene()
{
    //    bool use_right = false;
    //    std::shared_lock l(map.mutexUpdate);
    auto lock = map.LockReadOnly();
    map.GetAllKeyFrames(keyframesi);
    map.GetAllMapPoints(pointsi);


    if (keyframesi.empty()) return;

    scene.clear();
    if (settings.inputType != InputType::Mono)
    {
        SAIGA_ASSERT(stereo_cam.fx != 1);
        scene.bf = stereo_cam.bf;
    }

    IntrinsicsPinholed intr = K;
    scene.intrinsics.push_back(intr);

    //    kfmap.clear();
    kfmapinv.clear();
    //    mpmap.clear();
    mpmapinv.clear();

    for (auto i : keyframesi)
    {
        KeyFrame* kf = &map.getKeyframe(i);
        if (kf->isBad()) continue;

        int internalId = scene.images.size();

        SceneImage si;
        si.intr        = 0;
        si.se3         = kf->Pose();
        si.constant    = false;
        si.validPoints = 0;
        scene.images.push_back(si);

        perKeyframeData[kf->id()].id_in_scene = internalId;

        kfmapinv.push_back(kf);
    }

    // Make last keyframe constant
    // -> Tracking doesn't get lost if BA is executed in parallel
    scene.images.back().constant = true;

    for (auto i : pointsi)
    {
        MapPoint* mp = &map.getMapPoint(i);

        int wpid = scene.worldPoints.size();
        //        mpmap[pMP] = wpid;

        mpmapinv.push_back(mp);
        scene.worldPoints.push_back(WorldPoint());
        WorldPoint& wp = scene.worldPoints.back();
        wp.p           = mp->getPosition();
        wp.constant    = mp->constant;
        SAIGA_ASSERT(!wp.constant);

        int nEdges = 0;

        // Set edges
        for (auto obs : mp->GetObservationList())
        {
            KeyFrame* pKFi = obs.first;
            if (pKFi->isBad()) continue;

            nEdges++;
            auto& kpUn = pKFi->frame->undistorted_keypoints[obs.second];

            float weight = scalePyramid.InverseSquaredScale(kpUn.octave);
            //            int iid      = kfmap[pKFi];
            int iid = perKeyframeData[pKFi->id()].id_in_scene;
            {
                StereoImagePoint ip;
                ip.point  = kpUn.point;
                ip.depth  = pKFi->frame->depth[obs.second];
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

    if (has_imu)
    {
        for (int i = 1; i < keyframesi.size(); ++i)
        {
            KeyFrame* kf1 = &map.getKeyframe(keyframesi[i - 1]);
            KeyFrame* kf2 = &map.getKeyframe(keyframesi[i]);

            if (has_imu)
            {
                auto& imu_data = kf2->imudata;
                if (kf1->frame->timeStamp != imu_data.time_begin || kf2->frame->timeStamp != imu_data.time_end)
                {
                    std::cout << "Skip gyro constraint (gba) " << *kf1 << " - " << *kf2 << std::endl;
                    continue;
                }
                SAIGA_ASSERT(kf1->frame->timeStamp == imu_data.time_begin &&
                             kf2->frame->timeStamp == imu_data.time_end);
            }
            if (!kf2->preint_valid)
            {
                continue;
            }

            RelPoseConstraint rpc = kf2->rpc;


            SAIGA_ASSERT(rpc.img1 == kf1->id());
            SAIGA_ASSERT(rpc.img2 == kf2->id());


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

void GlobalBundleAdjustment::UpdateGlobalScene(bool remove_outliers)
{
    //    std::unique_lock l(map.mutexUpdate);
    auto lock = map.LockFull();

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


    if (remove_outliers)
    {
        auto c = map.removeOutliers(reprojectionErrorThresholdMono * reprojectionErrorThresholdMono,
                                    reprojectionErrorThresholdStereo * reprojectionErrorThresholdStereo);
        std::cout << "Removed Observations: " << c << std::endl;
    }
    map.mapState++;
}



}  // namespace Snake
