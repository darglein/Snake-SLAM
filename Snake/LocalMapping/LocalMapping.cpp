/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#include "LocalMapping.h"

#include "saiga/core/imgui/imgui.h"
#include "saiga/core/util/table.h"
#include "saiga/vision/reconstruction/Epipolar.h"
#include "saiga/vision/reconstruction/Triangulation.h"
#include "saiga/vision/util/DepthmapPreprocessor.h"
#include "saiga/vision/util/Random.h"

#include "IMU/ImuStateSolver.h"
#include "LoopClosing/LoopClosing.h"
#include "Map/Map.h"
#include "NeighbourSearch.h"
#include "Optimizer/GlobalBundleAdjustment.h"
#include "Optimizer/LocalBundleAdjustment.h"


namespace Snake
{
LocalMapping::LocalMapping() : Module(ModuleType::KEYFRAME_INSERTION)
{
    recentMapPoints.reserve(10000);


    CreateTable({5, 7}, {"KF", "Time (ms)"});
}

LocalMapping::~LocalMapping() {}


void LocalMapping::Process(Keyframe* kf)
{
    if (!kf)
    {
        return;
    }

    {
        auto timer = ModuleTimer();

        ProcessNewKeyFrame(kf);
        MapPointCulling(kf);
        CreateNewMapPoints(kf);


#if 0
        if (settings.inputType == InputType::RGBD)
        {
            auto f = kf->frame;
            depth_process.enqueue([f]() {
                DepthProcessor2::Settings depth_settings;
                depth_settings.gauss_radius     = 4;
                depth_settings.hyst_min         = 7;
                depth_settings.hyst_max         = 9;
                depth_settings.cameraParameters = StereoCamera4(rgbd_intrinsics.stereoCamera()).cast<float>();
                DepthProcessor2 dp(depth_settings);

                dp.Process(f->depth_image.getImageView());
            });
        }
#endif


        MapSearchParams params;

        params.fuse_threshold =
            settings.inputType == InputType::Mono ? reprojectionErrorThresholdMono : reprojectionErrorThresholdStereo;
        params.fuse_threshold *= 1.2;

        params.feature_error        = 50;
        params.two_match_factor     = 2.0;
        params.only_older_keyframes = false;
        params.num_threads          = settings.num_tracking_threads;
        map_search.Process(params, kf);
    }

    (*output_table) << kf->id() << LastTime();


    if (has_imu)
    {
        imu_state_solver->ProcessNewKeyframe(kf);
    }


    lba->Add(kf);
    lba->Update();
    //    gba->FullBA(3);

    simplification->Add(kf);
    simplification->Update();

    deferredMapper->Add(kf);
    deferredMapper->Update();

    loopClosing->Add(kf);
    loopClosing->Update();

    if (has_imu)
    {
        imu_state_solver->Add(kf);
        imu_state_solver->Update();
    }

    //    kf->ComputeDenseCandiates();

    //    kf->frame->mvpMapPoints = kf->GetMapPointMatches();


    UpdateViewer();
}  // namespace Snake



void LocalMapping::setLocalMapInitial()
{
    //    recentMapPoints
    std::vector<int> mps;
    map.GetAllMapPoints(mps);
    for (auto i : mps)
    {
        auto mp = &map.getMapPoint(i);
        recentMapPoints.push_back(mp);
    }
}


void LocalMapping::UpdateViewer()
{
    if (viewer)
    {
        static std::future<void> result;
        result = std::future<void>();
        result = globalThreadPool->enqueue([]() { viewer->setMap(std::make_unique<ViewerMap>()); });
    }
}

void LocalMapping::ProcessNewKeyFrame(KeyFrame* kf)
{
    SAIGA_ASSERT(kf);


    CHECK_VALID_MAP;
    auto lock = map.LockFull();

    {
        //        kf->GetMapPointMatches(tmpPoints);
        // project kf to global space
        kf->SetPose(kf->poseFromRef());
    }


    auto& tmpPoints = kf->MapPointRef();
    // Associate MapPoints to the new keyframe and update normal and descriptor
    for (int i = 0; i < (int)tmpPoints.size(); i++)
    {
        MapPoint* mp = tmpPoints[i];
        if (!mp) continue;

        if (mp->isBad())
        {
            // the mappoint was invalidated somewhere between tracking and here.
            // -> remove it also from the keyframe.
            kf->setMapPoint(nullptr, i);
        }

        auto oldId = mp->GetIndexInKeyFrame(kf);

        if (oldId == -1)
        {
            // add connection
            mp->AddObservation(kf, i);
            mp->UpdateNormal();
        }
        else
        {
            if (oldId == i)
            {
                // the connection already exists
                // can currenlty only happen for init points
            }
            else
            {
                // there is already a connection but linking to a different keypoint
                // -> erase the connection with the larger feature error
                auto& pdesc = mp->descriptor;
                auto& desc1 = kf->frame->descriptors[i];
                auto& desc2 = kf->frame->descriptors[oldId];

                auto dis1 = distance(pdesc, desc1);
                auto dis2 = distance(pdesc, desc2);

                if (dis1 < dis2)
                {
                    // the new point wins
                    // -> replace
                    kf->EraseMapPointMatch(mp);
                    kf->addMappoint(mp, i);
                    mp->RelinkObservation(kf, oldId, i);
                }
                else
                {
                    // the old point wins
                    // remove the current link
                    kf->setMapPoint(nullptr, i);
                }
            }
        }
        mp->ComputeDistinctiveDescriptors();
    }



    if (settings.inputType != InputType::Mono)
    {
        // Insert stereo points
        auto pinv = kf->Pose().inverse();

        std::unique_lock l(map.mMutexMap);
        auto frame = kf->frame;

        for (auto i : frame->featureRange())
        {
            if (!frame->hasDepth(i)) continue;
            if (frame->mvpMapPoints[i]) continue;
            MapPoint& mp = map.allocateMapPoint();
            auto z       = frame->depth[i];
            auto wp      = K.unproject(frame->undistorted_keypoints[i].point, z);
            mp.init(*kf, wp);
            mp.descriptor = frame->descriptors[i];


            if (z > th_depth)
            {
                mp.far_stereo_point = true;
            }
            mp.UpdateNormalLocal();
            mp.UpdateDepthLocal(i);
            mp.position            = pinv * mp.position;
            mp.normal              = pinv.so3() * mp.normal;
            frame->mvpMapPoints[i] = &mp;
            frame->mvbOutlier[i]   = false;
            map.AddMapPoint(mp);
            kf->addMappoint(&mp, i);
            mp.AddObservation(kf, i);
            recentMapPoints.push_back(&mp);
        }
    }

    {
        // Update links in the Covisibility Graph
        kf->UpdateConnections();
        if (!kf->inMap)
        {
            map.AddKeyFrame(*kf);
            kf->inMap = true;
        }
    }
}

void LocalMapping::MapPointCulling(KeyFrame* kf)
{
    int minMatches     = (settings.inputType == InputType::Mono) ? 2 : 3;
    int erasedPoints   = 0;
    int erasedByObs    = 0;
    int acceptedPoints = 0;
    int erasedLate     = 0;

    {
        auto lock = map.LockFull();
        recentMapPoints.erase(std::remove_if(recentMapPoints.begin(), recentMapPoints.end(),
                                             [&](MapPoint* mp) {
                                                 SAIGA_ASSERT(mp);
                                                 if (mp->isBad())
                                                 {
                                                     return false;
                                                 }

                                                 // time in keyframes since creation
                                                 auto timeSinceCreation = kf->id() - mp->mnFirstKFid;
                                                 auto foundRatio        = mp->GetFoundRatio();
                                                 auto observations      = mp->GetNumObservations();

                                                 if (foundRatio < 0.25f)
                                                 {
                                                     mp->SetBadFlag();
                                                     erasedPoints++;
                                                     erasedByObs++;
                                                     return true;
                                                 }
                                                 else if (timeSinceCreation >= 2 && observations <= minMatches)
                                                 {
                                                     mp->SetBadFlag();
                                                     erasedPoints++;
                                                     erasedLate++;
                                                     return true;
                                                 }
                                                 else if (timeSinceCreation >= 3)
                                                 {
                                                     acceptedPoints++;
                                                     return true;
                                                 }
                                                 return false;
                                             }),
                              recentMapPoints.end());
    }
}



void LocalMapping::CreateNewMapPoints(KeyFrame* kf)
{
    TriangulationParams params;
    params.errorMono           = reprojectionErrorThresholdMono;
    params.errorStereo         = reprojectionErrorThresholdStereo;
    params.epipolarDistance    = 4;
    params.feature_distance    = 50;
    params.only_past_keyframes = false;
    params.num_neighbors       = 10;
    params.num_threads         = settings.num_tracking_threads;
    triangulator.Process(params, kf, &recentMapPoints);
    TEST_MAP_SYNC;
}



}  // namespace Snake
