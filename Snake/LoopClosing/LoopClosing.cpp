/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */
//#define ENABLE_MAP_SYNC_TEST
#include "LoopClosing.h"

#include "saiga/core/imgui/imgui.h"

#include "LocalMapping/LocalMapping.h"
#include "LoopORBMatcher.h"
#include "Optimizer/GlobalBundleAdjustment.h"
#include "Tracking/Tracking.h"

namespace Snake
{
LoopClosing::LoopClosing()
    : DelayedParallelMapOptimization("LoopClosing", lc_enable, lc_delay, settings.async),
      Module(ModuleType::LOOP_CLOSING)

{
    CreateTable({4, 7, 7, 7, 7, 7}, {"KF", "Cands.", "Mat.", "Inl.", "Found", "Time (ms)"});
}

LoopClosing::~LoopClosing() {}


void LoopClosing::Process(KeyFrame* kf)
{
    if (!lc_enable)
    {
        return;
    }
    if (kf->id() < last_loop_close_kf + 4)
    {
        keyFrameDB->Add(kf);
        return;
    }


    Loop loop;

    {
        auto timer = ModuleTimer();
        loop       = loopDetector.Detect(kf);
    }

    (*output_table) << kf->id() << loop.candidates << loop.matches << loop.inliers << loop.found_loop << LastTime();

    if (loop.found_loop)
    {
        TEST_MAP_SYNC;
        CorrectLoop(loop);
        last_loop_close_kf = kf->id();
        TEST_MAP_SYNC;
    }
    keyFrameDB->Add(kf);
}

static void setpause()
{
    return;
    pause = true;
    while (pause)
    {
        localMapping->UpdateViewer();
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
}

void LoopClosing::CorrectLoopold(const Loop& loop)
{
    std::cout << "  Loop: " << *loop.source_keyframe << " -> " << *loop.target_keyframe << " (Scale=" << loop.scale
              << ")" << std::endl;

    loop.source_keyframe->debug_flag = 1;
    loop.target_keyframe->debug_flag = 1;

    setpause();

    if (settings.async)
    {
        simplification->Pause();
    }

#if REF_SWITCH
    tracking->trackingReferenceSwitch.source_keyframe    = loop.source_keyframe;
    tracking->trackingReferenceSwitch.target_keyframe    = loop.target_keyframe;
    tracking->trackingReferenceSwitch.need_switch_coarse = true;
    tracking->trackingReferenceSwitch.need_switch_fine   = true;

    if (settings.async)
    {
        while (tracking->trackingReferenceSwitch.need_switch_coarse)
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
#endif

    if (loop.source_keyframe->isBad() || loop.target_keyframe->isBad())
    {
        simplification->Resume();
        std::cout << "One of the loop keyframes were removed." << std::endl;
        return;
    }

    if (settings.async)
    {
        simplification->WaitUntilPaused();
    }

    std::map<Keyframe*, std::pair<SE3, DSim3>> transformed_poses;
    transformed_poses[loop.source_keyframe] = {loop.source_keyframe->PoseInv(), loop.T_w_correctSource};

    std::set<Keyframe*> constant_kfs = {
        loop.source_keyframe,
        loop.target_keyframe,
    };
    //    constant_kfs.insert();

    ConstructPGO(constant_kfs, transformed_poses, loop.scale == 1.0);
    OptimizeEssentialGraph();

    setpause();

    TEST_MAP_SYNC;
    gba->PointBA();
    TEST_MAP_SYNC;

    setpause();
    std::vector<Keyframe*> source_keyframe_list;
    std::vector<Keyframe*> target_keyframe_list;

    {
        auto lock            = map.LockFull();
        source_keyframe_list = loop.source_keyframe->GetVectorCovisibleKeyFrames();
        source_keyframe_list.push_back(loop.source_keyframe);
        SearchAndFuse(source_keyframe_list, loop.target_map_points);

        target_keyframe_list = loop.target_keyframe->GetVectorCovisibleKeyFrames();
        target_keyframe_list.push_back(loop.target_keyframe);
        SearchAndFuse(target_keyframe_list, loop.source_map_points);


        std::cout << "[LoopClosing] remove " << map.removeOutliers(50, 50) << " outliers" << std::endl;


        for (auto kf : source_keyframe_list)
        {
            kf->UpdateConnections();
        }
        for (auto kf : target_keyframe_list)
        {
            kf->UpdateConnections();
        }
        map.mapState++;
    }
    setpause();

    gba->FullBA(3, true);

    for (auto kf : source_keyframe_list)
    {
        deferredMapper->Add(kf);
    }
    for (auto kf : target_keyframe_list)
    {
        deferredMapper->Add(kf);
    }

    if (settings.async)
    {
        simplification->Resume();
    }
    setpause();
    std::cout << "===== Loop Closing done. =====" << std::endl;

    TEST_MAP_SYNC;
}

void LoopClosing::CorrectLoop(const Loop& loop)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    std::cout << "  Loop: " << *loop.source_keyframe << " -> " << *loop.target_keyframe << " (Scale=" << loop.scale
              << ")"
              << " Points Source/Target: " << loop.source_map_points.size() << " " << loop.target_map_points.size()
              << std::endl;


    if (IsParallel())
    {
        tracking->BarrierWait();
    }
    loop.source_keyframe->debug_flag = 1;
    loop.target_keyframe->debug_flag = 1;



    if (settings.async)
    {
        simplification->Pause();
    }

    if (loop.source_keyframe->isBad() || loop.target_keyframe->isBad())
    {
        simplification->Resume();
        std::cout << "One of the loop keyframes were removed." << std::endl;
        return;
    }

    if (settings.async)
    {
        simplification->WaitUntilPaused();
    }


    // Collect all never keyframes
    std::set<Keyframe*> kfs_around_source;

    for (auto kf = loop.source_keyframe->nextKF; kf; kf = kf->nextKF)
    {
        kf->debug_flag = 2;
        kfs_around_source.insert(kf);
    }
#if 0
    // collect a few older keyframes
    int count = 0;
    for (auto kf = loop.source_keyframe->previousKF; kf && count < 1; kf = kf->previousKF, ++count)
    {
        kf->debug_flag = 2;
        kfs_around_source.insert(kf);
    }
#endif
    kfs_around_source.insert(loop.source_keyframe);

    setpause();


    std::map<Keyframe*, std::pair<SE3, DSim3>> transformed_poses;

    std::cout << "rms start " << map.ReprojectionStats().rms << std::endl;
    {
        auto lock = map.LockFull();


        for (auto kf : kfs_around_source)
        {
            SE3 prev_pose = kf->PoseInv();
            DSim3 tp      = loop.T_target_source * DSim3(prev_pose);

            transformed_poses[kf] = {prev_pose, tp};

            kf->Transform(loop.T_target_source.se3(), loop.T_target_source.scale());

            //            auto con = kf->GetConnectedKeyFrames();
            //            for (auto c : con)
            //            {
            //                source_connections.insert(c);
            //            }
        }


        setpause();

        //        for (auto kf : source_connections)
        //        {
        //            kf->constant = true;
        //            kf->debug_flag = 2;
        //        }

        //        for (auto kf : kfs_around_source)
        //        {
        //            kf->reprojectionErrorDebug();
        //        }

        setpause();

#if 0
        // Start Loop Fusion
        // Update matched map points and replace if duplicated
        for (size_t i = 0; i < loop.mvpCurrentMatchedPoints.size(); i++)
        {
            if (loop.mvpCurrentMatchedPoints[i])
            {
                MapPoint* pLoopMP = loop.mvpCurrentMatchedPoints[i];
                MapPoint* pCurMP  = loop.source_keyframe->GetMapPoint(i);
                if (pCurMP)
                    pCurMP->Replace(pLoopMP);
                else
                {
                    loop.source_keyframe->addMappoint(pLoopMP, i);
                    pLoopMP->AddObservation(loop.source_keyframe, i);
                    pLoopMP->ComputeDistinctiveDescriptors();
                }
            }
        }
        loop.source_keyframe->UpdateConnections();
#endif
        // tracking->SignalRescale(loop.scale);
    }



    std::set<Keyframe*> constant_kfs;
    constant_kfs.insert(loop.source_keyframe);
    constant_kfs.insert(loop.target_keyframe);
    ConstructPGO(constant_kfs, transformed_poses, loop.scale == 1.0);

#if 1

    {
        auto lock = map.LockFull();

        LoopORBmatcher matcher;
        int totalFused = 0;
        for (auto kf : kfs_around_source)
        {
            totalFused += matcher.Fuse(kf, loop.target_map_points, 5);
            //            kf->reprojectionErrorDebug();
        }
        for (auto kf : kfs_around_source)
        {
            kf->UpdateConnections();
        }
        std::cout << "Fused points target->source " << totalFused << std::endl;
    }
    std::cout << "rms transform " << map.ReprojectionStats().rms << std::endl;
#endif


    setpause();



    OptimizeEssentialGraph();


    if (IsParallel())
    {
        // Todo move this one up
        tracking->BarrierResume();
    }

    //    for (auto kf : kfs_around_source)
    //    {
    //        auto lock = map.LockFull();
    //        kf->reprojectionErrorDebug();
    //    }
    std::cout << "rms after pgo " << map.ReprojectionStats().rms << std::endl;

    setpause();

    {
        auto lock = map.LockReadOnly();
        std::vector<int> keyframesi, pointsi;
        map.GetAllKeyFrames(keyframesi);
        map.GetAllMapPoints(pointsi);
        //        for (auto i : keyframesi)
        //        {
        //            KeyFrame* kf = &map.getKeyframe(i);
        //        }
        for (auto i : pointsi)
        {
            MapPoint* mp         = &map.getMapPoint(i);
            mp->constant         = false;
            mp->loop_transformed = false;
        }
    }

    TEST_MAP_SYNC;
    gba->PointBA(4, true);

    //    for (auto kf : kfs_around_source)
    //    {
    //        auto lock = map.LockFull();
    //        kf->reprojectionErrorDebug();
    //    }

    //    {
    //        auto lock = map.LockFull();
    //        std::vector<int> keyframesi, pointsi;
    //        map.GetAllKeyFrames(keyframesi);
    //        map.GetAllMapPoints(pointsi);
    //        for (auto i : keyframesi)
    //        {
    //            KeyFrame* kf = &map.getKeyframe(i);
    //            kf->UpdateConnections();
    //        }
    //    }

    std::cout << "rms after point ba " << map.ReprojectionStats().rms << std::endl;
    TEST_MAP_SYNC;

    setpause();
    std::vector<Keyframe*> source_keyframe_list;
    std::vector<Keyframe*> target_keyframe_list;

    {
        auto lock            = map.LockFull();
        source_keyframe_list = loop.source_keyframe->GetVectorCovisibleKeyFrames();
        source_keyframe_list.push_back(loop.source_keyframe);
        SearchAndFuse(source_keyframe_list, loop.target_map_points);

        target_keyframe_list = loop.target_keyframe->GetVectorCovisibleKeyFrames();
        target_keyframe_list.push_back(loop.target_keyframe);
        SearchAndFuse(target_keyframe_list, loop.source_map_points);


        std::cout << "[LoopClosing] remove " << map.removeOutliers(50, 50) << " outliers" << std::endl;


        for (auto kf : source_keyframe_list)
        {
            kf->UpdateConnections();
        }
        for (auto kf : target_keyframe_list)
        {
            kf->UpdateConnections();
        }
        map.mapState++;
    }
    setpause();

    gba->FullBA(3, true);

    for (auto kf : source_keyframe_list)
    {
        deferredMapper->Add(kf);
    }
    for (auto kf : target_keyframe_list)
    {
        deferredMapper->Add(kf);
    }


    if (settings.async)
    {
        simplification->Resume();
    }
    setpause();
    std::cout << "===== Loop Closing done. =====" << std::endl;

    TEST_MAP_SYNC;
}



void LoopClosing::SearchAndFuse(const std::vector<KeyFrame*>& source_keyframes, const std::vector<MapPoint*>& points)
{
    LoopORBmatcher matcher;
    int totalFused = 0;
    for (auto kf : source_keyframes)
    {
        totalFused += matcher.Fuse(kf, points, 5);
    }
    std::cout << "[LoopClosing::SearchAndFuse] Fused Points: " << totalFused << std::endl;
}



}  // namespace Snake
