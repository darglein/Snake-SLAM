/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#include "Map.h"

#include "saiga/core/imgui/imgui.h"
#include "saiga/core/util/FileSystem.h"
#include "saiga/core/util/table.h"
#include "saiga/vision/camera/TimestampMatcher.h"
#include "saiga/vision/icp/ICPAlign.h"
#include "saiga/vision/slam/Trajectory.h"
#include "saiga/vision/util/Random.h"

#include "Eigen/Eigenvalues"
#include "IMU/ImuStateSolver.h"
#include "Module.h"


namespace Snake
{
Map::Map(int maxKeyframes, int maxPoints) : allKeyframes(maxKeyframes), allPoints(maxPoints)
{
    pointValid.resize(maxPoints, false);
    keyframeValid.resize(maxKeyframes, false);

    for (int i = 0; i < maxKeyframes; ++i)
    {
        allKeyframes.emplace_back(i);
    }

    for (int i = 0; i < maxPoints; ++i)
    {
        allPoints.emplace_back(i);
    }

    //    VLOG(3) << "[Map] Created.";
    //    VLOG(3) << "Max Keyframes: " << allKeyframes.capacity();
    //    VLOG(3) << "Max Points: " << allPoints.capacity();
    //    std::cout << "~Memory for Map: " << double(allKeyframes.memory() + allPoints.memory()) / (1000 * 1000) << "mb"
    //              << std::endl;
}

void Map::Clear()
{
    std::cout << "clear map." << std::endl;
    std::unique_lock lock2(mutexUpdate);

#if WITH_IMU
    imu_state_solver->clear();
#endif

    std::vector<int> keyframes;
    GetAllKeyFrames(keyframes);

    for (auto i : keyframes)

    {
        auto& kf = allKeyframes[i];
        kf.SetBadFlag();
    }
}

void Map::Transform(const SE3& t, double scale)
{
    auto lock = LockFull();
    mapState++;
    total_scale *= scale;
    std::vector<int> keyframes, points;
    GetAllKeyFrames(keyframes);
    GetAllMapPoints(points);

    for (auto i : points)
    {
        auto& mp = allPoints[i];
        mp.Transform(t, scale);
    }

    for (auto i : keyframes)
    {
        auto& kf = allKeyframes[i];
        if (!kf.inMap) continue;
        kf.Transform(t, scale);
    }
}


SE3 Map::crazyMove()
{
    if (map.KeyFramesInMap() <= 3)
    {
        return SE3();
    }


    SE3 worldTrans;
    worldTrans = Random::randomSE3();
    worldTrans.translation() *= 10.0;

    Transform(worldTrans);

    return worldTrans;
}

void Map::RemoveRandomPoint()
{
    std::unique_lock lock(mutexUpdate);

    CHECK_VALID_MAP;

    if (countPoints == 0) return;

    auto& mp = getMapPoint(Saiga::Random::uniformInt(0, countPoints - 1));

    if (!mp.isBad())
    {
        std::cout << "remove random point" << std::endl;
        mp.SetBadFlag();
    }
    CHECK_VALID_MAP;
}

void Map::RemoveRandomKeyframe()
{
    std::unique_lock lock(mutexUpdate);

    CHECK_VALID_MAP;

    if (countKeyframes == 0) return;

    if (Random::sampleDouble(0, 1) > 0.1) return;

    auto& kf = getKeyframe(Saiga::Random::uniformInt(0, countKeyframes - 1));

    if (!kf.isBad())
    {
        std::cout << "remove random kf " << kf << std::endl;
        kf.SetBadFlag();
    }
    CHECK_VALID_MAP;
}

void Map::RemoveRandomObservation()
{
    std::unique_lock lock(mutexUpdate);

    CHECK_VALID_MAP;

    if (countKeyframes == 0) return;

    if (Random::sampleDouble(0, 1) > 0.1) return;

    auto& kf = getKeyframe(Saiga::Random::uniformInt(0, countKeyframes - 1));

    for (int i = 0; i < 10; ++i)
    {
        int idx = Random::uniformInt(0, kf.frame->N - 1);

        auto mp = kf.GetMapPoint(idx);

        if (mp)
        {
            kf.EraseMapPointMatch(idx);
            mp->EraseObservation(&kf);
        }
    }
    CHECK_VALID_MAP;
}



void Map::printStatistics(std::ostream& strm, bool align, bool rescale)
{
    auto lock = LockReadOnly();
    if (validKeyframes == 0 || validPoints == 0)
    {
        std::cout << "Map is empty. Can't print statistics." << std::endl;
        return;
    }

    Trajectory::TrajectoryType A, B;

    int kfCount      = 0;
    double chi2      = 0;
    int observations = 0;

    std::vector<int> keyframes, points;
    GetAllKeyFrames(keyframes);
    GetAllMapPoints(points);

    for (auto i : keyframes)
    {
        auto& kf = allKeyframes[i];
        //        auto kf = &allKeyframes[kfid];
        if (kf.isBad()) continue;

        auto [chi, cou] = kf.reprojectionError();
        chi2 += chi;
        observations += cou;
        kfCount++;

        if (!kf.frame->groundTruth.has_value()) continue;
        A.emplace_back(kf.id(), kf.Pose().inverse());
        B.emplace_back(kf.id(), kf.frame->groundTruth.value());

        //        std::cout << kf.frame->id << " " << kf.MatchCount() << std::endl;
    }


    int pointCount = 0;
    for (auto i : points)
    {
        auto& mp = allPoints[i];
        if (mp.isBad()) continue;
        pointCount++;
    }

    Keyframe* firstValidKf = &allKeyframes[keyframes.front()];
    for (auto kf : keyframes)
    {
        if (allKeyframes[kf].id() < firstValidKf->id())
        {
            firstValidKf = &allKeyframes[kf];
        }
    }

    strm << std::endl;
    Table table({20, 10}, strm);
    table << "[Map]"
          << "";
    table << "Keyframes" << std::to_string(kfCount) + "/" + std::to_string(countKeyframes);
    table << "Points" << std::to_string(pointCount) + "/" + std::to_string(countPoints);
    table << "First KF: " << *firstValidKf;

    table << "Observatons " << observations;
    table << "Obs/Kf" << double(observations) / kfCount;
    table << "Obs/Point" << double(observations) / pointCount;
    table << "Rep Chi2" << chi2;
    table << "Rep  RMS" << sqrt(chi2 / observations);



    if (A.size() > 0)
    {
        if (align)
        {
            auto scale_error = Trajectory::align(A, B, rescale);
            for (auto& se : A) se.second = se.second * mono_intrinsics.camera_to_gt;
            Trajectory::align(A, B, rescale);
            strm << "Scale Error: " << scale_error.second << std::endl;
        }


        int rpe_diff = 1;
        auto rpe     = Trajectory::rpe(A, B, rpe_diff);
        auto ate     = Trajectory::ate(A, B);

        int index = std::max_element(ate.begin(), ate.end()) - ate.begin();
        std::cout << "worst kf frame: " << A[index].first << " with error = " << ate[index] << std::endl;


        if (0)
        {
            std::ofstream rel_file("eval/rel.csv", std::ios::app);
            for (auto r : rpe)
            {
                rel_file << r << ",";
            }
            rel_file << std::endl;
        }
        if (0)
        {
            static int asdf = 0;
            if (asdf <= 1)
            {
                std::ofstream rel_file("eval/eval_pp.csv", std::ios::app);

                rel_file << Statistics(ate).rms << ",";


                if (asdf == 1)
                {
                    rel_file << std::endl;
                }
                asdf++;
            }
        }



        Statistics srpe(rpe);
        Statistics sate(ate);

        strm << "Translation Error (in meters)" << std::endl;
        strm << sate << std::endl;

        //        strm << "Relative Translation Error (in meters)" << std::endl;
        //        strm << srpe << std::endl;
        //        strm << "Rotation Error (in degrees)" << std::endl;
        //        strm << Statistics(Trajectory::are(A, B)) << std::endl;
    }
}

void Map::printTrajectory(bool rescale)
{
    //    std::shared_lock lock(mutexUpdate);
    if (validKeyframes == 0 || validPoints == 0)
    {
        std::cout << "Map is empty. Can't print statistics." << std::endl;
        return;
    }

    Trajectory::TrajectoryType A, B, B_full;


    auto lock   = LockReadOnly();
    auto frames = GetReallyAllFrames();
    for (auto frame : frames)
    {
        if (!frame->groundTruth.has_value()) continue;
        B_full.emplace_back(frame->id, frame->groundTruth.value());

        if (!frame->validPose) continue;

        auto kf = frame->referenceKF();
        if (!kf || kf->isBad()) continue;

        if (frame->trackingInliers < 10) continue;
        // std::cout << "inl " << frame->trackingInliers << std::endl;
        frame->tmpPose = frame->getPoseFromReference();
        A.emplace_back(frame->id, frame->tmpPose.inverse());
        B.emplace_back(frame->id, frame->groundTruth.value());
    }


    if (A.size() > 0)
    {
        auto scale_error = Trajectory::align(A, B, rescale);
        for (auto& se : A) se.second = se.second * mono_intrinsics.camera_to_gt;
        Trajectory::align(A, B, rescale);

        auto ate = Trajectory::ate(A, B);

        std::cout << "Scale Error: " << scale_error.second << std::endl;
        std::cout << "Align Error:" << std::endl;
        std::cout << Statistics(ate) << std::endl;

        //        if (1)
        //        {
        //            // save x/z trajectory
        //            std::ofstream out_file("traj_xz.csv");
        //            out_file << "x,z" << std::endl;
        //            for (int i = 0; i < A.size(); ++i)
        //            {
        //                out_file << A[i].second.translation().x() << "," << A[i].second.translation().z() <<
        //                std::endl;
        //            }

        //            std::ofstream out_file_gt("traj_gt_xz.csv");
        //            out_file_gt << "x,z" << std::endl;
        //            for (int i = 0; i < B.size(); ++i)
        //            {
        //                out_file_gt << B[i].second.translation().x() << "," << B[i].second.translation().z() <<
        //                std::endl;
        //            }

        //            std::cout << "Saved xz trajectory" << std::endl;
        //        }

        {
            // save x/y trajectory
            std::ofstream out_file("traj_xy.csv");
            out_file << "x,y" << std::endl;
            for (int i = 0; i < A.size(); ++i)
            {
                out_file << A[i].second.translation().x() << "," << A[i].second.translation().y() << std::endl;
            }

            std::ofstream out_file_gt("traj_gt_xy.csv");
            out_file_gt << "x,y" << std::endl;
            for (int i = 0; i < B.size(); ++i)
            {
                out_file_gt << B[i].second.translation().x() << "," << B[i].second.translation().y() << std::endl;
            }


            std::ofstream out_file_gt_full("traj_gt_xy_full.csv");
            out_file_gt_full << "x,y" << std::endl;
            for (int i = 0; i < B_full.size(); ++i)
            {
                out_file_gt_full << B_full[i].second.translation().x() << "," << B_full[i].second.translation().y()
                                 << std::endl;
            }
            std::cout << "Saved xy trajectory" << std::endl;
        }
    }
}

Map::ReprojectionError Map::ReprojectionStats()
{
    auto lock = LockReadOnly();
    ReprojectionError result;

    std::vector<int> keyframes, points;
    GetAllKeyFrames(keyframes);
    GetAllMapPoints(points);

    for (auto p : points)
    {
        auto& mp = allPoints[p];
        if (mp.isBad()) continue;
        result.num_points++;
    }

    for (auto i : keyframes)
    {
        auto& kf = allKeyframes[i];
        if (kf.isBad()) continue;
        auto [chi, cou] = kf.reprojectionError();
        result.chi2 += chi;
        result.num_observations += cou;
        result.num_keyframes++;
    }
    result.rms                    = sqrt(result.chi2 / result.num_observations);
    result.num_inserted_keyframes = countKeyframes;
    result.num_inserted_points    = countPoints;
    return result;
}

Map::TrajError Map::TrajectoryError(bool solve_scale)
{
    auto lock = LockReadOnly();

    TrajError result;

    if (validKeyframes == 0 || validPoints == 0)
    {
        std::cout << "Map is empty. Can't print statistics." << std::endl;
        return result;
    }

    Trajectory::Scene scene;
    std::vector<int> keyframes, points;
    GetAllKeyFrames(keyframes);
    GetAllMapPoints(points);


    for (auto i : keyframes)
    {
        auto& kf = allKeyframes[i];
        if (kf.isBad()) continue;

        if (!kf.frame->groundTruth.has_value()) continue;


        Trajectory::Observation obs;

        obs.estimate     = kf.Pose().inverse();
        obs.ground_truth = kf.frame->groundTruth.value();
        scene.vertices.push_back(obs);
    }

    if (scene.vertices.empty())
    {
        return result;
    }

    scene.extrinsics     = mono_intrinsics.camera_to_gt;
    scene.optimize_scale = solve_scale;

    scene.InitialAlignment();

#ifdef SAIGA_USE_CERES
     scene.OptimizeCeres();
#endif


    //    std::cout << "Oopt extr. " << scene.extrinsics << std::endl;
    result.ate_rmse    = scene.rmse();
    result.scale_error = scene.scale;
    return result;
}

int Map::removeOutliers(double thresholdMono, double thresholdStereo)
{
    int count = 0;

    std::vector<int> keyframes;
    GetAllKeyFrames(keyframes);


    for (auto i : keyframes)
    {
        auto& kf = allKeyframes[i];
        if (kf.isBad()) continue;
        count += kf.removeOutliers(thresholdMono, thresholdStereo);
    }
    return count;
}


bool Map::valid()
{
    //    ASSERT_HAS_READ_LOCK;
    std::shared_lock lock(mutexUpdate);
    std::vector<int> keyframes, points;
    GetAllKeyFrames(keyframes);
    GetAllMapPoints(points);


    // Keyframes only observe valid mappoints
    for (auto i : keyframes)
    {
        auto kf = &allKeyframes[i];
        if (kf->isBad()) continue;
        if (!kf->inMap) continue;

        SAIGA_ASSERT(kf->IsValid());

        auto mps = kf->GetMapPointMatches();
        for (auto pi : kf->frame->featureRange())
        {
            SAIGA_ASSERT(!mps[pi] || !mps[pi]->isBad(), "A keyframe has a pointer to a bad mappoint.");
        }
    }

    for (auto p : points)
    {
        auto mp = &allPoints[p];


        int obs_count = 0;
        for (auto obs : mp->GetObservationList())
        {
            auto kf  = obs.first;
            auto idx = obs.second;
            SAIGA_ASSERT(kf->GetMapPoint(idx) == mp);

            if (kf->frame->depth[idx] > 0)
            {
                obs_count += 2;
            }
            else
            {
                obs_count += 1;
            }
        }

        SAIGA_ASSERT(obs_count == mp->GetNumObservations());


        if (settings.inputType == InputType::Mono)
        {
            SAIGA_ASSERT(mp->GetNumObservations() == (int)mp->GetObservationList().size());
        }
        else
        {
        }
    }

    // Points should be either invalid or have a valid reference.
    for (auto p : points)
    {
        auto mp = &allPoints[p];
        if (mp->isBad()) continue;
        KeyFrame* ref_kf = mp->referenceKF;
        SAIGA_ASSERT(ref_kf && !ref_kf->isBad());
    }



    for (auto i : keyframes)
    {
        auto kf = &allKeyframes[i];
        if (kf->isBad()) continue;
        if (!kf->inMap) continue;

        for (auto pi : kf->frame->featureRange())
        {
            auto mp = kf->GetMapPoint(pi);
            if (!mp || mp->isBad()) continue;

            auto idx = mp->GetIndexInKeyFrame(kf);
            if (idx != pi)
            {
                std::cout << "Map Error: " << i << " " << idx << " " << pi << std::endl;
                return false;
            }
        }
    }
    return true;
}

void Map::writeKeyframesToFile(const std::string& dir)
{
    std::filesystem::create_directory(dir);

    std::vector<int> keyframes;
    GetAllKeyFrames(keyframes);

    for (auto i : keyframes)
    {
        auto& kf = allKeyframes[i];
        if (kf.isBad()) continue;
        auto img = kf.frame->image;


        std::string name = to_string(kf.id()) + "_" + to_string(kf.frame->id);
        img.save(dir + "/" + name + ".png");
    }
}

// void Map::UpdateIntermidieateFramePoses() {}

void Map::imgui()
{
    if (ImGui::Button("Scale 2"))
    {
        Transform(SE3(), 2.0);
    }

    if (ImGui::Button("Scale 0.5"))
    {
        Transform(SE3(), .5);
    }

    if (ImGui::Button("Map Statistics"))
    {
        performance_stats.PrintStatistics();
    }

    if (ImGui::Button("Map Statistics No Align"))
    {
        printStatistics(std::cout, false);
    }


    if (ImGui::Button("writeKeyframesToFile"))
    {
        writeKeyframesToFile();
    }
}

}  // namespace Snake
