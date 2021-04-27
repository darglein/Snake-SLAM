/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#include "saiga/vision/ceres/CeresPGO.h"
#include "saiga/vision/recursive/PGORecursive.h"
#include "saiga/vision/recursive/PGOSim3Recursive.h"
#include "saiga/vision/scene/PoseGraph.h"
#include "saiga/vision/util/Random.h"

#include "LoopClosing.h"
namespace Snake
{
void LoopClosing::ConstructPGO(std::set<Keyframe*> constant_kfs,
                               const std::map<Keyframe*, std::pair<SE3, DSim3>>& transformed_poses, bool fix_scale)
{
    pg = PoseGraph();
    per_keyframe_data.clear();

    pg.fixScale = fix_scale;


    {
        auto lock = map.LockReadOnly();

        per_keyframe_data.resize(map.MaxNumberOfKeyframes());

        std::vector<int> keyframes, points;
        map.GetAllKeyFrames(keyframes);
        map.GetAllMapPoints(points);


        // Set pose of all keyframes to their current global pose.
        for (auto i : keyframes)
        {
            KeyFrame* kf = &map.getKeyframe(i);
            if (kf->isBad()) continue;

            int internalId = pg.vertices.size();

            LC_PerKeyframeData& data = per_keyframe_data[i];
            data.index_in_pg         = internalId;
            data.kf                  = kf;

            auto it = transformed_poses.find(kf);

            if (it == transformed_poses.end())
            {
                data.before_T_w_i = kf->Pose().inverse();
            }
            else
            {
                data.before_T_w_i = it->second.first;
            }

            PoseVertex v;
            //            v.T_w_i    = DSim3(kf->Pose(), 1.0).inverse();
            v.T_w_i = DSim3(data.before_T_w_i);

            //            v.constant = kf == target_kf || kf == source_kf;
            if (constant_kfs.find(kf) != constant_kfs.end())
            {
                v.constant = true;
            }
            else
            {
                v.constant = false;
            }


            pg.vertices.push_back(v);
        }

        // Add all visibility edges using the current pose.

        std::set<std::pair<int, int>> basic_edges;
        for (auto i : keyframes)
        {
            KeyFrame* kf = &map.getKeyframe(i);
            if (kf->isBad()) continue;
            int ii = per_keyframe_data[kf->id()].index_in_pg;

            // Always insert the parent
            KeyFrame* parent = kf->GetParent();
            if (parent && !parent->isBad())
            {
                int jj = per_keyframe_data[parent->id()].index_in_pg;
                basic_edges.insert({std::min(ii, jj), std::max(ii, jj)});
            }

            // Covisibility graph edges
            auto connections = kf->GetCovisiblesByWeight(20);
            for (auto other : connections)
            {
                if (other->isBad()) continue;
                int jj = per_keyframe_data[other->id()].index_in_pg;
                basic_edges.insert({std::min(ii, jj), std::max(ii, jj)});
            }
        }

        for (auto p : basic_edges)
        {
            pg.AddVertexEdge(p.first, p.second, 1.0);
        }
    }
    pg.sortEdges();

    // Now we transform the new_kf and add the loop edges
    for (auto kf_pose : transformed_poses)
    {
        auto source_index = per_keyframe_data[kf_pose.first->id()].index_in_pg;
        pg.vertices[source_index].SetPose(kf_pose.second.second);
    }
    //    auto source_index = per_keyframe_data[source_kf->id()].index_in_pg;
    //    pg.vertices[source_index].SetPose(T_w_correctedSource);
}

void LoopClosing::OptimizeEssentialGraph()
{
    {
        Saiga::OptimizationOptions pgoo;
        pgoo.debugOutput   = false;
        pgoo.maxIterations = 50;
        pgoo.minChi2Delta  = 1e-10;
        pgoo.solverType    = OptimizationOptions::SolverType::Direct;
        //        SAIGA_BLOCK_TIMER();
        //        Saiga::PGORec pgo;
        //        Saiga::CeresPGO pgo;

        if (pg.fixScale)
        {
            Saiga::PGORec pgo;
            pgo.optimizationOptions = pgoo;
            pgo.create(pg);
            auto res = pgo.initAndSolve();
            std::cout << "[PGORec] " << res.cost_initial << " -> " << res.cost_final << " in " << res.total_time
                      << " ms." << std::endl;
        }
        else
        {
            Saiga::PGOSim3Rec pgo;
            pgo.optimizationOptions = pgoo;
            pgo.create(pg);
            auto res = pgo.initAndSolve();
            std::cout << "[PGOSim3Rec] " << res.cost_initial << " -> " << res.cost_final << " in " << res.total_time
                      << " ms." << std::endl;
        }
    }

    {
        //        std::unique_lock lock(map.mutexUpdate);
        auto lock = map.LockFull();
        //        std::cout << "rms before pgo " << map.rms() << std::endl;
        std::vector<int> keyframes, points;
        map.GetAllKeyFrames(keyframes);
        map.GetAllMapPoints(points);

        //        SAIGA_ASSERT(per_keyframe_data.size() == map.MaxNumberOfKeyframes());
        per_keyframe_data.resize(map.MaxNumberOfKeyframes());

        // Update old poses (might have been changed during pgo)
        for (auto i : keyframes)
        {
            KeyFrame* kf = &map.getKeyframe(i);
            auto& data   = per_keyframe_data[kf->id()];
            if (kf->isBad())
            {
                data.index_in_pg = -1;
                continue;
            }

            // kf pose might have changed by optimizer
            // data.before_T_w_i = kf->Pose().inverse();

            if (data.index_in_pg != -1)
            {
                data.after_T_w_i = pg.vertices[data.index_in_pg].Sim3Pose();
                kf->SetPose(data.after_T_w_i.se3().inverse());
            }
            else
            {
                SAIGA_EXIT_ERROR("not implemented");
            }
        }

#if 0
        // Compute new poses
        for (auto i : keyframes)
        {
            KeyFrame* kf = &map.getKeyframe(i);
            if (kf->isBad()) continue;

            auto& data = per_keyframe_data[kf->id()];


            if (data.index_in_pg == -1)
            {
                // this kf didn't exist at the beginning of pgo
                // -> it should have atlesat one neighbor with valid pg index
                auto ns = kf->GetConnectedKeyFrames();

                Keyframe* rel_kf = nullptr;

                for (auto other : ns)
                {
                    auto& other_data = per_keyframe_data[other->id()];
                    if (other_data.index_in_pg != -1)
                    {
                        rel_kf = other;
                        break;
                    }
                }
                SAIGA_ASSERT(rel_kf);

                data.index_in_pg      = -2;
                auto& other_data      = per_keyframe_data[rel_kf->id()];
                SE3 before_T_kf_other = data.before_T_w_i.inverse() * other_data.before_T_w_i;
                auto after_w_other    = other_data.after_T_w_i;

                // goal: w_kf
                DSim3 w_kf       = after_w_other * DSim3(before_T_kf_other.inverse(), 1.0);
                data.after_T_w_i = w_kf;
                kf->SetPose(data.after_T_w_i.se3().inverse());
            }
        }
#endif


        for (auto i : points)
        {
            MapPoint* mp = &map.getMapPoint(i);
            if (mp->isBad()) continue;

            KeyFrame* ref_kf = mp->referenceKF;

            if (!ref_kf || ref_kf->isBad())
            {
                mp->SetBadFlag();
                continue;
            }

            auto& data = per_keyframe_data[ref_kf->id()];
            if (data.index_in_pg == -1) continue;
            mp->Transform(data.after_T_w_i * DSim3(data.before_T_w_i.inverse()));
            continue;



            auto old_wp = mp->getPosition();



            // Project to old camera space and then to new world space
            auto new_wp = data.after_T_w_i * (data.before_T_w_i.inverse() * old_wp);
            mp->SetWorldPos(new_wp);
            mp->normal =
                data.after_T_w_i.se3().unit_quaternion() * (data.before_T_w_i.inverse().unit_quaternion() * mp->normal);
            mp->reference_depth *= data.after_T_w_i.scale();
        }
        map.mapState++;
    }
}



}  // namespace Snake
