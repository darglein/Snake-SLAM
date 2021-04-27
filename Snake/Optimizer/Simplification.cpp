/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */
//#define ENABLE_MAP_SYNC_TEST

#include "Simplification.h"

#include "saiga/core/imgui/imgui.h"
#include "saiga/core/util/tostring.h"


namespace Snake
{
Simplification::Simplification()
    : DelayedParallelMapOptimization("Simplification", simpl_enable, simpl_delay, settings.async),
      Module(ModuleType::SIMPLIFICATION)
{
    CreateTable({10, 10, 10, 10}, {"Keyframe", "Type", "Value", "Time (ms)"});
}

Simplification::~Simplification() {}



void Simplification::Process(Keyframe* kf)
{
    if (!kf || kf->isBad()) return;



    TEST_MAP_SYNC;


    bool res = false;
    const char* type;
    double value;


    {
        auto timer                 = ModuleTimer();
        std::tie(res, type, value) = KeyFrameCullingGraph(kf);

        if (res)
        {
            CHECK_VALID_MAP;


            tmp_keyframes.clear();
            {
                auto lock = map.LockReadOnly();
                kf->GetBestCovisibilityKeyFrames(tmp_keyframes, 3);
            }
            EraseKeyframe(kf);
            TEST_MAP_SYNC;
            for (auto c : tmp_keyframes)
            {
                if (c && !c->isBad())
                {
                    Add(c, 5);
                }
            }
            std::this_thread::sleep_for(std::chrono::microseconds(2000));
        }
    }

    if (res)
    {
        (*output_table) << kf->id() << type << value << LastTime();
    }
}


float Simplification::Redundancy(KeyFrame* kf)
{
    int nObs                   = 3;
    const int thObs            = nObs;
    int nRedundantObservations = 0;
    int nMPs                   = 0;

    TEST_MAP_SYNC;
    {
        // Check redundant keyframes (only local keyframes)
        // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
        // in at least other 3 keyframes (in the same or finer scale)
        // We only consider close stereo points
        //        std::shared_lock l(map.mutexUpdate);
        auto lock = map.LockReadOnly();


        // This kf is already removed.
        if (kf->isBad())
        {
            return false;
        }

        auto& vpMapPoints = kf->GetMapPointMatches();

        for (size_t i = 0, iend = vpMapPoints.size(); i < iend; i++)
        {
            MapPoint* pMP = vpMapPoints[i];
            if (pMP)
            {
                if (!pMP->isBad())
                {
                    if (settings.inputType != InputType::Mono)
                    {
                        if (kf->frame->depth[i] > th_depth)
                        {
                            continue;
                        }
                    }


                    nMPs++;
                    if (pMP->GetNumObservations() > thObs)
                    {
                        const int& scaleLevel = kf->frame->undistorted_keypoints[i].octave;

                        int nObs = 0;
                        for (auto obs : pMP->GetObservationList())
                        {
                            KeyFrame* pKFi = obs.first;
                            if (pKFi == kf) continue;
                            const int& scaleLeveli = pKFi->frame->undistorted_keypoints[obs.second].octave;

                            if (scaleLeveli <= scaleLevel + 1)
                            {
                                nObs++;
                                if (nObs >= thObs) break;
                            }
                        }
                        if (nObs >= thObs)
                        {
                            nRedundantObservations++;
                        }
                    }
                }
            }
        }
    }
    TEST_MAP_SYNC;

    return float(nRedundantObservations) / nMPs;
}

std::tuple<bool, const char*, double> Simplification::KeyFrameCullingGraph(KeyFrame* kf)
{
    if (kf->cull_factor > 3)
    {
        return {true, "Factor", 0};
    }

    TEST_MAP_SYNC;


#if WITH_IMU
    if (current_gyro_weight == 0 || current_acc_weight == 0)
    {
        return {false, "", 0};
    }

    if (kf->previousKF && kf->nextKF)
    {
        double time_delta_without = kf->nextKF->frame->timeStamp - kf->previousKF->frame->timeStamp;
        //        std::cout << "cull " << time_delta_without << " - " << max_time_between_kf_map << std::endl;
        if (time_delta_without > max_time_between_kf_map)
        {
            return {false, "", 0};
        }
    }
    else
    {
        // std::cout << "wtf" << std::endl;
    }


#endif



    std::vector<Keyframe*> id_to_keyframe;
    std::unordered_map<Keyframe*, int> id_map;

    auto addConnections = [](GraphType& graph, std::unordered_map<Keyframe*, int>& id_map, KeyFrame* from,
                             const std::vector<std::pair<int, Keyframe*>>& v) {
        auto it = id_map.find(from);
        if (it == id_map.end()) return;
        auto id_from = (*it).second;

        for (auto c : v)
        {
            auto it = id_map.find(c.second);
            if (it == id_map.end()) continue;
            auto id_to = (*it).second;

            graph.AddEdge(id_from, id_to, c.first);
        }
    };


    {
        // full lock during update
        // the rest is read only
        auto lock = map.LockFull();
        kf->UpdateConnections();
    }


    TEST_MAP_SYNC;
    {
        auto lock = map.LockReadOnly();


        // This kf is already removed.
        if (kf->isBad())
        {
            return {false, "", 0};
        }

        auto connections = kf->GetConnections();
        connections.erase(std::remove_if(connections.begin(), connections.end(),
                                         [=](auto pair) { return pair.first < simpl_min_matches_in_graph; }),
                          connections.end());

        // Maps keyframes to local ids
        int n = connections.size() + 1;
        id_to_keyframe.resize(n);

        id_map[kf]        = 0;
        id_to_keyframe[0] = kf;

        int current_id = 1;
        for (auto c : connections)
        {
            id_to_keyframe[current_id] = c.second;
            id_map[c.second]           = current_id++;
        }

        graph.Resize(n);
        addConnections(graph, id_map, kf, connections);
        for (auto c : connections)
        {
            addConnections(graph, id_map, c.second, c.second->GetConnections());
        }
    }

    TEST_MAP_SYNC;
    graph.BuildMST();
    TEST_MAP_SYNC;


    // extract all mst edges of the root node
    auto mst_edges = graph.GetMSTEdgesForNode(0);
    if (mst_edges.size() == 0)
    {
        // No connections at all
        // (We added only connections with a minimum number of matches)
        return {true, "No Conn.", 0};
    }


    // compute angle to all mst edges
    std::vector<double> mst_angles;
    int kf_matches;
    {
        auto lock = map.LockReadOnly();
        kf->ComputeDepthRange();
        auto depth1 = kf->MedianDepth();
        for (auto e : mst_edges)
        {
            auto other_kf = id_to_keyframe[e.to];
            other_kf->ComputeDepthRange();
            auto depth2   = other_kf->MedianDepth();
            auto depth    = (depth1 + depth2) * 0.5;
            auto baseline = (kf->CameraPosition() - other_kf->CameraPosition()).norm();
            auto alpha    = atan2(0.5 * baseline, depth);
            double angle  = degrees(alpha * 2.0);
            mst_angles.push_back(angle);
        }
        kf_matches = kf->MatchCount(3);
    }
    TEST_MAP_SYNC;


    if (mst_edges.size() == 1)
    {
        // Mst == 1 means that this kf is a side branch of the tree.
        // Such keyframes aren't very important for overall completeness so we can cull them without much fear.
        auto angle = mst_angles[0];
        if (angle < simpl_border_angle * kf->cull_factor)
        {
            return {true, "Angle", angle};
        }

        if (kf_matches < simpl_border_matches * kf->cull_factor)
        {
            //            return true;
            return {true, "Matches", kf_matches};
        }

        auto redu = Redundancy(kf);
        if (redu > simpl_border_redundancy)
        {
            return {true, "Redu.", redu};
        }

        return {false, "", 0};
    }


    // Build a graph only containing the nodes from the neighbours
    // Only the super-local kfs
    // Id 0 = the root node
    std::vector<KeyFrame*> kfs;
    std::unordered_map<Keyframe*, int> local_id_map;

    for (auto n : mst_edges)
    {
        SAIGA_ASSERT(n.from == 0);
        SAIGA_ASSERT(n.to != 0);
        SAIGA_ASSERT(n.weight != WeightedUndirectedGraph<int>::invalid_weight);
        SAIGA_ASSERT(n.weight > 0);
        auto kf          = id_to_keyframe[n.to];
        local_id_map[kf] = kfs.size();
        kfs.push_back(kf);
    }

    graph.Resize(kfs.size());

    {
        auto lock = map.LockReadOnly();
        for (auto kf : kfs)
        {
            addConnections(graph, local_id_map, kf, kf->GetConnections());
        }
    }

    graph.BuildMST();
    auto weakest_link = graph.WeightOfWeakestMSTEdge();
    if (weakest_link > 100000)
    {
        return {false, "", 0};
    }

    // Add edges between these nodes from visibility
    // Remove root node and compute spanning tree
    // Accept culling if edges not significantly worse than edges including the current node (enforce minimum
    // weight) and current node is not so good (see condition above)
    if (weakest_link * kf->cull_factor > settings.th_map)
    {
        return {true, "Link", weakest_link};
    }


    return {false, "", 0};
}

void Simplification::EraseKeyframe(Keyframe* kf)
{
    CHECK_VALID_MAP;
    {
        //        std::unique_lock l(map.mutexUpdate);
        auto lock = map.LockFull();
        kf->SetBadFlag();
    }

    CHECK_VALID_MAP;
}



}  // namespace Snake
