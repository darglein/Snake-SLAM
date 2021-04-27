/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once
#include "saiga/vision/icp/RegistrationRANSAC.h"

#include "KeyframeDatabase.h"
#include "Map/Map.h"

#include <optional>

namespace Snake
{
struct Loop
{
    bool found_loop = false;
    int candidates  = 0;
    int matches     = 0;
    int inliers     = 0;
    double scale    = 1;

    // === Variables Set if loop was found ===
    KeyFrame* source_keyframe;
    KeyFrame* target_keyframe;
    std::vector<MapPoint*> mvpCurrentMatchedPoints;
    std::vector<MapPoint*> target_map_points;
    std::vector<MapPoint*> source_map_points;
    DSim3 T_w_correctSource;
    DSim3 T_target_source;
};


class LoopDetector
{
   public:
    LoopDetector();

    Loop Detect(Keyframe* kf);


   private:
    bool ComputeSim3(KeyFrame* source_kf, Keyframe* target_kf);

    bool ComputeSim3(KeyFrame* kf);

    bool DetectLoop(KeyFrame* kf);
    bool CheckConsistency(KeyFrame* new_kf);

    std::tuple<SE3, double, int> solve(KeyFrame* pKF1, KeyFrame* pKF2, const std::vector<MapPoint*>& vpMatched12,
                                       vector<bool>& vbInliers, bool compute_scale);

    Loop loop;

    struct LoopCandidate
    {
        KeyFrame* kf;
        float score;
        int consistency_count = 1;
        std::vector<Keyframe*> connections_with_kf;

        LoopCandidate() {}
        LoopCandidate(Keyframe* kf, float score)
            : kf(kf), score(score), connections_with_kf(kf->GetConnectedKeyFrames())
        {
            connections_with_kf.push_back(kf);
        }
        bool InGroup(Keyframe* kf)
        {
            return std::find(connections_with_kf.begin(), connections_with_kf.end(), kf) != connections_with_kf.end();
        }
        bool operator>(const LoopCandidate& other) const { return score > other.score; }
    };

    // similar to orb-slam covisibilty groups, but just stored as keyframes here.
    std::vector<LoopCandidate> previous_candidates;

    // initial candidates computed from the bow-database matcher
    std::vector<LoopCandidate> initial_candidates;

    // actual candidates for feature matching
    std::vector<LoopCandidate> candidateKFs;

    std::vector<int> mnLoopPointForKF;


    size_t max_match_keyframes = 3;
    double min_score_alpha     = 0.5;
    double average_min_score   = -1;
    RegistrationProjectRANSAC solver;
};



}  // namespace Snake
