/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once

#include "Map/Map.h"


namespace Snake
{
class KeyframeDatabase
{
   public:
    KeyframeDatabase();

    void Add(KeyFrame* kf);
    void Remove(KeyFrame* kf);

    //  Given an BowVector, this function finds similar keyframes. These are returned together with the bow-score in a
    //  pair. All keyframes in connected_keyframes are removed from the output before computing the score.
    std::vector<std::pair<Keyframe*, float>> DetectLoopCandidates(const BowVector* bv,
                                                                  Saiga::ArrayView<Keyframe*> connected_keyframes,
                                                                  float minScore, int max_candidates);

    // Find matching similar keyframes.
    std::vector<std::pair<Keyframe*, float>> DetectRelocalizationCandidates(const BowVector* bv, float minScore,
                                                                            int max_candidates);

   protected:
    // Inverted file
    std::vector<std::vector<KeyFrame*>> inverse_list;

    std::vector<Keyframe*> tmp_keyframes;

    mutable std::mutex mtx;

    // For each keyframe one of these objects is allocated.
    // The keyframe id references directly into the array.
    struct PerKeyframeDatabaseData
    {
        // Variables used by the keyframe database
        int mnLoopQuery  = 0;
        int mnLoopWords  = 0;
        float mLoopScore = 0;

        const BowVector* bv = nullptr;
    };

    int query_counter = 0;

    std::vector<PerKeyframeDatabaseData> perKeyframeData;

    // Returns all keyframes that have at least one sharing word.
    void GetKeyframesWithSharingWords(const BowVector* bv, std::vector<KeyFrame*>& kfs_with_sharing_words);


    void RemoveWeakMatches(const BowVector* bv, std::vector<KeyFrame*>& kfs, float sharing_word_ratio,
                           float score_ratio, float min_score, int max_keyframes);
};

inline std::unique_ptr<KeyframeDatabase> keyFrameDB;


}  // namespace Snake
