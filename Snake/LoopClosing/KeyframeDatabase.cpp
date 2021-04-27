/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#include "KeyframeDatabase.h"

#include "LocalMapping/Bow.h"

namespace Snake
{
KeyframeDatabase::KeyframeDatabase()
{
    SAIGA_ASSERT(vocabulary.size() > 0);
    inverse_list.resize(vocabulary.size());
    perKeyframeData.resize(maxKeyframes);
}

void KeyframeDatabase::Add(KeyFrame* kf)
{
    std::unique_lock lock(mtx);
    SAIGA_ASSERT(kf);
    for (auto vit : kf->frame->bow_vec) inverse_list[vit.first].push_back(kf);
    perKeyframeData[kf->id()].bv = &kf->frame->bow_vec;
}

void KeyframeDatabase::Remove(KeyFrame* kf)
{
    std::unique_lock lock(mtx);

    SAIGA_ASSERT(kf->id() < (int)perKeyframeData.size());
    auto& bv = perKeyframeData[kf->id()].bv;
    if (bv)
    {
        // Erase elements in the Inverse File for the entry
        for (auto vit : *bv)
        {
            // List of keyframes that share the word
            auto& kfs          = inverse_list[vit.first];
            size_t idx_in_list = std::distance(kfs.begin(), std::find(kfs.begin(), kfs.end(), kf));
            SAIGA_ASSERT(idx_in_list < kfs.size());


            kfs[idx_in_list] = kfs.back();
            kfs.pop_back();

            //            lKFs.remove(kf);
            //            lKFs.erase(kf);


            //                kfs.erase(std::find(kfs.begin(), kfs.end(), kf));
        }
        bv = nullptr;
    }
}

std::vector<std::pair<Keyframe*, float>> KeyframeDatabase::DetectLoopCandidates(
    const BowVector* bv, Saiga::ArrayView<Keyframe*> connected_keyframes, float minScore, int max_candidates)
{
    std::unique_lock lock(mtx);

    GetKeyframesWithSharingWords(bv, tmp_keyframes);
    tmp_keyframes.erase(std::remove_if(tmp_keyframes.begin(), tmp_keyframes.end(),
                                       [&](auto kf) {
                                           return std::find(connected_keyframes.begin(), connected_keyframes.end(),
                                                            kf) != connected_keyframes.end();
                                       }),
                        tmp_keyframes.end());

    RemoveWeakMatches(bv, tmp_keyframes, 0.8, 0.75, minScore, max_candidates);

    std::vector<std::pair<Keyframe*, float>> result;
    result.reserve(tmp_keyframes.size());
    for (auto kf : tmp_keyframes)
    {
        result.emplace_back(kf, perKeyframeData[kf->id()].mLoopScore);
    }
    return result;
}


std::vector<std::pair<Keyframe*, float>> KeyframeDatabase::DetectRelocalizationCandidates(const BowVector* bv,
                                                                                          float minScore,
                                                                                          int max_candidates)
{
    std::unique_lock lock(mtx);
    GetKeyframesWithSharingWords(bv, tmp_keyframes);
    RemoveWeakMatches(bv, tmp_keyframes, 0.8, 0.75, 0, max_candidates);

    std::vector<std::pair<Keyframe*, float>> result;
    result.reserve(tmp_keyframes.size());
    for (auto kf : tmp_keyframes)
    {
        result.emplace_back(kf, perKeyframeData[kf->id()].mLoopScore);
    }
    return result;
}

void KeyframeDatabase::GetKeyframesWithSharingWords(const BowVector* bv, std::vector<KeyFrame*>& kfs_with_sharing_words)
{
    // Search all keyframes that share a word with current keyframes
    // Discard keyframes connected to the query keyframe
    query_counter++;
    kfs_with_sharing_words.clear();
    for (auto vit : *bv)
    {
        auto& lKFs = inverse_list[vit.first];
        for (auto other_kf : lKFs)
        {
            auto& other_data = perKeyframeData[other_kf->id()];
            if (other_data.mnLoopQuery != query_counter)
            {
                other_data.mnLoopWords = 0;
                other_data.mnLoopQuery = query_counter;
                kfs_with_sharing_words.push_back(other_kf);
            }
            other_data.mnLoopWords++;
        }
    }
}

void KeyframeDatabase::RemoveWeakMatches(const BowVector* bv, std::vector<KeyFrame*>& kfs, float sharing_word_ratio,
                                         float score_ratio, float min_score, int max_keyframes)
{
    // Only compare against those keyframes that share enough words
    int maxCommonWords = 0;
    for (auto other_kf : kfs)
    {
        auto& other_data = perKeyframeData[other_kf->id()];
        if (other_data.mnLoopWords > maxCommonWords) maxCommonWords = other_data.mnLoopWords;
    }

    kfs.erase(std::remove_if(kfs.begin(), kfs.end(),
                             [&](auto kf) {
                                 auto& other_data = perKeyframeData[kf->id()];
                                 return other_data.mnLoopWords < sharing_word_ratio * maxCommonWords;
                             }),
              kfs.end());

    // Compute similarity score.
    float best_score = 0;
    for (auto other_kf : kfs)
    {
        auto& other_data      = perKeyframeData[other_kf->id()];
        float si              = vocabulary.score(*bv, other_kf->frame->bow_vec);
        other_data.mLoopScore = si;

        if (si > best_score)
        {
            best_score = si;
        }
    }

    kfs.erase(std::remove_if(kfs.begin(), kfs.end(),
                             [&](auto kf) {
                                 auto& other_data = perKeyframeData[kf->id()];
                                 return other_data.mLoopScore < score_ratio * best_score ||
                                        other_data.mLoopScore < min_score;
                             }),
              kfs.end());

    std::sort(kfs.begin(), kfs.end(), [this](Keyframe* kf1, Keyframe* kf2) {
        return perKeyframeData[kf1->id()].mLoopScore > perKeyframeData[kf2->id()].mLoopScore;
    });

    kfs.resize(std::min<int>(max_keyframes, kfs.size()));
}

}  // namespace Snake
