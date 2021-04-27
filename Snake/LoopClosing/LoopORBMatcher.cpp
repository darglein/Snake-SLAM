/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */
#include "LoopORBMatcher.h"

#include "saiga/vision/icp/RegistrationRANSAC.h"

#include "Map/Map.h"
namespace Snake
{
int LoopORBmatcher::Fuse(KeyFrame* pKF, const std::vector<MapPoint*>& vpPoints, float th)
{
    int nFused = 0;

    auto pose           = pKF->Pose();
    auto cameraposition = pose.inverse().translation();

    // For each candidate MapPoint project and match
    for (auto mp : vpPoints)
    {
        // Discard Bad MapPoints and already found
        if (mp->isBad() || mp->IsInKeyFrame(pKF)) continue;

        Vec3 wp = mp->getPosition();
        Vec3 np = pose * wp;
        if (np.z() <= 0) continue;

        Vec2 ip = K.project(np);
        if (!featureGridBounds.inImage(ip)) continue;

        Vec3 PO     = cameraposition - wp;
        auto dist3D = PO.norm();

        // Viewing angle must be less than 60 deg
        auto Pn = mp->GetNormal();

        //        if (PO.dot(Pn) < 0.5 * dist3D) continue;
        if (PO.transpose() * Pn < 0.5 * dist3D) continue;

        tmp_indices.clear();


        // Search in a radius
        // maybe multiply by scale factor
        auto radius = th * 1;
        pKF->frame->GetFeaturesInArea(tmp_indices, ip, radius);
        if (tmp_indices.empty()) continue;

        // Match to the most similar keypoint in the radius

        auto dMP = mp->GetDescriptor();

        int bestDist = 256;
        int bestIdx  = -1;

        for (auto idx : tmp_indices)
        {
#ifdef MATCHING_CHECK_SCALE_CONSISTENCY
            auto& kp           = pKF->frame->undistorted_keypoints[idx];
            const int& kpLevel = kp.octave;
            if (kpLevel < nPredictedLevel - 1 || kpLevel > nPredictedLevel) continue;
#endif
            // todo: check depth error
            auto& dKF = pKF->frame->descriptors[idx];
            auto dist = distance(dMP, dKF);
            if (dist < bestDist)
            {
                bestDist = dist;
                bestIdx  = idx;
            }
        }

        // If there is already a MapPoint replace otherwise add new measurement
        if (bestDist <= TH_LOW)
        {
            MapPoint* pMPinKF = pKF->GetMapPoint(bestIdx);
            if (pMPinKF)
            {
                if (!pMPinKF->isBad())
                {
                    pMPinKF->Replace(mp);
                }
            }
            else
            {
                pKF->addMappoint(mp, bestIdx);
                mp->AddObservation(pKF, bestIdx);
                mp->ComputeDistinctiveDescriptors();
            }
            nFused++;
        }
    }
    return nFused;
}
int LoopORBmatcher::MatchBruteforce(KeyFrame* pKF1, KeyFrame* pKF2, vector<MapPoint*>& vpMatches12, int threshold,
                                    float ratio)
{
    auto& vpMapPoints1 = pKF1->GetMapPointMatches();
    auto& vpMapPoints2 = pKF2->GetMapPointMatches();
    vpMatches12        = vector<MapPoint*>(vpMapPoints1.size(), static_cast<MapPoint*>(NULL));
    BruteForceMatcher<FeatureDescriptor> matcher;
    matcher.matchKnn2(pKF1->frame->descriptors, pKF2->frame->descriptors);
    matcher.filterMatches(threshold, ratio);

    int c = 0;
    for (auto m : matcher.matches)
    {
        MapPoint* pMP1 = vpMapPoints1[m.first];
        MapPoint* pMP2 = vpMapPoints2[m.second];
        if (pMP1 && pMP2)
        {
            vpMatches12[m.first] = pMP2;
            c++;
        }
    }
    return c;
}

int LoopORBmatcher::MatchBoW(KeyFrame* pKF1, KeyFrame* pKF2, vector<MapPoint*>& vpMatches12, int threshold, float ratio)
{
    SAIGA_ASSERT(pKF1 && pKF2);
    auto& vFeatVec1    = pKF1->frame->bow_feature_vec;
    auto& vpMapPoints1 = pKF1->GetMapPointMatches();
    auto& Descriptors1 = pKF1->frame->descriptors;

    auto& vFeatVec2    = pKF2->frame->bow_feature_vec;
    auto& vpMapPoints2 = pKF2->GetMapPointMatches();
    auto& Descriptors2 = pKF2->frame->descriptors;
    vpMatches12        = vector<MapPoint*>(vpMapPoints1.size(), static_cast<MapPoint*>(NULL));
    vector<bool> vbMatched2(vpMapPoints2.size(), false);


    int nmatches = 0;

    auto f1it  = vFeatVec1.begin();
    auto f2it  = vFeatVec2.begin();
    auto f1end = vFeatVec1.end();
    auto f2end = vFeatVec2.end();

    while (f1it != f1end && f2it != f2end)
    {
        if (f1it->first == f2it->first)
        {
            for (size_t i1 = 0, iend1 = f1it->second.size(); i1 < iend1; i1++)
            {
                const size_t idx1 = f1it->second[i1];

                MapPoint* pMP1 = vpMapPoints1[idx1];
                if (!pMP1) continue;
                if (pMP1->isBad()) continue;

                auto& d1 = Descriptors1[idx1];

                int bestDist1 = 256;
                int bestIdx2  = -1;
                int bestDist2 = 256;

                for (size_t i2 = 0, iend2 = f2it->second.size(); i2 < iend2; i2++)
                {
                    const size_t idx2 = f2it->second[i2];

                    MapPoint* pMP2 = vpMapPoints2[idx2];

                    if (vbMatched2[idx2] || !pMP2) continue;

                    if (pMP2->isBad()) continue;

                    auto& d2 = Descriptors2[idx2];

                    auto dist = distance(d1, d2);

                    if (dist < bestDist1)
                    {
                        bestDist2 = bestDist1;
                        bestDist1 = dist;
                        bestIdx2  = idx2;
                    }
                    else if (dist < bestDist2)
                    {
                        bestDist2 = dist;
                    }
                }

                if (bestDist1 < threshold)
                {
                    if (static_cast<float>(bestDist1) < ratio * static_cast<float>(bestDist2))
                    {
                        vbMatched2[bestIdx2] = true;

                        vpMatches12[idx1] = vpMapPoints2[bestIdx2];

                        nmatches++;
                    }
                }
            }

            f1it++;
            f2it++;
        }
        else if (f1it->first < f2it->first)
        {
            f1it = std::lower_bound(f1it, vFeatVec1.end(), *f2it,
                                    [](const auto& a, const auto& b) { return a.first < b.first; });
        }
        else
        {
            f2it = std::lower_bound(f2it, vFeatVec2.end(), *f1it,
                                    [](const auto& a, const auto& b) { return a.first < b.first; });
        }
    }

    return nmatches;
}

}  // namespace Snake
