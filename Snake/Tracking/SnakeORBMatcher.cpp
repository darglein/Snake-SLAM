/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#include "SnakeORBMatcher.h"

#include "saiga/vision/reconstruction/Epipolar.h"

#include "Map/Map.h"
namespace Snake
{
SnakeORBMatcher::SnakeORBMatcher()
{
    tmp_indices.reserve(10000);
    tmp_matches_world_image.reserve(50000);


    for (auto& hist : rotHist)
    {
        hist.reserve(500);
    }
}


inline void ComputeThreeMaxima(vector<int>* histo, const int L, int& ind1, int& ind2, int& ind3)
{
    int max1 = 0;
    int max2 = 0;
    int max3 = 0;

    for (int i = 0; i < L; i++)
    {
        const int s = histo[i].size();
        if (s > max1)
        {
            max3 = max2;
            max2 = max1;
            max1 = s;
            ind3 = ind2;
            ind2 = ind1;
            ind1 = i;
        }
        else if (s > max2)
        {
            max3 = max2;
            max2 = s;
            ind3 = ind2;
            ind2 = i;
        }
        else if (s > max3)
        {
            max3 = s;
            ind3 = i;
        }
    }

    if (max2 < 0.1f * (float)max1)
    {
        ind2 = -1;
        ind3 = -1;
    }
    else if (max3 < 0.1f * (float)max1)
    {
        ind3 = -1;
    }
}


int SnakeORBMatcher::SearchByProjectionFrameToKeyframe(Frame& CurrentFrame, const Keyframe& kf, float th,
                                                       FeatureDistance featureError)
{
    std::set<MapPoint*> sAlreadyFound;

    for (int i : CurrentFrame.featureRange())
    {
        auto mp = CurrentFrame.mvpMapPoints[i];
        if (mp)
        {
            sAlreadyFound.insert(mp);
        }
    }


    // Rotation Histogram (to check rotation consistency)
    bool mbCheckOrientation = false;
    for (auto& hist : rotHist)
    {
        hist.clear();
    }
    const float factor = 1.0f / HISTO_LENGTH;

    int matches      = 0;
    auto currentPose = CurrentFrame.Pose();

    auto& points = kf.GetMapPointMatches();

    for (int i = 0; i < (int)points.size(); ++i)
    {
        auto mp = points[i];
        if (!mp) continue;
        if (sAlreadyFound.count(mp)) continue;

        // project to currentFrame
        auto wp  = mp->getPosition();
        auto ip3 = K.project3(currentPose * wp);
        auto ip  = ip3.segment<2>(0);
        auto z   = ip3(2);

        // this wp is behind the camera
        if (z <= 0) continue;
        // this wp is not in the image
        if (!featureGridBounds.inImage(ip)) continue;

        tmp_indices.clear();
        auto r = th;
        CurrentFrame.GetFeaturesInArea(tmp_indices, ip, th);

        auto descriptor1 = mp->GetDescriptor();
        int bestDist     = 256;
        int bestIdx2     = -1;
        for (auto i2 : tmp_indices)
        {
            if (CurrentFrame.mvpMapPoints[i2]) continue;
            if (CurrentFrame.right_points[i2] > 0)
            {
                SAIGA_ASSERT(stereo_cam.fx != 1);
                auto disp      = stereo_cam.LeftPointToRight(ip.x(), z);
                const float er = std::abs(disp - CurrentFrame.right_points[i2]);
                if (er > r * 0.5)
                {
                    continue;
                }
            }

            auto descriptor2 = CurrentFrame.descriptors[i2];
            auto dist        = distance(descriptor1, descriptor2);


            if (dist < bestDist)
            {
                bestDist = dist;
                bestIdx2 = i2;
            }
        }

        if (bestDist <= featureError)
        {
            CurrentFrame.mvpMapPoints[bestIdx2] = mp;
            matches++;

            if (mbCheckOrientation)
            {
                float rot =
                    kf.frame->undistorted_keypoints[i].angle - CurrentFrame.undistorted_keypoints[bestIdx2].angle;
                if (rot < 0.0) rot += 360.0f;
                int bin = round(rot * factor);
                if (bin == HISTO_LENGTH) bin = 0;
                assert(bin >= 0 && bin < HISTO_LENGTH);
                rotHist[bin].push_back(bestIdx2);
            }
        }
    }

    // Apply rotation consistency
    if (mbCheckOrientation)
    {
        int ind1 = -1;
        int ind2 = -1;
        int ind3 = -1;

        ComputeThreeMaxima(rotHist.data(), HISTO_LENGTH, ind1, ind2, ind3);

        for (int i = 0; i < HISTO_LENGTH; i++)
        {
            if (i != ind1 && i != ind2 && i != ind3)
            {
                for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++)
                {
                    CurrentFrame.mvpMapPoints[rotHist[i][j]] = static_cast<MapPoint*>(NULL);
                    matches--;
                }
            }
        }
    }
    return matches;
}


int SnakeORBMatcher::SearchByProjectionFrameFrame2(Frame& CurrentFrame, const LocalMap<CoarseTrackingPoint>& lm,
                                                   const float th, FeatureDistance featureError, int num_threads)
{
    for (auto& hist : rotHist)
    {
        hist.clear();
    }

    SE3 currentPose     = CurrentFrame.Pose();
    Vec3 cameraPosition = currentPose.inverse().translation();
    // Rotation Histogram (to check rotation consistency)

    const float factor = 1.0f / HISTO_LENGTH;

    int matches = 0;
    int test    = 0;


    // negative because we use a motion model in world->cameraspace
    auto z_diff          = -CurrentFrame.local_velocity.translation().z();
    const bool bForward  = settings.inputType != InputType::Mono && z_diff > stereo_cam.baseLine();
    const bool bBackward = settings.inputType != InputType::Mono && z_diff < stereo_cam.baseLine();


    int N = lm.points.size();

    tmp_indices_omp.resize(num_threads);
    tmp_matches_world_image.resize(N);

#pragma omp parallel for num_threads(num_threads)
    for (int i = 0; i < N; ++i)
    {
        tmp_matches_world_image[i].first = -1;
        auto&& lmp                       = lm.points[i];


        // project to currentFrame
        auto wp  = lmp.position;
        auto ip3 = K.project3(currentPose * wp);
        auto ip  = ip3.segment<2>(0);
        auto z   = ip3(2);

        // this wp is behind the camera
        if (z <= 0) continue;
        // this wp is not in the image
        if (!featureGridBounds.inImage(ip)) continue;


        Vec3 PO     = cameraPosition - wp;
        double dist = PO.norm();

        // Check viewing angle
        auto Pn              = lmp.normal;
        double viewCos       = PO.dot(Pn) / dist;
        auto viewingCosLimit = 0.5;
        if (viewCos < viewingCosLimit)
        {
            continue;
        }


        auto& tmp_indices = tmp_indices_omp[OMP::getThreadNum()];
        tmp_indices.clear();


        int last_scale_level = lmp.octave;


        auto r = th;
        r *= scalePyramid.Scale(last_scale_level);

        if (bForward)
        {
            CurrentFrame.GetFeaturesInAreaAndCheckScale(tmp_indices, ip, r, last_scale_level - 1, 100);
        }
        else if (bBackward)
        {
            CurrentFrame.GetFeaturesInAreaAndCheckScale(tmp_indices, ip, r, 0, last_scale_level);
        }
        else
        {
            CurrentFrame.GetFeaturesInAreaAndCheckScale(tmp_indices, ip, r, last_scale_level - 1, last_scale_level + 1);
        }


        if (tmp_indices.empty()) continue;
        test += tmp_indices.size();

        auto& descriptor1 = lmp.descriptor;
        int bestDist      = 256;
        int bestIdx       = -1;
        for (auto i2 : tmp_indices)
        {
            if (CurrentFrame.mvpMapPoints[i2]) continue;

            if (CurrentFrame.right_points[i2] > 0)
            {
                SAIGA_ASSERT(stereo_cam.fx != 1);
                auto disp = stereo_cam.LeftPointToRight(ip.x(), z);
                auto er   = std::abs(disp - CurrentFrame.right_points[i2]);
                if (er > r * 0.5)
                {
                    continue;
                }
            }

            auto descriptor2 = CurrentFrame.descriptors[i2];
            auto dist        = distance(descriptor1, descriptor2);
            if (dist < bestDist)
            {
                bestDist = dist;
                bestIdx  = i2;
            }
        }


        if (bestDist <= featureError)
        {
            SAIGA_ASSERT(bestIdx != -1);

            float rot = lmp.angle - CurrentFrame.undistorted_keypoints[bestIdx].angle;
            if (rot < 0.0) rot += 360.0f;
            int bin = round(rot * factor);
            if (bin == HISTO_LENGTH) bin = 0;
            SAIGA_ASSERT(bin >= 0 && bin < HISTO_LENGTH);

            tmp_matches_world_image[i] = {bestIdx, bin};
        }
    }

    for (int i = 0; i < N; ++i)
    {
        auto bestIdx = tmp_matches_world_image[i].first;
        if (bestIdx == -1) continue;
        if (CurrentFrame.mvpMapPoints[bestIdx]) continue;
        auto&& lmp                         = lm.points[i];
        CurrentFrame.mvpMapPoints[bestIdx] = lmp.mp;

        int bin = tmp_matches_world_image[i].second;
        rotHist[bin].push_back(bestIdx);
        matches++;
    }

    // Apply rotation consistency
    int ind1 = -1;
    int ind2 = -1;
    int ind3 = -1;

    ComputeThreeMaxima(rotHist.data(), HISTO_LENGTH, ind1, ind2, ind3);

    for (int i = 0; i < HISTO_LENGTH; i++)
    {
        if (i != ind1 && i != ind2 && i != ind3)
        {
            for (size_t j = 0, jend = rotHist[i].size(); j < jend; j++)
            {
                CurrentFrame.mvpMapPoints[rotHist[i][j]] = nullptr;
                matches--;
            }
        }
    }

    return matches;
}  // namespace Snake


inline float RadiusByViewingCos(float viewCos)
{
    if (viewCos > 0.998)
        return 2.5;
    else
        return 4.0;
}

int SnakeORBMatcher::SearchByProjection2(Frame& CurrentFrame, LocalMap<FineTrackingPoint>& lm, const float th,
                                         float ratio, int num_threads)
{
    //    SAIGA_BLOCK_TIMER();
    int matches         = 0;
    SE3 currentPose     = CurrentFrame.Pose();
    Vec3 cameraPosition = currentPose.inverse().translation();
    int N               = lm.points.size();

    const bool bFactor = th != 1.0;

    tmp_matches_world_image.resize(N);
    tmp_indices_omp.resize(num_threads);


#pragma omp parallel for num_threads(num_threads)
    for (int i = 0; i < N; ++i)
    {
        tmp_matches_world_image[i].first = -1;
        auto&& lmp                       = lm.points[i];
        if (!lmp.valid)
        {
            continue;
        }

        auto& P = lmp.position;

        // project to camera an check if the point is in the frustum
        const auto Pc = currentPose * P;
        auto z        = Pc(2);
        if (z < 0)
        {
            lmp.valid = false;
            continue;
        }
        auto ip2 = K.project(Pc);
        if (!featureGridBounds.inImage(ip2))
        {
            lmp.valid = false;
            continue;
        }

        Vec3 PO     = cameraPosition - P;
        double dist = PO.norm();
#ifdef MATCHING_MIN_MAX_DISTANCE2
        // Check distance is in the scale invariance region of the MapPoint
        auto [min_distance, max_distance] =
            scalePyramid.EstimateMinMaxDistance(lmp.reference_depth, lmp.reference_scale_level);
        if (dist < min_distance || dist > max_distance)
        {
            lmp.valid = false;
            continue;
        }
#endif


        // Check viewing angle
        auto Pn      = lmp.normal;
        auto viewCos = PO.dot(Pn) / dist;

        auto viewingCosLimit = 0.5;
        if (viewCos < viewingCosLimit)
        {
            lmp.valid = false;
            continue;
        }
        lmp.mp->IncreaseVisible();


        auto& tmp_indices = tmp_indices_omp[OMP::getThreadNum()];
        tmp_indices.clear();

        // The size of the window will depend on the viewing direction
        //        float r = th;



#ifdef SNAKE_OS_MODE
        float r = RadiusByViewingCos(viewCos);
        if (bFactor) r *= th;
        auto prediction = scalePyramid.PredictScaleLevel(lmp.reference_depth, lmp.reference_scale_level, dist);
        r *= scalePyramid.ScaleForContiniousLevel(prediction);
        CurrentFrame.GetFeaturesInAreaAndCheckScale(tmp_indices, ip2, r, prediction);
#else
        float r = RadiusByViewingCos(viewCos);
        if (bFactor) r *= th;
        auto prediction = scalePyramid.PredictScaleLevel(lmp.reference_depth, lmp.reference_scale_level, dist);
        r *= scalePyramid.ScaleForContiniousLevel(prediction);
        CurrentFrame.GetFeaturesInAreaAndCheckScale(tmp_indices, ip2, r, prediction);
#endif



        if (tmp_indices.empty()) continue;

        auto MPdescriptor = lmp.descriptor;

        int bestDist   = 256;
        int bestLevel  = -1;
        int bestDist2  = 256;
        int bestLevel2 = -1;
        int bestIdx    = -1;

        // Get best and second matches with near keypoints
        for (auto& idx : tmp_indices)
        {
            //            const size_t idx = *vit;

            if (CurrentFrame.mvpMapPoints[idx]) continue;
            //                if (CurrentFrame.mvpMapPoints[idx]->Observations() > 0) continue;

            //            SAIGA_ASSERT(pMP->mTrackProjX != -1);
            if (CurrentFrame.right_points[idx] > 0)
            {
                SAIGA_ASSERT(stereo_cam.fx != 1);
                auto er = std::abs(stereo_cam.LeftPointToRight(ip2.x(), z) - CurrentFrame.right_points[idx]);
                if (er > r * 0.5)
                {
                    continue;
                }
            }


            auto d = CurrentFrame.descriptors[idx];

            const int dist = distance(MPdescriptor, d);

            if (dist < bestDist)
            {
                bestDist2  = bestDist;
                bestDist   = dist;
                bestLevel2 = bestLevel;
                bestLevel  = CurrentFrame.undistorted_keypoints[idx].octave;
                bestIdx    = idx;
            }
            else if (dist < bestDist2)
            {
                bestLevel2 = CurrentFrame.undistorted_keypoints[idx].octave;
                bestDist2  = dist;
            }
        }

        // Apply ratio to second match (only if best and second are in the same scale level)
        if (bestDist <= TH_HIGH)
        {
            if (bestLevel == bestLevel2 && bestDist > ratio * bestDist2) continue;
            tmp_matches_world_image[i] = {bestIdx, bestDist};
        }
    }

    //    if (0)
    for (int i = 0; i < N; ++i)
    {
        auto bestIdx = tmp_matches_world_image[i].first;
        if (bestIdx == -1) continue;
        if (CurrentFrame.mvpMapPoints[bestIdx]) continue;
        auto&& lmp                         = lm.points[i];
        CurrentFrame.mvpMapPoints[bestIdx] = lmp.mp;
        matches++;
    }
    return matches;
}  // namespace Snake


}  // namespace Snake
