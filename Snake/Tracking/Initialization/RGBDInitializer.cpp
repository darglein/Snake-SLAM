/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#include "RGBDInitializer.h"

#include "Map/Frame.h"
#include "Map/Map.h"
#include "Map/MapPoint.h"
#include "LocalMapping/LocalMapping.h"

namespace Snake
{
InitializerResult RGBDInitializer::Initialize(FramePtr frame)
{
    result.success = false;

    const int minFeaturesInit = 180;


    int featuresWithValidDepth = 0;
    for (int i = 0; i < frame->N; ++i)
    {
        auto z = frame->depth[i];
        if (z > 0)
        {
            featuresWithValidDepth++;
        }
    }

    if (featuresWithValidDepth < minFeaturesInit)
    {
        std::cout << "Not enough depth features: " << featuresWithValidDepth << std::endl;
        return result;
    }

    auto& kf = *map.allocateKeyframe();

    {
        auto lock = map.LockFull();

        SE3 initialPose = SE3();
        frame->tmpPose  = initialPose;

        kf.init(frame, nullptr, nullptr);

        auto invpose         = initialPose.inverse();
        int newMapPointCount = 0;
        int random_remove    = 0;
        for (int i = 0; i < frame->N; ++i)
        {
            auto z = frame->depth[i];
            if (z <= 0) continue;

            if (remove_random_points && Saiga::Random::sampleDouble(0, 1) < 0.1)
            {
                random_remove++;
                continue;
            }

            auto wp  = invpose * K.unproject(frame->undistorted_keypoints[i].point, z);
            auto& mp = map.allocateMapPoint();
            mp.init(kf, wp);

            mp.UpdateNormalLocal();
            mp.UpdateDepthLocal(i);

            mp.AddObservation(&kf, i);
            kf.addMappoint(&mp, i);

            mp.ComputeDistinctiveDescriptors();
            map.AddMapPoint(mp);
            frame->mvpMapPoints[i] = &mp;

            newMapPointCount++;
        }

        if (remove_random_points)
        {
            std::cout << "Stereo Init: Remove " << random_remove << " random points" << std::endl;
        }

        frame->setReference(&kf, SE3());

        kf.ComputeDepthRange();
        frame->validPose = true;
        SAIGA_ASSERT(frame->isKeyframe);

        localMapping->setLocalMapInitial();
        map.AddKeyFrame(kf);
        kf.inMap = true;
        kf.UpdateConnections();

        std::cout << "Stereo/RGBD Initialization SUCCESS! Added " << newMapPointCount << " new map points."
                  << std::endl;
    }

    kf.cull_factor = 0.8;

    result.success = true;
    result.kfLast  = &kf;
    result.kfFirst = &kf;

    localMapping->Process(&kf);
    return result;
}



}  // namespace Snake
