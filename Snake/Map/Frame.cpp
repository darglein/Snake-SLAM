/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */
#include "Frame.h"

#include "Map/Keyframe.h"
#include "Map/MapPoint.h"


namespace Snake
{
Frame::Frame(int _id) {}

Frame::~Frame()
{
    //    std::cout << "~Frame " << id << std::endl;
}

void Frame::allocateTmp()
{
    mvpMapPoints.resize(N, nullptr);
    mvbOutlier.resize(N, false);
    right_points.resize(N, -1000);
    depth.resize(N, -1000);
    undistorted_keypoints.reserve(N);
    normalized_points.resize(N);
}
std::ostream& operator<<(std::ostream& strm, const Frame& frame)
{
    strm << "[Frame] " << frame.id;
    return strm;
}

void Frame::computeBoW()
{
    if (bow_vec.empty())
    {
        vocabulary.transform(descriptors, bow_vec, bow_feature_vec, 4, settings.num_tracking_threads);
    }
}

void Frame::setReference(Keyframe* kf, const SE3& p)
{
    SAIGA_ASSERT(kf);
    mpReferenceKF = kf;
    reltoRef      = p;
}

void Frame::setReference(Keyframe* kf)
{
    SAIGA_ASSERT(kf);
    mpReferenceKF = kf;
    auto p1       = Pose();
    auto p2       = kf->PoseGlobal();
    reltoRef      = p1 * p2.inverse();
}

void Frame::UpdateRelFromTmpPose()
{
    auto p1  = Pose();
    auto p2  = mpReferenceKF->PoseGlobal();
    reltoRef = p1 * p2.inverse();
}

SE3 Frame::getPoseFromReference()
{
    SAIGA_ASSERT(mpReferenceKF);
    return reltoRef * mpReferenceKF->PoseGlobal();
}

void Frame::UpdateReference()
{
    auto kf = mpReferenceKF;
    while (kf && kf->isBad())
    {
        kf = kf->GetParent();
    }
    if (!kf)
    {
        validPose = false;
    }
}


int Frame::removeOutliers(double thresholdMono, double thresholdStereo)
{
    int removed_points = 0;
    for (auto i : featureRange())
    {
        auto mp = mvpMapPoints[i];
        if (!mp || mp->isBad()) continue;

        if (hasDepth(i))
        {
            SAIGA_ASSERT(stereo_cam.fx != 1);
            Vec3 p  = tmpPose * mp->position;
            Vec3 ip = stereo_cam.projectStereo(p);
            Vec3 diff =
                ip - Vec3(undistorted_keypoints[i].point(0), undistorted_keypoints[i].point(1), right_points[i]);
            auto error = diff.squaredNorm();
            if (p.z() <= 0 || error > thresholdStereo)
            {
                mvpMapPoints[i] = nullptr;
                removed_points++;
            }
        }
        else
        {
            Vec3 p     = Pose() * mp->position;
            Vec2 ip    = mono_intrinsics.model.K.project(p);
            Vec2 diff  = ip - undistorted_keypoints[i].point;
            auto error = diff.squaredNorm();
            if (p.z() <= 0 || error > thresholdMono)
            {
                mvpMapPoints[i] = nullptr;
                removed_points++;
            }
        }
    }
    return removed_points;
}


}  // namespace Snake
