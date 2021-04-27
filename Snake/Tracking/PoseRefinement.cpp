/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#include "PoseRefinement.h"



namespace Snake
{
PoseRefinement::PoseRefinement(double errorFactor)
    : rpo(reprojectionErrorThresholdMono * errorFactor, reprojectionErrorThresholdStereo * errorFactor),
      rpo_smooth(reprojectionErrorThresholdMono * errorFactor, reprojectionErrorThresholdStereo * errorFactor)
{
    int maxPointsPerImage = 5000;
    idx.reserve(maxPointsPerImage);
    wps.reserve(maxPointsPerImage);
    obs.reserve(maxPointsPerImage);
    outlier.reserve(maxPointsPerImage);
}


int PoseRefinement::RefinePoseWithMatches(Frame& frame)
{
    int nInitialCorrespondences = 0;
    SE3Type pose                = frame.Pose().cast<T>();


    idx.clear();
    wps.clear();
    obs.clear();
    outlier.clear();


    for (auto i : frame.featureRange())
    {
        MapPoint* pMP = frame.mvpMapPoints[i];
        // the mappoint might be set to bad,
        // but we don't care here because the pose refinement is robust anyways
        if (!pMP) continue;

        nInitialCorrespondences++;
        frame.mvbOutlier[i] = false;
        const auto& kp      = frame.undistorted_keypoints[i];

        Obs o;
        o.ip     = kp.point.cast<T>();
        o.weight = scalePyramid.InverseSquaredScale(kp.octave);
        o.weight = sqrt(o.weight);
        o.depth  = frame.depth[i];

        obs.push_back(o);
        wps.push_back(pMP->getPosition().cast<T>());
        idx.push_back(i);
        outlier.push_back(false);
    }

    if (nInitialCorrespondences < 3) return 0;
    int inliers;

    {
        // In monocular mode we just use a dummy camera because the pose optimizer doesn't use bf for mono points
        // anyways.
        StereoCamera4 cam = settings.inputType == InputType::Mono ? mono_intrinsics.dummyStereoCamera() : stereo_cam;
        inliers           = rpo.optimizePoseRobust(wps, obs, outlier, pose, cam.cast<T>());
    }


    // Apply optimized pose and outliers to frame
    for (size_t i = 0; i < outlier.size(); i++)
    {
        int id               = idx[i];
        frame.mvbOutlier[id] = outlier[i];
    }
    frame.setPose(pose.cast<double>());
    return inliers;
}



}  // namespace Snake
