/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once

#include "saiga/vision/reconstruction/RobustPoseOptimization.h"
#include "saiga/vision/reconstruction/RobustSmoothPoseOptimization.h"

#include "Map/LocalMap.h"
#include "Map/Map.h"

namespace Snake
{
/**
 * Given a frame with 2D to 3D matches and an initial guess, this class computes
 * the best pose.
 */
class PoseRefinement
{
   public:
    PoseRefinement(double errorFactor = 1.0);

    template <typename PointType>
    int refinePose(Frame& frame, const LocalMap<PointType>& lm)
    {
        SE3Type pose = frame.Pose().cast<T>();

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

            int lid = lm.localIndex(pMP);
            if (lid == -1) continue;

            frame.mvbOutlier[i] = false;
            const auto& kp      = frame.undistorted_keypoints[i];

            Obs o;
            o.ip = kp.point.cast<T>();

            o.weight = scalePyramid.InverseSquaredScale(kp.octave);
            o.weight = sqrt(o.weight);
            o.depth  = frame.depth[i];

            obs.push_back(o);
            wps.push_back(lm.points[lid].position.template cast<T>());
            idx.push_back(i);
            outlier.push_back(false);
        }

        int inliers;

        // In monocular mode we just use a dummy camera because the pose optimizer doesn't use bf for mono points
        // anyways.
        StereoCamera4 cam = settings.inputType == InputType::Mono ? mono_intrinsics.dummyStereoCamera() : stereo_cam;

        if (frame.prediction_weight_rotation > 0)
        {
            inliers =
                rpo_smooth.optimizePoseRobust(wps, obs, outlier, pose, cam.cast<T>(), frame.prediction,
                                              frame.prediction_weight_rotation, frame.prediction_weight_translation);
        }
        else
        {
            inliers = rpo.optimizePoseRobust(wps, obs, outlier, pose, cam.cast<T>());
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

    int RefinePoseWithMatches(Frame& frame);

   private:
    using T       = double;
    using SE3Type = Sophus::SE3<T>;
    using Vec3    = Eigen::Matrix<T, 3, 1>;
    using Vec2    = Eigen::Matrix<T, 2, 1>;
    using Obs     = ObsBase<T>;
    AlignedVector<int> idx;
    AlignedVector<Vec3> wps;
    AlignedVector<Obs> obs;
    AlignedVector<int> outlier;

    RobustPoseOptimization<T, false> rpo;
    RobustSmoothPoseOptimization<T, false> rpo_smooth;
};


}  // namespace Snake
