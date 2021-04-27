/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#include "Triangulator.h"

#include "saiga/vision/reconstruction/Epipolar.h"
#include "saiga/vision/reconstruction/Triangulation.h"
namespace Snake
{
Triangulator::~Triangulator() {}

int Triangulator::Process(TriangulationParams params, Keyframe* kf, std::vector<MapPoint*>* out_points)
{
    {
        // The actual triangulation happens read-only into a temp vector
        auto lock = map.LockReadOnly();
        ComputeDepthMap(kf);

        //        SAIGA_BLOCK_TIMER();

        pose1 = kf->Pose();
        tmp_keyframes.clear();
        kf->GetBestCovisibilityKeyFrames(tmp_keyframes, params.num_neighbors);
        std::reverse(tmp_keyframes.begin(), tmp_keyframes.end());


        if (params.only_past_keyframes)
        {
            tmp_keyframes.erase(std::remove_if(tmp_keyframes.begin(), tmp_keyframes.end(),
                                               [kf](auto other_kf) { return other_kf->id() > kf->id(); }),
                                tmp_keyframes.end());
        }

        newPointsa.resize(tmp_keyframes.size());
    }

    int num_threads = settings.num_tracking_threads;
    thread_data.resize(num_threads);
#pragma omp parallel for num_threads(num_threads) schedule(dynamic)
    for (int i = 0; i < tmp_keyframes.size(); ++i)
    {
        auto lock     = map.LockReadOnly();
        newPointsa[i] = triangulate(params, kf, tmp_keyframes[i], i < 3);
    }


    int nnew = 0;

    {
        // Create new points with full lock
        auto lock = map.LockFull();
        //        SAIGA_BLOCK_TIMER();
        std::unique_lock l2(map.mMutexMap);

        SE3 new_pose1 = kf->Pose();
        SE3 T         = new_pose1.inverse() * pose1;

        for (auto& match : newPointsa)
        {
            auto kf1 = match.kf1;
            auto kf2 = match.kf2;
            for (auto& np : match.newPoints)
            {
                if (kf1->GetMapPoint(np.featureId1) || kf2->GetMapPoint(np.featureId2))
                {
                    continue;
                }

                Vec3 wp = T * np.worldPosition;

                MapPoint* mp;
                mp = &map.allocateMapPoint();
                // use the keyframe with the larger id as reference
                if (kf1->id() > kf2->id())
                {
                    mp->init(*kf1, wp);
                }
                else
                {
                    mp->init(*kf2, wp);
                }


                mp->far_stereo_point = np.far_away;

                mp->AddObservation(kf1, np.featureId1);
                mp->AddObservation(kf2, np.featureId2);
                kf1->addMappoint(mp, np.featureId1);
                kf2->addMappoint(mp, np.featureId2);

                mp->ComputeDistinctiveDescriptors();

                mp->UpdateNormal();
                mp->UpdateDepth();

                map.AddMapPoint(*mp);

                if (out_points)
                {
                    out_points->push_back(mp);
                }

                nnew++;
            }
        }
    }
    return nnew;
}

ImageTriangulationResult Triangulator::triangulate(TriangulationParams params, Keyframe* kf1, Keyframe* kf2,
                                                   bool precise)
{
    ASSERT_HAS_READ_LOCK;


    int tid                    = OMP::getThreadNum();
    auto& tmp_matches          = thread_data[tid].tmp_matches;
    auto& triangulationMatcher = thread_data[tid].triangulationMatcher;

    ImageTriangulationResult result;
    result.kf1 = kf1;
    result.kf2 = kf2;

    auto chi2Mono   = params.errorMono * params.errorMono;
    auto chi2Stereo = params.errorStereo * params.errorStereo;



    const float ratioFactor = 1.5f * scalePyramid.Factor();

    //    KeyFrame* kf2 = tmp_keyframes[i];
    auto pose1     = kf1->Pose();
    auto pose2     = kf2->Pose();
    auto position1 = pose1.inverse().translation();
    auto position2 = pose2.inverse().translation();

    auto baseline = (position1 - position2).norm();
    //        std::cout << "Baseline: " << baseline << std::endl;
    if (settings.inputType == InputType::Mono)
    {
        kf2->ComputeDepthRange();
        const float medianDepthKF2     = kf2->MedianDepth();
        const float ratioBaselineDepth = baseline / medianDepthKF2;
        if (ratioBaselineDepth < 0.01)
        {
            //            std::cout << "skipping triangulation for " << kf1->id << " - " << kf2->id << std::endl;
            return result;
        }
    }
    else
    {
        SAIGA_ASSERT(stereo_cam.fx != 1);
        if (baseline < stereo_cam.baseLine()) return result;
    }

    Mat3 E = EssentialMatrix(pose1, pose2);


    tmp_matches.clear();
    {
        triangulationMatcher.SearchForTriangulation2(kf1, kf2, E, tmp_matches, params.epipolarDistance,
                                                     params.feature_distance);
    }

    if (precise)
    {
        triangulationMatcher.SearchForTriangulationProject(grid, pose1, pose2, kf1, kf2, E, tmp_matches,
                                                           params.epipolarDistance, params.feature_distance);
    }

    for (auto [idx1, idx2] : tmp_matches)
    {
        auto& kp1          = kf1->frame->undistorted_keypoints[idx1];
        const float kp1_ur = kf1->frame->right_points[idx1];
        bool bStereo1      = kp1_ur >= 0;
        Vec3 kpp1(kp1.point(0), kp1.point(1), kp1_ur);


        auto& kp2          = kf2->frame->undistorted_keypoints[idx2];
        const float kp2_ur = kf2->frame->right_points[idx2];
        bool bStereo2      = kp2_ur >= 0;
        Vec3 kpp2(kp2.point(0), kp2.point(1), kp2_ur);

        // Check parallax between rays

        Vec3 xn1 = K.unproject(kp1.point, 1);
        Vec3 xn2 = K.unproject(kp2.point, 1);

        // rotate to world space
        Vec3 ray1 = pose1.inverse().so3() * xn1;
        Vec3 ray2 = pose2.inverse().so3() * xn2;


        const float cosParallaxRays = ray1.dot(ray2) / (ray1.norm() * ray2.norm());

        float cosParallaxStereo  = cosParallaxRays + 1;
        float cosParallaxStereo1 = cosParallaxStereo;
        float cosParallaxStereo2 = cosParallaxStereo;

        if (bStereo1)
            cosParallaxStereo1 = cos(2 * atan2(stereo_cam.baseLine() / 2, kf1->frame->depth[idx1]));
        else if (bStereo2)
            cosParallaxStereo2 = cos(2 * atan2(stereo_cam.baseLine() / 2, kf2->frame->depth[idx2]));

        cosParallaxStereo = std::min(cosParallaxStereo1, cosParallaxStereo2);


        float thParall = 0.9998;
        bool far_away  = false;

        Vec3 x3D;
        if (cosParallaxRays < cosParallaxStereo && cosParallaxRays > 0 &&
            (bStereo1 || bStereo2 || cosParallaxRays < thParall))
        {
            Vec2 p1(xn1(0), xn1(1));
            Vec2 p2(xn2(0), xn2(1));
            x3D = TriangulateHomogeneous<double, true>(pose1, pose2, p1, p2);
        }
        else if (bStereo1 && cosParallaxStereo1 < cosParallaxStereo2)
        {
            x3D      = pose1.inverse() * K.unproject(kp1.point, kf1->frame->depth[idx1]);
            far_away = kf1->frame->depth[idx1] > th_depth;
        }
        else if (bStereo2 && cosParallaxStereo2 < cosParallaxStereo1)
        {
            x3D      = pose2.inverse() * K.unproject(kp2.point, kf2->frame->depth[idx2]);
            far_away = kf2->frame->depth[idx2] > th_depth;
        }
        else
        {
            continue;
        }

        // project back into cameras
        xn1 = pose1 * x3D;
        xn2 = pose2 * x3D;

        // check if behind camera
        if (xn1.z() <= 0) continue;
        if (xn2.z() <= 0) continue;


        auto sigmaSquare1 = scalePyramid.SquaredScale(kp1.octave);
        if (bStereo1)
        {
            Vec3 ip  = stereo_cam.projectStereo(xn1);
            Vec3 err = ip - kpp1;
            if (err.squaredNorm() > chi2Stereo * sigmaSquare1) continue;
        }
        else
        {
            Vec2 ip  = K.project(xn1);
            Vec2 err = ip - kp1.point;
            if (err.squaredNorm() > chi2Mono * sigmaSquare1) continue;
        }

        auto sigmaSquare2 = scalePyramid.SquaredScale(kp2.octave);
        if (bStereo2)
        {
            Vec3 ip  = stereo_cam.projectStereo(xn2);
            Vec3 err = ip - kpp2;
            if (err.squaredNorm() > chi2Stereo * sigmaSquare2) continue;
        }
        else
        {
            Vec2 ip  = K.project(xn2);
            Vec2 err = ip - kp2.point;
            if (err.squaredNorm() > chi2Mono * sigmaSquare2) continue;
        }

        auto dist1 = (position1 - x3D).norm();
        auto dist2 = (position2 - x3D).norm();
        if (dist1 == 0 || dist2 == 0) continue;


        auto ratioDist   = dist2 / dist1;
        auto ratioOctave = scalePyramid.Scale(kp1.octave) / scalePyramid.Scale(kp2.octave);

        if (ratioDist * ratioFactor < ratioOctave || ratioDist > ratioOctave * ratioFactor) continue;

        ImageTriangulationResult::NewPoint np;
        np.featureId1    = idx1;
        np.featureId2    = idx2;
        np.worldPosition = x3D;
        np.far_away      = far_away;

        result.newPoints.push_back(np);
    }

    return result;
}

void Triangulator::ComputeDepthMap(Keyframe* kf)
{
    grid.resize(featureGridBounds.Rows / 4 + 1, featureGridBounds.Cols / 4 + 1);
    grid_bool.resize(featureGridBounds.Rows / 4 + 1, featureGridBounds.Cols / 4 + 1);
    grid.setZero();
    grid_bool.setZero();


    SE3 pose = kf->Pose();
    for (auto i : kf->frame->featureRange())
    {
        auto mp = kf->GetMapPoint(i);
        if (!mp) continue;

        auto z = (pose * mp->position).z();


        if (z > 0 && Random::sampleDouble(0, 1) > 0.33)
        {
            auto cell = featureGridBounds.cellClamped(kf->frame->undistorted_keypoints[i].point);
            cell.first /= 4;
            cell.second /= 4;
            SAIGA_ASSERT(cell.second >= 0 && cell.second < grid.rows());
            SAIGA_ASSERT(cell.first >= 0 && cell.first < grid.cols());
            grid(cell.second, cell.first)      = z;
            grid_bool(cell.second, cell.first) = 1;
        }
    }

    for (int j = 0; j < grid.cols(); ++j)
    {
        double current = 0;
        for (int i = 0; i < grid.rows(); ++i)
        {
            if (grid(i, j) == 0)
            {
                grid(i, j) = current;
            }
            else
            {
                current = grid(i, j);
            }
        }
    }

    for (int i = 0; i < grid.rows(); ++i)
    {
        double current = 0;
        for (int j = 0; j < grid.cols(); ++j)
        {
            if (grid(i, j) == 0)
            {
                grid(i, j) = current;
            }
            else
            {
                current = grid(i, j);
            }
        }
    }


    for (int k = 0; k < 5; ++k)
    {
        for (int i = 1; i < grid.rows() - 1; ++i)
        {
            for (int j = 1; j < grid.cols() - 1; ++j)
            {
                if (grid_bool(i, j) == 0)
                {
                    auto sum   = grid(i + 1, j) + grid(i - 1, j) + grid(i, j + 1) + grid(i, j - 1);
                    grid(i, j) = sum / 4.0;
                }
            }
        }
    }
    //    std::cout << "depth copuated " << grid << std::endl;
}

}  // namespace Snake
