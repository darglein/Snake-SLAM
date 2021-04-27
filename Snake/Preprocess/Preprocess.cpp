/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#include "Preprocess.h"

#include "FeatureDetector.h"
#include "Input.h"
#include "opencv2/imgproc.hpp"
namespace Snake
{
Preprocess::Preprocess() : Module(ModuleType::PREPROCESS)
{
    thread = Thread([this]() {
        Saiga::Random::setSeed(settings.randomSeed);
        Saiga::setThreadName("Preprocess");
        while (true)
        {
            auto frame = featureDetector->GetFrame();

            if (!frame)
            {
                break;
            }
            Process(*frame);
            output_buffer.set(frame);
        }
        output_buffer.set(nullptr);
    });
    CreateTable({7, 12, 10}, {"Frame", "Stereo P.", "Time (ms)"});
}

void Preprocess::Process(Frame& frame)
{
    int stereo_matches = 0;
    {
        auto timer = ModuleTimer();
        frame.allocateTmp();
        undistortKeypoints(frame);
        computeFeatureGrid(frame);
        if (settings.inputType == InputType::RGBD)
        {
            stereo_matches = ComputeStereoFromRGBD(frame);
        }
        else if (settings.inputType == InputType::Stereo)
        {
            stereo_matches = StereoMatching(frame);
        }
    }
    (*output_table) << frame.id << stereo_matches << LastTime();
}

void Preprocess::undistortKeypoints(Frame& frame)
{
    SAIGA_ASSERT(frame.undistorted_keypoints.size() == 0);
    for (int i = 0; i < frame.N; i++)
    {
        frame.undistorted_keypoints.emplace_back(frame.keypoints[i]);

        Vec2 p = frame.keypoints[i].point;

        SAIGA_ASSERT(frame.image.inImage(p.y(), p.x()));

        p = rect_left.K_src.unproject2(p);
        //                Vec2 p_normalized = undistortNormalizedPoint(p, rect_left.D_src);
        Vec2 p_normalized = undistortPointGN(p, p, rect_left.D_src);

        Vec3 p_r = rect_left.R * Vec3(p_normalized(0), p_normalized(1), 1);

        p                                    = Vec2(p_r(0) / p_r(2), p_r(1) / p_r(2));
        frame.normalized_points[i]           = p;
        p                                    = rect_left.K_dst.normalizedToImage(p);
        frame.undistorted_keypoints[i].point = p;
    }
}

int Preprocess::ComputeStereoFromRGBD(Frame& frame)
{
    SAIGA_ASSERT(settings.inputType == InputType::RGBD);
    SAIGA_ASSERT(frame.depth_image.valid());

    auto N = frame.N;


    int num_matches = 0;


    for (int i = 0; i < N; i++)
    {
        auto& kpun = frame.undistorted_keypoints[i];

        Vec2 normalized_point = K.unproject2(kpun.point);
        Vec2 distorted_point  = distortNormalizedPoint(normalized_point, rgbd_intrinsics.depthModel.dis);
        Vec2 reprojected      = rgbd_intrinsics.depthModel.K.normalizedToImage(distorted_point);

        int x = reprojected.x() + 0.5;
        int y = reprojected.y() + 0.5;

        SAIGA_ASSERT(frame.depth_image.inImage(y, x));
        auto depth = frame.depth_image(y, x);

        SAIGA_ASSERT(depth >= 0);
        SAIGA_ASSERT(depth < 20);
        if (depth > 0)
        {
            frame.depth[i]        = depth;
            auto disparity        = rgbd_intrinsics.bf / depth;
            frame.right_points[i] = kpun.point(0) - disparity;
            num_matches++;
        }
        else
        {
            frame.depth[i]        = -1;
            frame.right_points[i] = -1;
        }
    }
    return num_matches;
}

int Preprocess::StereoMatching(Frame& frame)
{
    SAIGA_ASSERT(settings.inputType == InputType::Stereo);
    SAIGA_ASSERT(frame.keypoints.size() > 0);
    SAIGA_ASSERT(frame.keypoints_right.size() > 0);

    // Transform points to rectified space
    std::vector<KeyPoint> left_rectified  = frame.keypoints;
    std::vector<KeyPoint> right_rectified = frame.keypoints_right;

    int min_y = 1023123;
    int max_y = -19284;


    // Set limits for search
    const float min_disp = 0;
    const float max_disp = rect_left.bf * 0.5;

    for (int i = 0; i < left_rectified.size(); ++i)
    {
        left_rectified[i].point = rect_left.Forward(frame.keypoints[i].point);
    }

    for (int i = 0; i < right_rectified.size(); ++i)
    {
        right_rectified[i].point = rect_right.Forward(frame.keypoints_right[i].point);
        min_y                    = std::min(min_y, Saiga::iRound(right_rectified[i].point.y()));
        max_y                    = std::max(max_y, Saiga::iRound(right_rectified[i].point.y()));
    }

    std::vector<std::vector<int>> row_map(max_y - min_y + 1);
    for (int i = 0; i < right_rectified.size(); ++i)
    {
        int row = Saiga::iRound(right_rectified[i].point.y()) - min_y;
        SAIGA_ASSERT(row >= 0 && row < row_map.size());
        row_map[row].push_back(i);
    }

    int num_matches = 0;
    for (int i = 0; i < left_rectified.size(); ++i)
    {
        auto& kp = left_rectified[i];

        int y        = Saiga::iRound(left_rectified[i].point.y());
        int y_center = y - min_y;

        // go over +-r rows
        float r = ceil(2.0f * scalePyramid.Scale(kp.octave));

        int best_id          = -1;
        int best_dist        = 250;
        int second_best_dist = 250;

        for (int row = y_center - r; row <= y_center + r; ++row)
        {
            if (row < 0 || row >= row_map.size()) continue;

            for (auto other_id : row_map[row])
            {
                double disparity = left_rectified[i].point.x() - right_rectified[other_id].point.x();
                if (disparity < min_disp || disparity > max_disp)
                {
                    continue;
                }

                if (std::abs(left_rectified[i].octave - right_rectified[other_id].octave) > 1)
                {
                    continue;
                }

                auto dist = distance(frame.descriptors[i], frame.descriptors_right[other_id]);
                if (dist < best_dist)
                {
                    second_best_dist = best_dist;
                    best_dist        = dist;
                    best_id          = other_id;
                }
                else if (dist < second_best_dist)
                {
                    second_best_dist = dist;
                }
            }
        }


        if (best_dist > (settings.fd_relaxed_stereo ? 75 : 40)) continue;

        if (best_dist > (settings.fd_relaxed_stereo ? 0.9 : 0.7) * second_best_dist)
        {
            continue;
        }

        auto angle1 = left_rectified[i].angle;
        auto angle2 = right_rectified[best_id].angle;
        float rot   = std::min(std::abs(angle1 - angle2),
                             std::min(std::abs((angle1 + 365) - angle2), std::abs(angle1 - (angle2 + 365))));

        if (rot > (settings.fd_relaxed_stereo ? 25 : 5))
        {
            continue;
        }


        auto right_point = right_rectified[best_id].point.x();

        double disparity = left_rectified[i].point.x() - right_point;

        if (disparity <= 0.001)
        {
            disparity   = 0.001;
            right_point = left_rectified[i].point.x() - disparity;
        }

        frame.right_points[i] = right_point;
        frame.depth[i]        = rect_left.bf / disparity;
        num_matches++;

        SAIGA_ASSERT(frame.depth[i] > 0);
    }
    return num_matches;
}

void Preprocess::computeFeatureGrid(Frame& frame)
{
    auto permutation = frame.grid.create(featureGridBounds, frame.undistorted_keypoints);

    int N = permutation.size();
    std::vector<KeyPoint> mvKeys2(N);
    std::vector<Saiga::DescriptorORB> descriptors2(N);
    std::vector<KeyPoint> mvKeysUn2(N);
    AlignedVector<Vec2> norm2(N);

    for (int i = 0; i < N; ++i)
    {
        mvKeys2[permutation[i]]      = frame.keypoints[i];
        descriptors2[permutation[i]] = frame.descriptors[i];
        mvKeysUn2[permutation[i]]    = frame.undistorted_keypoints[i];
        norm2[permutation[i]]        = frame.normalized_points[i];
    }

    frame.keypoints.swap(mvKeys2);
    frame.descriptors.swap(descriptors2);
    frame.undistorted_keypoints.swap(mvKeysUn2);
    frame.normalized_points.swap(norm2);
}

}  // namespace Snake
