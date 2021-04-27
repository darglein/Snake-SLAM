/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#include "MonoInitializer.h"

#include "saiga/core/util/table.h"
#include "saiga/vision/cameraModel/MotionModel.h"
#include "saiga/vision/reconstruction/Homography.h"
#include "saiga/vision/reconstruction/TwoViewReconstruction.h"
#include "saiga/vision/util/HistogramImage.h"

#include "LocalMapping/LocalMapping.h"
#include "Map/MapPoint.h"
#include "Optimizer/LocalBundleAdjustment.h"

namespace Snake
{
MonoInitializer::MonoInitializer(int quality) : Initializer(quality)
{
    quality = std::min(2, std::max(quality, 0));
    switch (quality)
    {
        case 0:
            params.MakeLowQuality();
            break;
        case 1:
            params.MakeMediumQuality();
            break;
        case 2:
            params.MakeHighQuality();
            break;
    }

#ifdef USE_FIVE_POINT
    {
        RansacParameters rparams;
        rparams.maxIterations   = params.five_point_ransac_iterations;
        double epipolarTheshold = params.five_point_threshold_px / K.fx;
        //    double epipolarTheshold   = 1.5 / 530;
        rparams.residualThreshold = epipolarTheshold * epipolarTheshold;
        rparams.reserveN          = 2000;
        rparams.threads           = params.num_threads;
        tvr.init(rparams);
    }
#else
    {
        RansacParameters rparams;
        rparams.maxIterations     = params.five_point_ransac_iterations;
        double epipolarTheshold   = params.five_point_threshold_px;
        rparams.residualThreshold = epipolarTheshold * epipolarTheshold;
        rparams.reserveN          = 2000;
        rparams.threads           = settings.num_tracking_threads;
        tvr.init(rparams, K);
    }
#endif

    {
        RansacParameters rp;
        rp.maxIterations = 100;
        rp.reserveN      = 2000;
        rp.threads       = settings.num_tracking_threads;

        double epipolarTheshold = params.five_point_threshold_px / K.fx;

        rp.residualThreshold = epipolarTheshold * epipolarTheshold;
        hr.init(rp);
    }

    //    std::cout << "Create mono initializer with " << params.num_threads << " threads" << std::endl;

    SAIGA_ASSERT(K.fx != 1);
}


std::vector<std::string> state_names = {"SUCCESS", "MATCHES", "INLIERS", "ANGLE", "HISTO", "PLANAR"};

std::ostream& operator<<(std::ostream& strm, const MonoInitInfo& info)
{
    Saiga::Table tab({8, 5, 5, 5, 5, 10, 10, 10, 10, 5}, strm);

#if 1
    tab << "[Init]" << info.frame1 << info.frame2 << info.matches << info.inliers << info.homographyInliers
        << info.angle << info.histo;
    if (info.state == MonoInitInfo::State::SUCCESS)
        tab << " SUCCESS;";
    else
        tab << " FAILED " + state_names[(int)info.state];
#endif
    return strm;
}


MonoInitInfo MonoInitializer::ComputeRelativeTransformation(FramePtr frame1, FramePtr frame2)
{
    MonoInitInfo info;

    info.frame1 = frame1->id;
    info.frame2 = frame2->id;


    //    int matches2 = ComputeMatchesPrediction(left, right);
    //    info.matches = ComputeMatchesBF(left, right);

    //    std::cout << "Matches Prediction/BF " << matches2 << " " << info.matches << std::endl;


    info.matches = ComputeMatchesPrediction(frame1, frame2);
    if (info.matches < params.min_matches)
    {
        info.state = MonoInitInfo::State::NOT_ENOUGH_MATCHES;
        return info;
    }


    std::vector<double> distances;
    for (int i = 0; i < info.matches; ++i)
    {
        distances.push_back((points1[i] - points2[i]).norm());
    }
    std::sort(distances.begin(), distances.end());
    if (distances[distances.size() / 2] < 10)
    {
        info.state = MonoInitInfo::State::NOT_ENOUGH_INLIERS;
        return info;
    }



    {
//        SAIGA_BLOCK_TIMER();
#ifdef USE_FIVE_POINT
        tvr.compute(normalized_points1, normalized_points2);
#else
        tvr.compute(points1, points2, normalized_points1, normalized_points2);
#endif
    }

    info.inliers = tvr.inlierCount;
    if (tvr.inlierCount < params.min_inliers)
    {
        info.state = MonoInitInfo::State::NOT_ENOUGH_INLIERS;
        return info;
    }

    //    tvr.scene.rmsPrint();
    //    std::cout << "Inliers " << info.inliers << "/" << info.matches << " " << tvr.scene.rms() << std::endl;
    tvr.scene.removeOutliers(3);
    //    tvr.scene.rmsPrint();
    //    std::cout << "Inliers " << info.inliers << "/" << info.matches << " " << tvr.scene.rms() << std::endl;

    {
#ifdef USE_FIVE_POINT
        auto th = reprojectionErrorThresholdMono / K.fx;
#else
        auto th = reprojectionErrorThresholdMono;
#endif

        // tvr.scene.removeOutliers(
        //        double initial_rms = tvr.scene.rms();
        //        if (initial_rms > reprojectionErrorThresholdMono)
        //        {
        //            std::cout << "Skipping init rms = " << initial_rms << std::endl;
        //            info.state = MonoInitInfo::State::NOT_ENOUGH_INLIERS;
        //            return info;
        //        }

        //        tvr.scene.removeOutliers(reprojectionErrorThresholdMono * 1.5);

#if WITH_IMU
        if (settings.initial_bias_gyro.squaredNorm() > 0)
        {
            // collect imu data
            auto frame = frame2;
            std::vector<FramePtr> frames_since_start;
            while (frame != nullptr && frame != frame1)
            {
                frames_since_start.push_back(frame);
                frame = frame->previousFrame;
            }

            Imu::ImuSequence imudata12 = frames_since_start.back()->imu_data;
            // reverse iterate through frames and insert imu data into current vector
            for (int i = (int)frames_since_start.size() - 2; i >= 0; --i)
            {
                auto f = frames_since_start[i];
                imudata12.Add(f->imu_data);
            }

            SAIGA_ASSERT(imudata12.time_begin == frame1->timeStamp);
            SAIGA_ASSERT(imudata12.time_end == frame2->timeStamp);

            //            std::cout << "using imu weight" << std::endl;

            Imu::Preintegration preint(settings.initial_bias_gyro);
            preint.IntegrateMidPoint(imudata12, false);
            SE3 predicted_p2 = preint.Predict(SE3(), Vec3::Zero(), Vec3::Zero()).first;


            SE3 delta_R_body = predicted_p2;
            SE3 delta_R_camera =
                (mono_intrinsics.camera_to_body * delta_R_body * mono_intrinsics.camera_to_body.inverse());

            tvr.rel_pose_prediction = delta_R_camera.inverse();

            double wg = settings.weight_gyro_initialization * settings.weight_gyro_optimization /
                        (imu.omega_sigma * sqrt(imu.frequency) * sqrt(preint.delta_t));
            tvr.rel_pose_weight_rotation = wg;
            //            tvr.rel_pose_weight_rotation =
            //                settings.weight_gyro_initialization * settings.weight_gyro_optimization / preint.delta_t;


            info.inliers = tvr.optimize(3, th);
        }
        else
#endif
        {
            tvr.scene.rel_pose_constraints.clear();
            tvr.rel_pose_weight_rotation    = 0;
            tvr.rel_pose_weight_translation = 0;
            //            tvr.scene.rms();
            //            tvr.scene.chi2();

            //            auto before = tvr.scene.rms();
            //            std::cout << tvr.scene.rms() << " " << tvr.scene.chi2() << std::endl;
            info.inliers = tvr.optimize(3, th);
            //            auto after   = tvr.scene.rms();
            //            std::cout << "Optimize: " << before << " -> " << after << std::endl;
        }
    }


    if (tvr.inlierCount < params.min_inliers)
    {
        info.state = MonoInitInfo::State::NOT_ENOUGH_INLIERS;
        return info;
    }

    info.angle = degrees(tvr.medianAngle());


    if (info.angle < params.min_angle)
    {
        info.state = MonoInitInfo::State::ANGLE_TOO_SMALL;
        return info;
    }

    {
        h_points1.clear();
        h_points2.clear();
        for (auto i = 0; i < tvr.N; ++i)
        {
            if (!tvr.inlierMask[i]) continue;

            h_points1.push_back(normalized_points1[i]);
            h_points2.push_back(normalized_points2[i]);
        }

        Mat3 H;
        info.homographyInliers = hr.solve(h_points1, h_points2, H);
    }


    if ((float)info.homographyInliers / info.inliers > params.max_homography_ratio)
    {
        info.state = MonoInitInfo::State::PLANAR_FAIL;
        return info;
    }

    info.state = MonoInitInfo::State::SUCCESS;

    tvr.setMedianDepth(MonoInitializer::target_scale);
    return info;
}

InitializerResult MonoInitializer::InitializeMap(const MonoInitInfo& info, FramePtr frame1, FramePtr frame2)
{
    auto& kf1 = *map.allocateKeyframe();
    auto& kf2 = *map.allocateKeyframe();

    {
        //        std::unique_lock lock(map.mutexUpdate);
        auto lock = map.LockFull();
        // create and add first keyframe
        kf1.init(frame1, nullptr, nullptr);
        frame1->setReference(&kf1, SE3());
        frame2->setReference(&kf1, tvr.pose2());

        // create and add second keyframe
        kf2.init(frame2, &kf1, &kf1);
        kf2.SetPose(kf2.poseFromRef());
        frame2->setReference(&kf2, SE3());


        // add all reconstructed mappoints
        int newPoints = 0;
        for (int i = 0; i < tvr.N; ++i)
        {
            if (!tvr.inlierMask[i]) continue;

            // 3d world point
            auto& p = tvr.scene.worldPoints[i].p;
            //        auto matchId = tvr.inliers[i];
            auto matchId = i;
            // keypoint id in first and second image
            auto [id1, id2] = matcher.matches[matchId];

            // check if it already exists
            // - can happen if multi matches are allowed
            if (frame1->mvpMapPoints[id1] || frame2->mvpMapPoints[id2]) continue;

            auto& mp = map.allocateMapPoint();
            mp.init(kf1, p);

            mp.AddObservation(&kf1, id1);
            mp.AddObservation(&kf2, id2);
            kf1.addMappoint(&mp, id1);
            kf2.addMappoint(&mp, id2);

            mp.UpdateDepthLocal(id1);
            mp.UpdateNormal();

            mp.ComputeDistinctiveDescriptors();
            map.AddMapPoint(mp);
            frame1->mvpMapPoints[id1] = &mp;
            frame2->mvpMapPoints[id2] = &mp;
            ++newPoints;
        }


        kf1.ComputeDepthRange();
        kf2.ComputeDepthRange();

        frame1->validPose = true;
        frame2->validPose = true;
        SAIGA_ASSERT(frame1->isKeyframe);
        SAIGA_ASSERT(frame2->isKeyframe);

        localMapping->setLocalMapInitial();
        map.AddKeyFrame(kf1);
        kf1.inMap = true;
        kf1.UpdateConnections();
    }
    localMapping->Process(&kf1);

    {
        auto lock = map.LockFull();
        map.AddKeyFrame(kf2);
        kf2.inMap = true;
        kf2.UpdateConnections();
    }
    localMapping->Process(&kf2);


    {
        double angle;
        {
            auto lock             = map.LockReadOnly();
            double medianDepthKF2 = kf1.MedianDepth();
            double baseline       = (kf1.CameraPosition() - kf2.CameraPosition()).norm();
            angle                 = degrees(atan2(baseline / 2.0, medianDepthKF2));
        }
        if (angle < 0.5 * info.angle)
        {
            //
            std::cout << "angle dropped after optimization: " << info.angle << " -> " << angle << std::endl;
#if 0
            map.Clear();
            frame1->clear();
            frame2->clear();
            return result;
#endif
        }
    }

    SAIGA_ASSERT(frame1->getRel().translation() == SE3().translation());
    SAIGA_ASSERT(frame2->getRel().translation() == SE3().translation());

    frames.clear();

    result.success = true;
    result.kfFirst = &kf1;
    result.kfLast  = &kf2;


    kf1.cull_factor = 0.7;
    kf2.cull_factor = 1.0;

    std::cout << "Initialization done." << std::endl;
    return result;
}

auto featureHistogram(FramePtr left)
{
    int w = featureGridBounds.bmax.x();
    int h = featureGridBounds.bmax.y();
    //    int h  = left->grayImage.h;
    //    int w  = left->grayImage.w;
    int bw = w / 48;
    int bh = h / 48;
    HistogramImage hi1(w, h, bw, bh);

    for (auto i : left->featureRange())
    {
        Vec2 ipLeft = left->undistorted_keypoints[i].point;
        hi1.add(ipLeft.y(), ipLeft.x(), 1);
    }
    return hi1.density(1);
}

auto checkHistogram(FramePtr left, FramePtr right, TwoViewReconstruction& tvr,
                    BruteForceMatcher<FeatureDescriptor>& matcher)
{
    int binLimit = 5;
    int h        = left->image.h;
    int w        = left->image.w;
    int bw       = w / 48;
    int bh       = h / 48;

    HistogramImage hi1(w, h, bw, bh);
    HistogramImage hi2(w, h, bw, bh);

    for (int i = 0; i < tvr.inlierCount; ++i)
    {
        // 3d world point
        //        auto& p      = tvr.worldPoints[i];
        auto matchId = tvr.inliers[i];
        // keypoint id in first and second image
        auto [id1, id2] = matcher.matches[matchId];

        Vec2 ipLeft  = left->undistorted_keypoints[id1].point;
        Vec2 ipRight = right->undistorted_keypoints[id2].point;

        hi1.add(ipLeft.y(), ipLeft.x(), 1);
        hi2.add(ipRight.y(), ipRight.x(), 1);
    }


    auto compute = [=](auto& img) {
        int clampedSum = 0;
        int nonZero    = 0;
        for (int i = 0; i < bh; ++i)
        {
            for (int j = 0; j < bw; ++j)
            {
                clampedSum += std::min(binLimit, img(i, j));
                nonZero += (img(i, j) > 0 ? 1 : 0);
            }
        }
        return std::make_pair(clampedSum, nonZero / float(bw * bh));
    };


    auto p1 = compute(hi1);
    auto p2 = compute(hi2);

    return std::make_pair(std::min(p1.first, p2.first), std::min(p1.second, p2.second));
    //    std::cout << "Histogram 1: " << p1.first << " " << p1.second << " " << p1.second / float(bw * bh) <<
    //    std::endl; std::cout << "Histogram 2: " << p2.first << " " << p2.second << " " << p2.second / float(bw * bh)
    //    << std::endl;


    //    hi1.writeBinary("hi1.png");
    //    hi2.writeBinary("hi2.png");
}

void MonoInitializer::selectFirstFrame()
{
    auto validFirst = [=](auto frame) {
        auto ratio = featureHistogram(frame);
        return (ratio > params.min_histogram_density);
        return true;
    };

    bool found = false;

    auto distance = std::max<int>(1, (frames.size() - currentFirst) / 2);
    currentFirst += distance;
    currentFirst = std::min<int>(currentFirst, (int)frames.size() - 1);
    currentFirst = frames.size() - 1;

    while (currentFirst < (int)frames.size())
    {
        if (validFirst(frames[currentFirst]))
        {
            firstFrame = frames[currentFirst];
            found      = true;
            break;
        }

        if (currentFirst == (int)frames.size() - 1) break;

        // didn't find good first
        // -> select mid from here to end
        auto distance = std::max<int>(1, (frames.size() - currentFirst) / 2);
        currentFirst += distance;
        currentFirst = std::min<int>(currentFirst, (int)frames.size() - 1);
    }


    if (found)
    {
        firstFrame = frames[currentFirst];


        last_frame_feature_position.clear();
        for (auto& p : firstFrame->undistorted_keypoints)
        {
            last_frame_feature_position.push_back(p.point);
        }
    }
}



InitializerResult MonoInitializer::Initialize(FramePtr frame)
{
    //    std::cout << "init " << frame->id << std::endl;
    frames.push_back(frame);
    result.success = false;

    if (!firstFrame)
    {
        selectFirstFrame();
        if (firstFrame == nullptr || firstFrame == frame)
        {
            return result;
        }
    }


#if WITH_IMU

    double delta_time = frame->timeStamp - firstFrame->timeStamp;
    if (delta_time > 2.0 * max_time_between_kf_map)
    {
        std::cout << "Skipping Init (delta T)" << std::endl;
        firstFrame = nullptr;
        return result;
    }

#endif


    auto initResult = ComputeRelativeTransformation(firstFrame, frame);



    if (initResult.state == MonoInitInfo::State::NOT_ENOUGH_MATCHES)
    {
        // not enough 2-view matches. Init probably took too long
        // -> reset first frame
        firstFrame = nullptr;
    }

#if 1
    if (initResult.state == MonoInitInfo::State::SUCCESS && firstFrame->image.h > 0)
    {
        auto [binnedInliers, binratio] = checkHistogram(firstFrame, frame, tvr, matcher);
        initResult.histo               = binratio;

        if (binratio < params.min_histogram_density)
        {
            initResult.state = MonoInitInfo::State::HISTOGRAM_FAILURE;
        }
    }
#endif

    std::cout << initResult << std::endl;

    if (initResult.state != MonoInitInfo::State::SUCCESS)
    {
        return result;
    }



    return InitializeMap(initResult, firstFrame, frame);
}

int MonoInitializer::ComputeMatchesBF(FramePtr left, FramePtr right)
{
    //    SAIGA_BLOCK_TIMER();
    int num_matches;
    {
        //        SAIGA_BLOCK_TIMER();
        matcher.matchKnn2_omp(left->descriptors, right->descriptors, settings.num_tracking_threads);
        //        matcher.matchKnn2_omp(left->descriptors, right->descriptors, 2);
        //        matcher.matchKnn2_omp(left->descriptors, right->descriptors, 16);
        num_matches = matcher.filterMatches(params.featureThreshold, params.featureRatio);
    }


    points1.clear();
    points2.clear();


    normalized_points1.clear();
    normalized_points2.clear();

    for (int i = 0; i < matcher.matches.size(); ++i)
    {
        auto m  = matcher.matches[i];
        auto p1 = left->undistorted_keypoints[m.first].point;
        auto p2 = right->undistorted_keypoints[m.second].point;

        points1.push_back(p1);
        points2.push_back(p2);

        normalized_points1.push_back(left->normalized_points[m.first]);
        normalized_points2.push_back(right->normalized_points[m.second]);
    }

    return num_matches;
}

int MonoInitializer::ComputeMatchesPrediction(FramePtr left, FramePtr right)
{
    //    SAIGA_BLOCK_TIMER();
    SAIGA_ASSERT(last_frame_feature_position.size() == left->N);

    float radius     = 50;
    int featureError = 50;

    matches.resize(left->N);

#pragma omp parallel for num_threads(settings.num_tracking_threads)
    for (int i = 0; i < left->N; ++i)
    {
        auto prev_point = last_frame_feature_position[i];
        vector<int> tmp_indices;
        right->GetFeaturesInArea(tmp_indices, prev_point, radius);

        auto& descriptor1 = left->descriptors[i];


        int bestDist = 256;
        int bestIdx2 = -1;
        for (auto i2 : tmp_indices)
        {
            auto descriptor2 = right->descriptors[i2];
            auto dist        = distance(descriptor1, descriptor2);
            if (dist < bestDist)
            {
                bestDist = dist;
                bestIdx2 = i2;
            }
        }

        if (bestDist < featureError)
        {
            last_frame_feature_position[i] = right->undistorted_keypoints[bestIdx2].point;
            matches[i]                     = {bestIdx2};
        }
        else
        {
            matches[i] = -1;
        }
    }


    points1.clear();
    points2.clear();
    normalized_points1.clear();
    normalized_points2.clear();

    matcher.matches.clear();

    for (int i = 0; i < left->N; ++i)
    {
        //        auto m  = matcher.matches[i];
        auto i1 = i;
        auto i2 = matches[i];


        if (i2 < 0) continue;

        points1.push_back(left->undistorted_keypoints[i1].point);
        points2.push_back(right->undistorted_keypoints[i2].point);

        normalized_points1.push_back(left->normalized_points[i1]);
        normalized_points2.push_back(right->normalized_points[i2]);


        matcher.matches.push_back({i1, i2});
    }


    //    std::cout << Statistics(distances) << std::endl;

    return points1.size();
}


}  // namespace Snake
