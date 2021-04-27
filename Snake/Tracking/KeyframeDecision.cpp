/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#include "KeyframeDecision.h"

#include "Map/Map.h"
#include "Optimizer/LocalBundleAdjustment.h"
namespace Snake
{
TrackingQualityEvaluator::TrackingQualityEvaluator()
{
    tab = Table({5, 6, 4, 5, 20, 4, 20});
}

std::pair<TrackingQuality, const char*> TrackingQualityEvaluator::ComputeTrackingQuality(Frame& frame)
{
    auto lock = map.LockReadOnly();

    int current_matches = frame.trackingInliers;

    if (settings.inputType == InputType::Stereo)
    {
        int close_stereo = 0;
        int non_close    = 0;
        for (auto i : frame.featureRange())
        {
            if (!frame.mvpMapPoints[i]) continue;
            if (frame.mvbOutlier[i]) continue;

            //            if (frame.mvDepth[i] > 0)
            {
                if (frame.depth[i] > th_depth || frame.mvpMapPoints[i]->far_stereo_point)
                    non_close++;
                else
                    close_stereo++;
            }
        }

        if (close_stereo < 90 && non_close > 60)
        {
            return {TrackingQuality::NEED, "Low Stereo"};
        }
        current_matches = frame.trackingInliers - non_close;
    }



    int last_kf_matches = frame.referenceKF()->MatchCount(map.KeyFramesInMap() <= 2 ? 2 : 3);
    float targetRatio   = float(current_matches) / settings.kfi_target_matches;

    float target_kf_ratio = float(current_matches) / last_kf_matches;


    if (current_matches < 50)
    {
        return {TrackingQuality::SUPER_BAD, "Super Low Matches"};
    }


    if (current_matches < 60)
    {
        return {TrackingQuality::BAD, "Low Matches"};
    }

    if (targetRatio < 0.5)
    {
        return {TrackingQuality::BAD, "Low Ratio"};
    }


    if (target_kf_ratio < 0.6)
    {
        return {TrackingQuality::BAD, "Low KF Ratio"};
    }

    if (targetRatio >= 1.3)
    {
        return {TrackingQuality::VERY_GOOD, "Very High Ratio"};
    }

    if (targetRatio >= 0.8)
    {
        return {TrackingQuality::GOOD, "High Ratio"};
    }


    if (target_kf_ratio > 2.0)
    {
        return {TrackingQuality::GOOD, "HIGH KF Ratio"};
    }
    else
    {
        return {TrackingQuality::MEDIUM, "Medium Ratio"};
    }


    return {TrackingQuality::MEDIUM, "Default"};
}

std::pair<bool, const char*> TrackingQualityEvaluator::Decision(TrackingQuality quality, Frame& frame,
                                                                Keyframe* last_keyframe)
{
    auto num_frames_since_kf = frame.id - last_keyframe->frame->id;

#if WITH_IMU
    auto time_since_kf = float(num_frames_since_kf) / mono_intrinsics.fps;
    if (time_since_kf >= max_time_between_kf_tracking)
    {
        return {true, "Time"};
    }
#endif



    if (quality == TrackingQuality::NEED)
    {
        return {true, "Need"};
    }

    if ((int)quality <= (int)TrackingQuality::SUPER_BAD)
    {
        return {false, "Super Bad"};
    }

    if ((int)quality >= (int)TrackingQuality::VERY_GOOD)
    {
        return {false, "Very Good"};
    }

    double translation_angle, rotation_angle;
    {
        auto lock         = map.LockReadOnly();
        auto global_pose  = frame.getPoseFromReference().inverse();
        auto last_kf_pose = last_keyframe->PoseInv();

        // compute relative translation
        last_keyframe->ComputeDepthRange();
        double medianDepthKF2 = last_keyframe->MedianDepth();
        double baseline       = (global_pose.translation() - last_kf_pose.translation()).norm();
        translation_angle     = degrees(atan2(baseline / 2.0, medianDepthKF2));

        // compute relative rotation
        Vec3 dir1      = global_pose.unit_quaternion() * Vec3(0, 0, 1);
        Vec3 dir2      = last_kf_pose.unit_quaternion() * Vec3(0, 0, 1);
        rotation_angle = degrees(acos(dir1.dot(dir2)));
    }


    if (num_frames_since_kf > 30 && translation_angle > 0.5)
    {
        return {true, "Time"};
    }


    if ((int)quality >= (int)TrackingQuality::GOOD)
    {
        return {false, "Good"};
    }

    if (translation_angle > 1 || rotation_angle > 15)
    {
        return {true, "Good Angle"};
    }



    if ((translation_angle > 1 || rotation_angle > 10) && (int)quality <= (int)TrackingQuality::BAD)
    {
        return {true, "Self Rotation"};
    }

    if ((translation_angle > 1.5 || rotation_angle > 15) && (int)quality <= (int)TrackingQuality::SUPER_BAD)
    {
        return {true, "Bad Angle"};
    }

    return {false, "Default"};
}


std::pair<bool, double> TrackingQualityEvaluator::NeedNewKeyframe(Frame& frame, Keyframe* last_keyframe)
{
    //    return {frame.id % 15 == 0, 1.0};
    auto [quality, msg]                  = ComputeTrackingQuality(frame);
    auto [actual_decision, decision_msg] = Decision(quality, frame, last_keyframe);

    double cull_factor = 1.0;

    switch (quality)
    {
        case TrackingQuality::SUPER_BAD:
            cull_factor = 1.0;
        case TrackingQuality::BAD:
            cull_factor = 1.0;
        default:
            cull_factor = 1.0;
    }

    if (actual_decision)
    {
        //        tab << "[TQ] " << frame.id << frame.trackingInliers << (int)quality << msg << actual_decision <<
        //        decision_msg;
    }

    return {actual_decision, cull_factor};
}



}  // namespace Snake
