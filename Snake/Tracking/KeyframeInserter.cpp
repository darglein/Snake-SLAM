/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#include "saiga/core/util/ini/ini.h"
#include "saiga/vision/util/HistogramImage.h"

#include "KeyframeDecision.h"
#include "Map/Map.h"
#include "Tracking.h"
#include "LocalMapping/LocalMapping.h"

namespace Snake
{
std::pair<bool, double> Tracking::NeedNewKeyframe(FramePtr frame)
{
    SAIGA_ASSERT(settings.kfi_target_matches > 0);
    SAIGA_ASSERT(frame);
    TrackingQualityEvaluator tq;
    auto kf_result = tq.NeedNewKeyframe(*frame, last_keyframe);
    return kf_result;
}


Keyframe* Tracking::CreateNewKeyFrame(FramePtr frame, double cull_factor)
{
    SAIGA_ASSERT(mono_intrinsics.fps != -1);
    if (mono_intrinsics.fps > 10 && last_keyframe->frame->id + 1 == frame->id)
    {
        VLOG(1) << "skipping kf (last frame already kf)";
        return nullptr;
    }

    if (frame->trackingInliers < 30)
    {
        return nullptr;
    }

    if (validInARow < 2)
    {
        return nullptr;
    }

    // compute angle between last and current kf
    double angle;
    TEST_MAP_SYNC;
    {
        auto lock            = map.LockReadOnly();
        auto global_position = frame->getPoseFromReference().inverse().translation();
        last_keyframe->ComputeDepthRange();
        double medianDepthKF2 = last_keyframe->MedianDepth();
        double baseline       = (global_position - last_keyframe->CameraPosition()).norm();
        angle                 = degrees(atan2(baseline / 2.0, medianDepthKF2));
    }
    TEST_MAP_SYNC;

#if 1
    if (angle < 0.3 && frame->trackingInliers > 20)
    {
        VLOG(1) << "skipping low angle kf: " << *frame << " " << angle << " " << frame->trackingInliers;
        return nullptr;
    }
#endif

    KeyFrame* kf;
    {
        std::unique_lock l(map.mMutexMap);
        kf = map.allocateKeyframe();
    }

    kf->init(frame, frame->referenceKF(), last_keyframe);
    frame->setReference(kf, SE3());
    frame->isKeyframe = true;
    kf->cull_factor   = cull_factor;
    last_keyframe     = kf;
    predictor.SetLastKeyframe(last_keyframe);
    return kf;
}

}  // namespace Snake
