/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */


#pragma once

#include "Map/Frame.h"
#include "Map/LocalMap.h"
#include "System/SnakeGlobal.h"
namespace Snake
{
struct StatePredictionResult
{
    bool valid = false;
    Keyframe* reference_kf;
    SE3 rel_pose_to_kf_body;
    SE3 local_velocity_body;

    SE3 pose_body;
};

class StatePredictor
{
   public:
    StatePredictor();
    StatePredictionResult Predict(FramePtr frame);


    bool PredictWithMotionModel(FramePtr frame);
    bool PredictFromKeyframeIMU(FramePtr frame);
    bool PredictFromFrameIMU(FramePtr frame);

    void SetLastKeyframe(Keyframe* kf);

    void Rescale(double scale);


    void SetVisualVelocity(Frame& frame, const SE3& velocity_body)
    {
        frame.local_velocity = velocity_body;
        motion_model_body->addRelativeMotion(velocity_body, frame.id);
    }


    Vec3 translation_motion_model = Vec3::Zero();
    SE3 predicted_pose_body;
    SE3 predicted_velocity_body;

    bool last_pose_valid;
    SE3 last_pose_body;
    SE3 last_kf_pose_body;

    Keyframe* last_keyframe = nullptr;
    FramePtr last_frame;

    Imu::Preintegration imu_preintegration_since_last_kf;
    Imu::ImuSequence imu_data_since_last_kf;


    // Motion model in body space.
    // The velocity is right-multiplied to the body_to_world transformation.
    std::shared_ptr<Saiga::MotionModel> motion_model_body;
};



}  // namespace Snake
