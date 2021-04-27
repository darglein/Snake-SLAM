/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#include "StatePredictor.h"

#include "Map/Map.h"
namespace Snake
{
StatePredictor::StatePredictor()
{
    Saiga::MotionModel::Settings mmparams;
    motion_model_body = Saiga::make_aligned_shared<Saiga::MotionModel>(mmparams);
}

StatePredictionResult StatePredictor::Predict(FramePtr frame)
{
    StatePredictionResult result;

    if (!last_keyframe)
    {
        last_frame = frame;
        return result;
    }

    if (last_keyframe->isBad())
    {
        // This currently can only happen after a map clear.
        last_keyframe = nullptr;
        return result;
    }



    if (has_imu)
    {
        imu_data_since_last_kf.Add(frame->imu_data);
        imu_preintegration_since_last_kf.IntegrateForward(frame->imu_data, true);
    }

    {
        auto lock       = map.LockReadOnly();
        last_pose_valid = last_frame->validPose;
        if (last_pose_valid)
        {
            last_pose_body = last_frame->getPoseFromReference().inverse() * mono_intrinsics.camera_to_body;
        }
        last_kf_pose_body = last_keyframe->PoseInv() * mono_intrinsics.camera_to_body;
    }

    //    std::cout << "bias " << imu_preintegration_since_last_kf.GetBiasGyro().transpose() << " "
    //              << imu_preintegration_since_last_kf.GetBiasGyro().transpose() << std::endl;
    //    std::cout << "pred  mm: " << (predicted_pose_body * mono_intrinsics.camera_to_body.inverse()).inverse()
    //              << std::endl;

    result.valid |= PredictWithMotionModel(frame);
    result.valid |= PredictFromKeyframeIMU(frame);

    result.pose_body = predicted_pose_body;



    // Use the IMU prediction as a constraint
    frame->prediction = (predicted_pose_body * mono_intrinsics.camera_to_body.inverse()).inverse();

#if 1
    Vec3 inter_t = acc_position_interpolation * predicted_pose_body.translation() +
                   (1 - acc_position_interpolation) * translation_motion_model;
    SE3 interpolated_pose_body(predicted_pose_body.so3(), inter_t);
    //, but set current pose to the interpolated version for feature matching
    frame->tmpPose = (interpolated_pose_body * mono_intrinsics.camera_to_body.inverse()).inverse();

#else
    frame->tmpPose = frame->prediction;
#endif



    SE3 last_pose_camera  = (last_pose_body * mono_intrinsics.camera_to_body.inverse()).inverse();
    frame->local_velocity = frame->tmpPose * last_pose_camera.inverse();
    //    std::cout << "pred  kf: " << (predicted_pose_body * mono_intrinsics.camera_to_body.inverse()).inverse()
    //              << std::endl;

    // PredictFromFrameIMU(frame);
    //    std::cout << "pred  fr: " << (predicted_pose_body * mono_intrinsics.camera_to_body.inverse()).inverse()
    //              << std::endl;

#if 0
    std::cout << std::endl;
    std::cout << *frame << std::endl;
    std::cout << "valid   " << last_pose_valid << std::endl;
    std::cout << "Predict " << frame->tmpPose << std::endl;
    std::cout << "last    " << last_pose_camera << std::endl;
    std::cout << "kf      " << last_keyframe->Pose() << std::endl;
    std::cout << "mm      " << translation_motion_model.transpose() << std::endl;
#endif

    last_frame = frame;
    return result;
}

bool StatePredictor::PredictWithMotionModel(FramePtr frame)
{
    if (!last_pose_valid)
    {
        return false;
    }

    auto opt_velocity_body = motion_model_body->predictVelocityForFrame(frame->id);

    if (!opt_velocity_body)
    {
        predicted_pose_body      = last_pose_body;
        translation_motion_model = predicted_pose_body.translation();
        predicted_velocity_body  = SE3();
        return true;
    }

    // Compute body pose of last frame and apply velocity
    predicted_velocity_body  = opt_velocity_body.value();
    predicted_pose_body      = last_pose_body * predicted_velocity_body;
    translation_motion_model = predicted_pose_body.translation();

    frame->prediction_weight_rotation    = 0;
    frame->prediction_weight_translation = 0;


    return true;
}

bool StatePredictor::PredictFromKeyframeIMU(FramePtr frame)
{
    if (!has_imu || current_gyro_weight == 0)
    {
        return false;
    }

    SAIGA_ASSERT(imu_data_since_last_kf.time_begin == last_keyframe->frame->timeStamp);
    SAIGA_ASSERT(imu_data_since_last_kf.time_end == frame->timeStamp);
    SAIGA_ASSERT(last_keyframe->velocity_and_bias.gyro_bias.norm() > 0);

    auto pose_velocity = imu_preintegration_since_last_kf.Predict(
        last_kf_pose_body, last_keyframe->velocity_and_bias.velocity, imu_gravity.Get());

    predicted_pose_body = pose_velocity.first;


    frame->prediction_weight_rotation    = settings.weight_gyro_tracking * current_gyro_weight / 1;
    frame->prediction_weight_translation = settings.weight_acc_tracking * current_acc_weight / 1;

    if (frame->prediction_weight_translation == 0)
    {
        // Only use integrated orientation (no translation)
        // -> Set translation from motion model
        predicted_pose_body               = pose_velocity.first;
        predicted_pose_body.translation() = translation_motion_model;
    }
    else
    {
        predicted_pose_body = pose_velocity.first;
    }

    return true;
}

bool StatePredictor::PredictFromFrameIMU(FramePtr frame)
{
    if (!has_imu || current_gyro_weight == 0)
    {
        return false;
    }

    SAIGA_ASSERT(frame->imu_data.time_begin == last_frame->timeStamp);
    SAIGA_ASSERT(frame->imu_data.time_end == frame->timeStamp);

    // Preintegrate from last frame.
    Imu::Preintegration preint(last_keyframe->velocity_and_bias);
    preint.IntegrateMidPoint(frame->imu_data, false);


    auto pose_velocity = preint.Predict(last_pose_body, Vec3(0, 0, 0), Vec3(0, 0, 0));

    predicted_pose_body               = pose_velocity.first;
    predicted_pose_body.translation() = translation_motion_model;

    frame->prediction_weight_rotation    = settings.weight_gyro_tracking * current_gyro_weight / 20;
    frame->prediction_weight_translation = 0;

    return true;
}

void StatePredictor::SetLastKeyframe(Keyframe* kf)
{
    if (current_gyro_weight > 0)
    {
        SAIGA_ASSERT(kf->velocity_and_bias.gyro_bias.norm() > 0);
    }

    imu_data_since_last_kf           = Imu::ImuSequence();
    imu_preintegration_since_last_kf = Imu::Preintegration(kf->velocity_and_bias);
    last_keyframe                    = kf;
}

void StatePredictor::Rescale(double scale)
{
    std::cout << "[StatePredictor] Rescale: " << scale << std::endl;
    if (last_frame)
    {
        auto rel = last_frame->getRel().inverse();
        rel.translation() *= scale;
        last_frame->setRelToRef(rel.inverse());
    }
    motion_model_body->ScaleLinearVelocity(scale);
}

}  // namespace Snake
