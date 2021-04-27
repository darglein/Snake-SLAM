/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#include "Input.h"

#include "saiga/core/image/ImageDraw.h"
#include "saiga/core/imgui/imgui.h"
#include "saiga/vision/opencv/opencv.h"
#include "saiga/vision/camera/all.h"

#include "Preprocess/StereoTransforms.h"
#include "opencv2/imgproc.hpp"

namespace Snake
{
Input::Input() : Module(ModuleType::INPUT)
{
    if (settings.inputType == InputType::Mono)
    {
        settings.datasetParameters.force_monocular = true;
    }
    else
    {
        settings.datasetParameters.force_monocular = false;
    }

    CreateCamera();

    switch (settings.inputType)
    {
        case InputType::Mono:
            SAIGA_ASSERT(mono_intrinsics.model.K.fx != 1);
            stereo_cam = mono_intrinsics.dummyStereoCamera();
            break;
        case InputType::Stereo:
            SAIGA_ASSERT(stereo_intrinsics.model.K.fx != 1);
            mono_intrinsics = stereo_intrinsics;

            stereo_cam = stereo_intrinsics.stereoCamera();
            th_depth   = stereo_intrinsics.maxDepth;
            break;
        case InputType::RGBD:
            SAIGA_ASSERT(rgbd_intrinsics.model.K.fx != 1);
            mono_intrinsics = rgbd_intrinsics;
            stereo_cam      = rgbd_intrinsics.stereoCamera();
            th_depth        = rgbd_intrinsics.maxDepth;
            break;
    }

    if (camera_mono) globalCamera = camera_mono.get();
    if (camera_stereo) globalCamera = camera_stereo.get();
    if (camera_rgbd) globalCamera = camera_rgbd.get();


    if (settings.inputType == InputType::Stereo)
    {
        // We need rectification if distortion is not 0
        bool need_rectify = stereo_intrinsics.model.dis.Coeffs().squaredNorm() != 0;

        if (need_rectify)
        {
            Rectify();
            stereo_intrinsics.bf      = rect_right.bf;
            stereo_cam.bf             = rect_right.bf;
            mono_intrinsics.model.K   = rect_left.K_dst;
            mono_intrinsics.model.dis = Distortion();
            stereo_cam                = StereoCamera4(rect_left.K_dst, stereo_intrinsics.bf);
        }
        else
        {
            rect_left.Identity(stereo_intrinsics.model.K, stereo_intrinsics.bf);
            rect_right.Identity(stereo_intrinsics.rightModel.K, stereo_intrinsics.bf);
            stereo_cam = stereo_intrinsics.stereoCamera();
        }

        SAIGA_ASSERT(stereo_intrinsics.bf != 0);
        SAIGA_ASSERT(stereo_cam.bf != 0);
    }
    else
    {
        rect_left.K_dst = K;
        rect_left.K_src = K;
        rect_left.D_src = mono_intrinsics.model.dis;
    }

    std::cout << mono_intrinsics << std::endl;



    featureGridBounds.computeFromIntrinsicsDist(mono_intrinsics.imageSize.w, mono_intrinsics.imageSize.h,
                                                rect_left.K_dst, rect_left.D_src);

    CreateTable({7, 10, 10}, {"Frame", "Time (s)", "Load (ms)"});
    SAIGA_ASSERT(output_table);
}

void Input::CreateCamera()
{
    switch (settings.sensorType)
    {
        case SensorType::PRIMESENSE:
        {
#ifdef SAIGA_USE_OPENNI2
            VLOG(0) << "Input set to Asus Primesense.";

            SAIGA_ASSERT(settings.inputType == InputType::RGBD);
            rgbd_intrinsics.fromConfigFile(settings.config_file);
            auto c          = std::make_unique<Saiga::RGBDCameraOpenni>(rgbd_intrinsics);
            rgbd_intrinsics = c->intrinsics();
            camera_rgbd     = std::move(c);
            break;
#else
            SAIGA_EXIT_ERROR("Openni not found.");
            break;
#endif
        }
        case SensorType::SAIGA_RAW:
        {
            SAIGA_ASSERT(settings.inputType == InputType::Mono || settings.inputType == InputType::RGBD);
            auto c = std::make_unique<Saiga::SaigaDataset>(settings.datasetParameters, true);
            if (settings.inputType == InputType::RGBD)
                rgbd_intrinsics = c->intrinsics();
            else
                mono_intrinsics = c->intrinsics();
            has_imu     = true;
            imu         = c->getIMU().value();
            camera_rgbd = std::move(c);
            break;
        }
        case SensorType::TUM_RGBD:
        {
            SAIGA_ASSERT(settings.inputType == InputType::Mono || settings.inputType == InputType::RGBD);
            auto c = std::make_unique<Saiga::TumRGBDDataset>(settings.datasetParameters);
            if (settings.inputType == InputType::RGBD)
                rgbd_intrinsics = c->intrinsics();
            else
                mono_intrinsics = c->intrinsics();
            camera_rgbd = std::move(c);
            break;
        }
        case SensorType::SCANNET:
        {
            VLOG(0) << "Input set to Scannet dataset.";

            SAIGA_ASSERT(settings.inputType == InputType::Mono || settings.inputType == InputType::RGBD);
            auto c = std::make_unique<Saiga::ScannetDataset>(settings.datasetParameters);
            if (settings.inputType == InputType::RGBD)
                rgbd_intrinsics = c->intrinsics();
            else
                mono_intrinsics = c->intrinsics();
            camera_rgbd = std::move(c);
            break;
        }
        case SensorType::ZJU:
        {
            SAIGA_ASSERT(settings.inputType == InputType::Mono);
            auto c = std::make_unique<Saiga::ZJUDataset>(settings.datasetParameters);
            c->saveGroundTruthTrajectory(settings.evalDir + settings.out_file_prefix + "_gt.tum");
            c->saveGroundTruthTrajectory(settings.evalDir + "/frames/" + settings.out_file_prefix + "_gt.tum");
            mono_intrinsics = c->intrinsics;
            has_imu         = true;
            imu             = c->getIMU().value();
            camera_mono     = std::move(c);
            break;
        }
        case SensorType::EUROC:
        {
            SAIGA_ASSERT(settings.inputType == InputType::Mono || settings.inputType == InputType::Stereo);
            auto c = std::make_unique<Saiga::EuRoCDataset>(settings.datasetParameters);
            c->saveGroundTruthTrajectory(settings.evalDir + settings.out_file_prefix + "_gt.tum");
            c->saveGroundTruthTrajectory(settings.evalDir + "/frames/" + settings.out_file_prefix + "_gt.tum");

            if (settings.inputType == InputType::Stereo)
                stereo_intrinsics = c->intrinsics;
            else
                mono_intrinsics = c->intrinsics;
            SAIGA_ASSERT(c->getIMU().has_value());
            has_imu = true;
            imu     = c->getIMU().value();

            camera_stereo = std::move(c);
            break;
        }
        case SensorType::KITTI:
        {
            SAIGA_ASSERT(settings.inputType == InputType::Mono || settings.inputType == InputType::Stereo);
            auto c = std::make_unique<Saiga::KittiDataset>(settings.datasetParameters);
            c->saveGroundTruthTrajectory(settings.evalDir + settings.out_file_prefix + "_gt.tum");
            c->saveGroundTruthTrajectory(settings.evalDir + "/frames/" + settings.out_file_prefix + "_gt.tum");
            if (settings.inputType == InputType::Stereo)
                stereo_intrinsics = c->intrinsics;
            else
                mono_intrinsics = c->intrinsics;
            camera_stereo = std::move(c);
            break;
        }
#ifdef SAIGA_USE_K4A
        case SensorType::KINECT_AZURE:
        {
            SAIGA_ASSERT(settings.inputType == InputType::Mono || settings.inputType == InputType::RGBD);
            KinectCamera::KinectParams k_params;
            k_params.color = false;
            auto c         = std::make_unique<Saiga::KinectCamera>(k_params);
            if (settings.inputType == InputType::RGBD)
                rgbd_intrinsics = c->intrinsics();
            else
                mono_intrinsics = c->intrinsics();
            has_imu     = true;
            imu         = c->getIMU().value();
            camera_rgbd = std::move(c);
            break;
        }
#endif
        default:
            SAIGA_EXIT_ERROR("Invalid Sensor ID");
    }

    if (has_imu)
    {
        SAIGA_ASSERT(imu.frequency > 0);
        SAIGA_ASSERT(imu.frequency_sqrt > 0);
        std::cout << imu << std::endl;
    }
    //    exit(0);
}


void Input::run()
{
    running       = true;
    camera_thread = ScopedThread([this]() {
        Saiga::Random::setSeed(settings.randomSeed);
        Saiga::setThreadName("CameraInput");


        std::shared_ptr<Frame> last_frame;
        auto dataset_rgbd   = dynamic_cast<DatasetCameraBase*>(camera_rgbd.get());
        auto dataset_mono   = dynamic_cast<DatasetCameraBase*>(camera_mono.get());
        auto dataset_stereo = dynamic_cast<DatasetCameraBase*>(camera_stereo.get());

        if (dataset_rgbd) dataset_rgbd->ResetTime();
        if (dataset_mono) dataset_mono->ResetTime();
        if (dataset_stereo) dataset_stereo->ResetTime();


        while (running)
        {
            std::shared_ptr<Frame> frame;
            {
                auto timer = ModuleTimer();
                frame      = ReadNextFrame();
            }


            if (!frame || stop_camera)
            {
                std::cout << "Camera Disconnected." << std::endl;
                running = false;
                break;
            }

#if 0
            // Drop random frames
            if ((frame->id > 500 && Random::sampleBool(0.1)) || frame->id == 200)
            {
                frame->image.makeZero();
                frame->image_rgb.makeZero();
            }
#endif

            if (store_previous_frame)
            {
                frame->previousFrame = last_frame;
            }
            last_frame = frame;


            if (first_image)
            {
                start_timestamp = frame->timeStamp;
                first_image     = false;
            }

            (*output_table) << frame->id << (frame->timeStamp - start_timestamp) << LastTime();

            camera_slot.set(frame);

            while (pause && running)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        camera_slot.set(nullptr);
    });

    process_image_thread = ScopedThread([this]() {
        Saiga::Random::setSeed(settings.randomSeed);
        Saiga::setThreadName("Grayscale");
        while (true)
        {
            auto frame = camera_slot.get();

            if (!frame)
            {
                break;
            }

            float gray_time;
            {
                Saiga::ScopedTimer timer(gray_time);
                computeGrayscaleImage(*frame);
            }
            output_buffer.add(frame);
        }
        output_buffer.add(nullptr);
    });
}

FramePtr Input::ReadNextFrame()
{
    FramePtr frame = Saiga::make_aligned_shared<Frame>(frameId++);
    if (camera_rgbd)
    {
        FrameData img;
        auto gotImage = camera_rgbd->getImageSync(img);
        if (!gotImage)
        {
            return nullptr;
        }

        frame->imu_data = img.imu_data;

        frame->timeStamp   = img.timeStamp;
        frame->groundTruth = img.groundTruth;
        frame->id          = img.id;
        frame->image_rgb   = std::move(img.image_rgb);
        frame->image       = std::move(img.image);
        frame->depth_image = std::move(img.depth_image);
    }
    else if (camera_mono)
    {
        FrameData img;
        auto gotImage = camera_mono->getImageSync(img);
        if (!gotImage)
        {
            return nullptr;
        }
        frame->imu_data = img.imu_data;

        frame->timeStamp   = img.timeStamp;
        frame->groundTruth = img.groundTruth;
        frame->id          = img.id;
        frame->image_rgb   = std::move(img.image_rgb);
        frame->image       = std::move(img.image);
    }
    else if (camera_stereo)
    {
        FrameData img;

        auto gotImage = camera_stereo->getImageSync(img);
        if (!gotImage)
        {
            return nullptr;
        }
        frame->imu_data = img.imu_data;

        frame->timeStamp   = img.timeStamp;
        frame->groundTruth = img.groundTruth;
        frame->id          = img.id;
        frame->image_rgb   = std::move(img.image_rgb);
        frame->image       = std::move(img.image);

        frame->right_image     = std::move(img.right_image);
        frame->right_image_rgb = std::move(img.right_image_rgb);
    }

    //    std::cout << std::setprecision(20) << frame->timeStamp << " " << frame->imu_frame.time_end << " "
    //              << frame->imu_frame.data.back().timestamp << std::endl;

    //    SAIGA_ASSERT(frame->timeStamp == frame->imu_frame.time_end);
    //    SAIGA_ASSERT(frame->timeStamp == frame->imu_frame.data.back().timestamp);
    return frame;
}

void Input::computeGrayscaleImage(Frame& frame)
{
    if (frame.image_rgb.rows == 0 || frame.image.rows > 0)
    {
        SAIGA_ASSERT(frame.image.valid());
        return;
    }

    frame.image.create(frame.image_rgb.rows, frame.image_rgb.cols);
#if 1
    cv::setNumThreads(0);
    cv::Mat4b cv_src = Saiga::ImageViewToMat(frame.image_rgb.getImageView());
    cv::Mat1b cv_dst = Saiga::ImageViewToMat(frame.image.getImageView());
    cv::cvtColor(cv_src, cv_dst, cv::COLOR_RGBA2GRAY);
#else
    ImageTransformation::RGBAToGray8(frame.image_rgb, frame.image);
#endif
}



}  // namespace Snake
