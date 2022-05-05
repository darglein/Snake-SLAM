/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#include "StereoTransforms.h"

#include "saiga/vision/opencv/opencv.h"

#include "opencv2/calib3d.hpp"
#include "opencv2/opencv.hpp"

#include <opencv2/core/eigen.hpp>


namespace Snake
{
void Rectify()
{
    std::cout << "Computing rectification" << std::endl;
    cv::Mat K1, K2;
    cv::Mat dist1, dist2;

    cv::eigen2cv(stereo_intrinsics.model.K.matrix(), K1);
    cv::eigen2cv(stereo_intrinsics.model.dis.OpenCVOrder(), dist1);

    cv::eigen2cv(stereo_intrinsics.rightModel.K.matrix(), K2);
    cv::eigen2cv(stereo_intrinsics.rightModel.dis.OpenCVOrder(), dist2);

    cv::Size size;
    size.width  = stereo_intrinsics.imageSize.width;
    size.height = stereo_intrinsics.imageSize.height;


    cv::Mat R, T;

    auto rel = stereo_intrinsics.left_to_right;
    cv::eigen2cv(rel.unit_quaternion().matrix(), R);
    cv::eigen2cv(rel.translation(), T);

    cv::Mat1d R1, R2, P1, P2, Q;
    cv::stereoRectify(K1, dist1, K2, dist2, size, R, T, R1, R2, P1, P2, Q);


    Mat3 K1_new, K2_new;
    cv::cv2eigen(P1.rowRange(0, 3).colRange(0, 3), K1_new);
    cv::cv2eigen(P2.rowRange(0, 3).colRange(0, 3), K2_new);

    Mat3 R1_eig, R2_eig;
    cv::cv2eigen(R1, R1_eig);
    cv::cv2eigen(R2, R2_eig);

    auto bf = std::abs(P2(0, 3));


    // build my own rectification
    rect_left.K_src = stereo_intrinsics.model.K;
    rect_left.D_src = stereo_intrinsics.model.dis;
    rect_left.K_dst = IntrinsicsPinholed(K1_new);
    rect_left.R     = Quat(R1_eig).normalized();
    rect_left.bf    = bf;

    rect_right.K_src = stereo_intrinsics.rightModel.K;
    rect_right.D_src = stereo_intrinsics.rightModel.dis;
    rect_right.K_dst = IntrinsicsPinholed (K2_new);
    rect_right.R     = Quat(R2_eig).normalized();
    rect_right.bf    = bf;


    // Code for creating the undistorted images.

    //    auto left_view  = ...;
    //    auto right_view = ...;

    //    TemplatedImage<unsigned char> left_test(right_view.h, right_view.w);
    //    TemplatedImage<unsigned char> right_test(right_view.h, right_view.w);

    //    for (auto i : right_test.rowRange())
    //    {
    //        for (auto j : right_test.colRange())
    //        {
    //            // transform from unrectified -> rectified
    //            auto result_left = rect_left.Backward(Vec2(j, i));
    //            left_test(i, j)  = left_view.inter(result_left(1), result_left(0));

    //            auto result_right = rect_right.Backward(Vec2(j, i));
    //            right_test(i, j)  = right_view.inter(result_right(1), result_right(0));
    //        }
    //    }
    //    left_test.save("rect_left2.png");
    //    right_test.save("rect_right2.png");
}

}  // namespace Snake
