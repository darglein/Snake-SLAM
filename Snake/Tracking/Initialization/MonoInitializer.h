/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once
#include "saiga/core/Core.h"
#include "saiga/core/imgui/imgui_saiga.h"
#include "saiga/vision/reconstruction/Homography.h"
#include "saiga/vision/reconstruction/TwoViewReconstruction.h"
#include "saiga/vision/util/HistogramImage.h"

#include "Initializer.h"
#include "Map/Map.h"
#include "System/SnakeGlobal.h"

//#define USE_FIVE_POINT
//#include "Tracking/PoseRefinement.h"
//#include "Tracking/TrackingCoarse.h"
//#include "Tracking/TrackingFine.h"

namespace Snake
{
struct MonoInitializerParameters
{
    // Minimum number of 2-view feature matches
    // Typical Range: [200,500]
    int min_matches;

    // 5-point outlier threshold in pixels (distance to epipolar line).
    // Typical Range: [0.5,3]
    float five_point_threshold_px;

    // Minimum number of 5-point inliers
    // Typical Range: [150,400]
    int min_inliers;

    // Minimum 2-view angle
    // Typical Range: [0.5,2]
    float min_angle;

    // Maximum inliers of the homographic transformation compared to 5-point.
    // Range [0,1]
    float max_homography_ratio;

    // Feature distribution density
    // Range [0,1]
    float min_histogram_density;


    static constexpr float featureThreshold = 60;
    static constexpr float featureRatio     = 0.75;


#ifdef SNAKE_MOBILE
    static constexpr int five_point_ransac_iterations = 150;
#else
    static constexpr int five_point_ransac_iterations = 300;
#endif

    void MakeLowQuality()
    {
        min_matches             = 175;
        five_point_threshold_px = 2.0;
        min_inliers             = 140;
        min_angle               = 1.1;
        max_homography_ratio    = 0.7;
        min_histogram_density   = 0.3;
    }

    void MakeMediumQuality()
    {
        min_matches             = 200;
        five_point_threshold_px = 1.5;
        min_inliers             = 150;
        min_angle               = 1.6;
        max_homography_ratio    = 0.5;
        min_histogram_density   = 0.3;
    }

    void MakeHighQuality()
    {
        min_matches             = 250;
        five_point_threshold_px = 1.5;
        min_inliers             = 200;
        min_angle               = 2.0;
        max_homography_ratio    = 0.5;
        min_histogram_density   = 0.5;
    }
};


struct MonoInitInfo
{
    int frame1 = -1, frame2 = -1;

    int matches = -1, inliers = -1;
    int homographyInliers = -1;
    float angle           = -1;
    float histo           = -1;

    enum class State : int
    {
        SUCCESS            = 0,
        NOT_ENOUGH_MATCHES = 1,
        NOT_ENOUGH_INLIERS = 2,
        ANGLE_TOO_SMALL    = 3,
        HISTOGRAM_FAILURE  = 4,
        PLANAR_FAIL        = 5,
        UNKNOWN_ERROR      = 6,
    };

    State state = State::UNKNOWN_ERROR;
};

class SAIGA_ALIGN_CACHE MonoInitializer : public Initializer
{
   public:
    MonoInitializer(int quality);


    virtual InitializerResult Initialize(FramePtr frame) override;


   private:
    int ComputeMatchesBF(FramePtr left, FramePtr right);
    int ComputeMatchesPrediction(FramePtr left, FramePtr right);


    MonoInitInfo ComputeRelativeTransformation(FramePtr frame1, FramePtr frame2);
    InitializerResult InitializeMap(const MonoInitInfo& info, FramePtr frame1, FramePtr frame2);


    MonoInitializerParameters params;
    std::vector<FramePtr> frames;

    InitializerResult result;

    // ========= Temp Data for mono initialization =========

    int currentFirst = -1;
    void selectFirstFrame();
    FramePtr firstFrame;
    BruteForceMatcher<FeatureDescriptor> matcher;

#ifdef USE_FIVE_POINT
    TwoViewReconstruction tvr;
#else
    TwoViewReconstructionEightPoint tvr;
#endif
    HomographyRansac hr;

    static constexpr double target_scale = 3;

    AlignedVector<Vec2> h_points1, h_points2;

    AlignedVector<Vec2> points1, points2;
    AlignedVector<Vec2> normalized_points1, normalized_points2;


    AlignedVector<Vec2> last_frame_feature_position;
    std::vector<int> matches;
};



}  // namespace Snake
