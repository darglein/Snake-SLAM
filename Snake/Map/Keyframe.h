/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once
#include "saiga/core/Core.h"
#include "saiga/vision/imu/Preintegration.h"
#include "saiga/vision/scene/Scene.h"

#include "Frame.h"
#include "KeyframeGraph.h"
#include "System/SnakeGlobal.h"

#include <set>

namespace Snake
{
class Keyframe : public KeyframeBase, public KeyframeGraph
{
   public:
    Keyframe(int id) : KeyframeBase(id), KeyframeGraph(this) { observations.reserve(maxFeatures); }

    void init(FramePtr frame, Keyframe* reference, Keyframe* previous);



    FramePtr frame;


    void UpdateConnections();

    void ComputeDepthRange(bool always = false);

    double MedianDepth() { return median_depth; }



    void EraseMapPointMatch(MapPoint* pMP);

    void EraseMapPointMatch(int idx);


    // This value changes how likely this keyframe is going to be culled.
    // == 1    default culling algorithm
    //  > 1    This kf is more likely to be culled.
    //  > 1    This kf is less liekly to be culled.
    //
    // This is used to mark helper or stabilization keyframes, which were inserted because of bad tracking.
    float cull_factor = 1.0;


#if WITH_IMU
    // Imu data and preintegration from the 'previous' keyframe to 'this' keyframe
    Imu::ImuSequence imudata;

    bool preint_valid = false;
    Imu::Preintegration preint;


    void PreintegrateFromPrevious(bool update_vb);

    RelPoseConstraint rpc;
    int imu_state = 0;


    int debug_flag = 0;
#endif
    Imu::VelocityAndBias velocity_and_bias;

    // returns chi2 + number of projections
    std::pair<double, int> reprojectionError();
    std::pair<double, int> reprojectionErrorDebug();
    bool IsValid();

    int removeOutliers(double thresholdMono, double thresholdStereo);

    MapPoint* GetMapPoint(int idx) const;

    void setMapPoint(MapPoint* mp, int idx);

    const vector<MapPoint*>& GetMapPointMatches() const;

    void GetMapPointMatches(vector<MapPoint*>& points) const;

    std::vector<MapPoint*>& MapPointRef() { return observations; }

    int MatchCount(int min_obs) const;


    void addMappoint(MapPoint* mp, int id);


    bool ReplaceMapPointMatch(int idx, MapPoint* mp);

    void SetBadFlag();

    // The reference KF which was used during fine tracking. The kf which had the most matches to this frame. Can be
    // different to previousKF
    Keyframe* referenceKF = nullptr;

    // The actual previous KF.
    // Should always have "-1" id during creation.
    Keyframe* previousKF = nullptr;
    Keyframe* nextKF     = nullptr;



    bool inMap = false;

    SE3 relToRef;  // before the keyframe is in the map, it only exists in relative space to its parent
    SE3 poseFromRef();
    SE3 PoseGlobal();


    TemplatedImage<ucvec4> createActiveFeatureImage();


    void Transform(const SE3& t, double scale);



   private:
    double median_depth = -1;

    std::vector<MapPoint*> observations;



    friend std::ostream& operator<<(std::ostream& strm, const Keyframe& frame);
};



using KeyFrame = Keyframe;

}  // namespace Snake
