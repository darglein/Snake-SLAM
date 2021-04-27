/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once

#include "saiga/core/Core.h"
#include "saiga/vision/slam/FrameBase.h"

#include "LocalMapping/Bow.h"
#include "Map/Features.h"
#include "System/SnakeGlobal.h"

namespace Snake
{
class Frame : public Features, public FrameData
{
   public:
    Frame(int _id);
    ~Frame();

    void allocateTmp();


    void clearMatches()
    {
        for (int i = 0; i < N; ++i)
        {
            mvpMapPoints[i] = nullptr;
            mvbOutlier[i]   = false;
        }
    }



    // Bag of Words Vector structures.
    BowVector bow_vec;
    FeatureVector bow_feature_vec;
    void computeBoW();


    std::vector<MapPoint*> mvpMapPoints;

    std::vector<int> mvbOutlier;

    bool validPose = false;


    Keyframe* referenceKF() { return mpReferenceKF; }

    void setReference(Keyframe* kf, const SE3& p);

    void setReference(Keyframe* kf);
    void setRelToRef(const SE3& p) { reltoRef = p; }

    void UpdateRelFromTmpPose();

    SE3 getPoseFromReference();

    void UpdateReference();

    SE3 getRel() { return reltoRef; }


    // Velocity in the local frame coordinate system.
    double prediction_weight_rotation    = 0;
    double prediction_weight_translation = 0;
    SE3 prediction;
    SE3 local_velocity;
    SE3 tmpPose;
    bool isKeyframe = false;


    // Number of inlier matches after tracking.
    // This is set first by coarse tracking and then overwritten by fine tracking.
    // It is not updated after the frame was converted to a keyframe.
    int trackingInliers = 0;

    // A reference to the last frame. Set by Input, direclty after calling the constructor.
    std::shared_ptr<Frame> previousFrame;


    void Rescale(double scale)
    {
        SE3 rel = reltoRef.inverse();
        rel.translation() *= scale;
        reltoRef = rel.inverse();
    }

    int removeOutliers(double thresholdMono, double thresholdStereo);

    SE3 reltoRef;
    Keyframe* mpReferenceKF = nullptr;
    SE3 Pose() const { return tmpPose; }
    Vec3 CameraPosition() const { return tmpPose.inverse().translation(); }
    void setPose(const SE3& pose) { tmpPose = pose; }

    friend std::ostream& operator<<(std::ostream& strm, const Frame& frame);
};



}  // namespace Snake
