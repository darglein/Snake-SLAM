/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once
#include "saiga/core/Core.h"
#include "saiga/core/util/DataStructures/FixedVector.h"

#include "System/SnakeGlobal.h"

namespace Snake
{
class Keyframe;

class MapPoint
{
   public:
    using ObservationType = std::vector<std::pair<Keyframe*, int>>;

    MapPoint(int id);
    void init(Keyframe& referenceKF, const Vec3& position);

    void RelinkObservation(Keyframe* kf, int idOld, int idNew);

    // only local mapping and loop closing use this function
    void AddObservation(Keyframe* kf, int id);

    /**
     * Compute the descriptor with the smallest distance to all other descriptor
     */
    void ComputeDistinctiveDescriptors();

    /**
     * Compute Normal by averaging the observing direction.
     * Compute min/max observation distance based on the keypoint scale and depth
     */
    void UpdateNormal();
    void UpdateNormalLocal();
    void UpdateDepthLocal(int id);
    void UpdateDepth();
    void ComputeMinMaxDepth(float dis);


    // Distance to the reference keyframe.
    float ReferenceDepth();

    explicit operator bool() { return valid; }
    bool isBad() { return !valid; }


    bool IsInKeyFrame(Keyframe* pKF) const;

    Vec3 getPosition();

    Vec3 GetNormal();


    void SetWorldPos(const Vec3& Pos);

    const FeatureDescriptor& GetDescriptor() const { return descriptor; }
    int GetNumObservations() { return numObservations; }
    const ObservationType& GetObservationList() const;
    int GetIndexInKeyFrame(Keyframe* pKF) const;


    int id() const { return id_; }

    Keyframe* referenceKF;
    bool constant         = false;
    bool loop_transformed = false;

    int mnFirstKFid       = -1;
    bool far_stereo_point = false;


    void SetBadFlag();

    void print();

    // frameid of the last frame that observed this point
    int lastFrameSeen = -1;
    int mnVisible     = 1;
    int mnFound       = 1;
    void IncreaseVisible(int n = 1) { mnVisible += n; }
    float GetFoundRatio() { return static_cast<float>(mnFound) / std::min<int>(mnVisible, 15); }
    void IncreaseFound(int n = 1) { mnFound += n; }

   public:
    // distance to the refrence kf
    float reference_depth = -1;

    // scale level of the corresponding keypoint in the reference kf
    int reference_scale_level = -1;


    // Mean viewing direction
    Vec3 normal;
    Vec3 position;


    bool valid = true;

    // Erase 'this' point and replace it by pmp
    void Replace(MapPoint* pMP);
    void EraseObservation(Keyframe* pKF);

    FeatureDescriptor descriptor;

    void Transform(const DSim3& t) { Transform(t.se3(), t.scale()); }
    void Transform(const SE3& t, double scale);


   private:
    int id_;

    ObservationType mObservations;
    int numObservations = 0;

    inline bool isInInternal(Keyframe* kf) const;
    inline int indexInternal(Keyframe* kf) const;

    inline void addInternal(Keyframe* kf, int id);

    inline void removeInternal(Keyframe* kf);
};



}  // namespace Snake
