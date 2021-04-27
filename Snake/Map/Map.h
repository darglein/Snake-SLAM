/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once
#include "saiga/core/Core.h"
#include "saiga/core/util/DataStructures/FixedVector.h"
#include "saiga/core/util/Thread/DebugLock.h"
#include "saiga/vision/scene/Scene.h"

#include "Keyframe.h"
#include "MapPoint.h"
#include "System/SnakeGlobal.h"
#include "ViewerInterface.h"

#include <shared_mutex>



//#define CHECK_VALID_MAP SAIGA_ASSERT(map.valid())
#define CHECK_VALID_MAP

#define TEST_MAP_SYNC

#if 0
// Debug asserts to check if the map mutex was acquired correctly
#    define ASSERT_HAS_MAP_LOCK2 SAIGA_ASSERT(mutexUpdate.IsLockedByCaller())
#    define ASSERT_HAS_READ_LOCK SAIGA_ASSERT(map.mutexUpdate.IsLockedByCallerShared())
#    define ASSERT_HAS_WRITE_LOCK SAIGA_ASSERT(map.mutexUpdate.IsLockedByCaller())
#    define USE_DEBUG_LOCK
#else
#    define ASSERT_HAS_MAP_LOCK2
#    define ASSERT_HAS_READ_LOCK
#    define ASSERT_HAS_WRITE_LOCK
#endif
namespace Snake
{
class Map
{
    using Index = int;

   public:
    Map(int _maxKeyframes = maxKeyframes, int _maxPoints = maxPoints);


    Keyframe* allocateKeyframe()
    {
        SAIGA_ASSERT(countKeyframes < (int)allKeyframes.capacity());
        auto id  = countKeyframes++;
        auto& kf = allKeyframes[id];
        SAIGA_ASSERT(kf.id() == id);
        return &kf;
    }


    MapPoint& allocateMapPoint()
    {
        SAIGA_ASSERT(countPoints < (int)allPoints.capacity());
        auto id  = countPoints++;
        auto& mp = allPoints[id];
        SAIGA_ASSERT(mp.id() == id);
        return mp;
    }

    // Allocate n mappoints.
    // returns the beginning of the array and the point id of the first point.
    // You still have to initialize the points with MapPoint::init(...)!
    MapPoint* allocateMulti(int n)
    {
        SAIGA_ASSERT(countPoints + n <= (int)allPoints.capacity());
        auto startId  = countPoints;
        MapPoint* ret = &allPoints[startId];
        countPoints += n;
        return ret;
    }

    Index KeyFramesInMap()
    {
        std::unique_lock lock(mMutexMap);
        return validKeyframes;
    }

    void Clear();


    void EraseMapPoint(const MapPoint& mp)
    {
        std::unique_lock lock(mMutexMap);
        SAIGA_ASSERT(mp.valid == false);
        Index id = mp.id();
        SAIGA_ASSERT(pointValid[id] == true);
        pointValid[id] = false;
        validPoints--;
    }


    void AddMapPoint(const MapPoint& mp)
    {
        Index id = mp.id();
        SAIGA_ASSERT(pointValid[id] == false);
        pointValid[id] = true;
        validPoints++;
    }


    void EraseKeyFrame(const Keyframe& kf)
    {
        ASSERT_HAS_MAP_LOCK2;
        std::unique_lock lock(mMutexMap);
        SAIGA_ASSERT(kf.isBad() == true);
        Index id = kf.id();
        SAIGA_ASSERT(keyframeValid[id] == true);
        keyframeValid[id] = false;
        clearedKeyframes.push_back(id);
        validKeyframes--;
    }

    void AddKeyFrame(const Keyframe& kf)
    {
        std::unique_lock lock(mMutexMap);
        Index id = kf.id();
        SAIGA_ASSERT(keyframeValid[id] == false);
        keyframeValid[id] = true;
        validKeyframes++;
    }

    int MaxNumberOfKeyframes()
    {
        std::unique_lock lock(mMutexMap);
        return countKeyframes;
    }


    inline MapPoint& getMapPoint(int id) { return allPoints[id]; }
    inline Keyframe& getKeyframe(int id) { return allKeyframes[id]; }

    int mapState = 0;

    auto LockFull() const { return std::unique_lock(mutexUpdate); }
    auto LockReadOnly() const { return std::shared_lock(mutexUpdate); }

#ifdef USE_DEBUG_LOCK
    mutable Saiga::DebugLock<std::mutex> mMutexMap;
    mutable Saiga::DebugLock<std::shared_mutex> mutexUpdate;
#else
    mutable std::shared_mutex mutexUpdate;
    mutable std::mutex mMutexMap;
#endif
   public:
    void Transform(const SE3& t, double scale = 1.0);
    SE3 crazyMove();
    void RemoveRandomPoint();
    void RemoveRandomKeyframe();
    void RemoveRandomObservation();

    // ==== Stats ====
    void printStatistics(std::ostream& strm, bool align = true, bool rescale = true);

    void printTrajectory(bool rescale = true);

    struct ReprojectionError
    {
        double chi2                = 0;
        double rms                 = 0;
        int num_inserted_keyframes = 0;
        int num_keyframes          = 0;
        int num_inserted_points    = 0;
        int num_points             = 0;
        int num_observations       = 0;
    };
    ReprojectionError ReprojectionStats();


    struct TrajError
    {
        double ate_rmse    = 0;
        double ate_mean    = 0;
        double scale_error = 1;
    };

    TrajError TrajectoryError(bool solve_scale);


    // Check if keyframe observations are consistent to mappoint observations.
    // This function is very expensive and should only be called during debug sessions.
    //
    // Returns true if everything is ok.
    bool valid();



    // updated each time a map rescale is performed.
    double total_scale = 1;

   private:
    int countKeyframes = 0;
    int countPoints    = 0;
    int validKeyframes = 0;
    int validPoints    = 0;


    std::vector<Index> clearedKeyframes;


    // some flags to see if given points/keyframes are valid
    // this would be not required but increases cache efficiency a bit :)
    std::vector<bool> pointValid, keyframeValid;

    //    std::vector<Keyframe> allKeyframes;
    //    std::vector<MapPoint> allPoints;
    Saiga::FixedVectorHeap<Keyframe> allKeyframes;
    Saiga::FixedVectorHeap<MapPoint> allPoints;

   public:
    void GetAllKeyFrames(vector<int>& keyframes)
    {
        keyframes.clear();
        int i = 0;
        std::unique_lock lock(mMutexMap);
        for (auto b : keyframeValid)
        {
            if (b) keyframes.push_back(i);
            ++i;
        }
    }

    std::vector<Keyframe*> GetAllKeyFrames()
    {
        std::vector<int> kfids;
        GetAllKeyFrames(kfids);
        std::vector<Keyframe*> result;
        for (auto id : kfids)
        {
            result.push_back(&getKeyframe(id));
        }
        return result;
    }

    std::vector<FramePtr> GetReallyAllFrames()
    {
        std::vector<FramePtr> snake_frames;
        auto c = getKeyframe(MaxNumberOfKeyframes() - 1).frame;
        for (; c; c = c->previousFrame)
        {
            snake_frames.push_back(c);
        }
        std::reverse(snake_frames.begin(), snake_frames.end());
        return snake_frames;
    }

    std::vector<FramePtr> GetAllFrames()
    {
        std::vector<FramePtr> snake_frames;
        auto c = getKeyframe(MaxNumberOfKeyframes() - 1).frame;
        for (; c; c = c->previousFrame)
        {
            c->UpdateReference();
            if (!c->validPose) continue;
            c->tmpPose = c->getPoseFromReference();
            snake_frames.push_back(c);
        }
        std::reverse(snake_frames.begin(), snake_frames.end());
        return snake_frames;
    }

    void GetAllMapPoints(vector<int>& points)
    {
        points.clear();
        int i = 0;
        std::unique_lock lock(mMutexMap);
        for (auto b : pointValid)
        {
            if (b) points.push_back(i);
            ++i;
        }
    }


    void writeKeyframesToFile(const std::string& dir = "debug/keyframes");

    void imgui();

    // This method computes the 'tmpPose' os all
    //    void UpdateIntermidieateFramePoses();
    int removeOutliers(double thresholdMono, double thresholdStereo);
};


// Just use a global map object here, because
// it is used in so many different places
inline Map map;


}  // namespace Snake
