/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once
#include "saiga/core/Core.h"

#include "Keyframe.h"
#include "MapPoint.h"
#include "System/SnakeGlobal.h"

//#define USE_LM_HASHMAP
namespace Snake
{
struct CoarseTrackingPoint
{
    MapPoint* mp;

    FeatureDescriptor descriptor;
    Vec3 position;
    Vec3 normal;
    int octave;
    float angle;


    // Descriptor, octave, and angle are directly set by coarse tracking
    CoarseTrackingPoint(MapPoint* _mp) : mp(_mp), position(_mp->getPosition()), normal(_mp->GetNormal()) {}
};

struct FineTrackingPoint
{
    MapPoint* mp;

    // Local copies of the important data so we don't have to synchronize it
    FeatureDescriptor descriptor;
    Vec3 position;
    Vec3 normal;

    float reference_depth;
    int reference_scale_level;

    bool valid = true;

    FineTrackingPoint(MapPoint* _mp)
        : mp(_mp),
          descriptor(_mp->GetDescriptor()),
          position(_mp->getPosition()),
          normal(_mp->GetNormal()),
          reference_depth(_mp->reference_depth),
          reference_scale_level(_mp->reference_scale_level)
    {
    }
};

struct FusionPoint
{
    // Local copies of the important data so we don't have to synchronize it
    FeatureDescriptor descriptor;
    Vec3 position;
    Vec3 normal;

    float reference_depth;
    int reference_scale_level;
    int observations;
    int id;

    FusionPoint(MapPoint* _mp)
        : descriptor(_mp->GetDescriptor()),
          position(_mp->getPosition()),
          normal(_mp->GetNormal()),
          reference_depth(_mp->reference_depth),
          reference_scale_level(_mp->reference_scale_level),
          observations(_mp->GetNumObservations()),
          id(_mp->id())
    {
    }
};


template <typename PointType>
class SAIGA_ALIGN_CACHE LocalMap
{
   public:
    AlignedVector<PointType> points;

    LocalMap(int reservedMaxPoints = 10000)
    {
        points.reserve(reservedMaxPoints);
        per_point_data.resize(maxPoints);
    }

    void addPoints(const std::vector<MapPoint*>& data)
    {
        SAIGA_ASSERT(globalLmId >= 0);
        //        int countBefore = points.size();
        for (auto mp : data)
        {
            if (!mp || mp->isBad()) continue;
            addPoint(mp);
        }
        //        int newCount = points.size() - countBefore;
        //        std::cout << "added " << newCount << " points " << std::endl;
    }


    // apply this transformation to the points
    void addPoints(const std::vector<MapPoint*>& data, const SE3& transform)
    {
        SAIGA_ASSERT(globalLmId >= 0);
        for (auto mp : data)
        {
            if (!mp || mp->isBad()) continue;
            addPoint(mp, transform);
        }
    }


    int localIndex(MapPoint* mp) const
    {
        auto& data = per_point_data[mp->id()];
        if (data.LmId == globalLmId)
        {
            SAIGA_ASSERT(data.LmId2 >= 0);
            return data.LmId2;
        }
        else
        {
            return -1;
        }
    }
    void clear()
    {
        globalLmId++;
        points.clear();
    }

    int addPoint(MapPoint* mp)
    {
        auto& data = per_point_data[mp->id()];
        // just check if the id stored in mappoint matches the globallmid
        if (data.LmId != globalLmId)
        {
            int id = points.size();
            points.emplace_back(mp);
            data.LmId2 = id;
            data.LmId  = globalLmId;
            return id;
        }
        //        return -1;
        // the point has already been added -> return previous id
        return data.LmId2;
    }

    int addPoint(MapPoint* mp, const SE3& transform)
    {
        auto& data = per_point_data[mp->id()];
        // just check if the id stored in mappoint matches the globallmid
        if (data.LmId != globalLmId)
        {
            int id = points.size();
            points.emplace_back(mp);
            points.back().position = transform * points.back().position;
            points.back().normal   = transform.so3() * points.back().normal;
            data.LmId2             = id;
            data.LmId              = globalLmId;
            return id;
        }
        return -1;
    }
    friend std::ostream& operator<<(std::ostream& strm, const LocalMap& lm)
    {
        strm << "[LocalMap] " << lm.points.size();
        return strm;
    }

   private:
    struct PerPointData
    {
        int LmId  = -1;
        int LmId2 = -1;
    };
    AlignedVector<PerPointData> per_point_data;
    int globalLmId = 0;
};



}  // namespace Snake
