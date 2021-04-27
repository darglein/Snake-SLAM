/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#include "MapPoint.h"

#include "Frame.h"
#include "Keyframe.h"
#include "Map.h"

namespace Snake
{
MapPoint::MapPoint(int id) : id_(id)
{
    // Only a few points have more than 16 observations.
    mObservations.reserve(8);
}

void MapPoint::init(Keyframe& referenceKF, const Vec3& position)
{
    this->referenceKF = &referenceKF;
    this->mnFirstKFid = referenceKF.id();
    this->position    = (position);
}

void MapPoint::RelinkObservation(Keyframe* kf, int idOld, int idNew)
{
    ASSERT_HAS_WRITE_LOCK;
    SAIGA_ASSERT(isInInternal(kf));
    SAIGA_ASSERT(idOld != idNew);

    for (auto&& i : mObservations)
    {
        if (i.first == kf)
        {
            SAIGA_ASSERT(i.second == idOld);
            i.second = idNew;
        }
    }
}



void MapPoint::AddObservation(Keyframe* kf, int id)
{
    ASSERT_HAS_WRITE_LOCK;
    if (isInInternal(kf)) return;
    addInternal(kf, id);

    if (kf->frame->hasDepth(id))
    {
        numObservations += 2;
    }
    else
        numObservations++;
}

void MapPoint::ComputeDistinctiveDescriptors()
{
    ASSERT_HAS_WRITE_LOCK;
    if (!valid) return;

    vector<FeatureDescriptor> allDescriptors;
    allDescriptors.clear();
    allDescriptors.reserve(mObservations.size());

    for (auto obs : mObservations)
    {
        if (obs.first->isBad()) continue;
        allDescriptors.push_back(obs.first->frame->descriptors[obs.second]);
    }

    if (allDescriptors.empty()) return;

    Saiga::MeanMatcher<FeatureDescriptor> matcher;

    auto bestId = matcher.bestDescriptorFromArray(allDescriptors);
    descriptor  = allDescriptors[bestId];
}


void MapPoint::SetBadFlag()
{
    ASSERT_HAS_WRITE_LOCK;
    ObservationType obs;
    bool prev;
    {
        prev  = valid;
        valid = false;
        obs   = mObservations;
        mObservations.clear();
    }
    for (auto mit : obs)
    {
        KeyFrame* pKF = mit.first;
        pKF->EraseMapPointMatch(mit.second);
    }

    // the point was valid before removal
    if (prev) map.EraseMapPoint(*this);
}

void MapPoint::print()
{
    ASSERT_HAS_READ_LOCK;
    std::cout << "[MapPoint] " << id_ << std::endl;
    std::cout << position.transpose() << std::endl;
    for (auto obs : mObservations)
    {
        auto ip    = K.project(obs.first->Pose() * position);
        auto error = (obs.first->frame->undistorted_keypoints[obs.second].point - ip).norm();
        std::cout << "{" << obs.first->id() << "," << obs.second << "," << error << "} ";
    }
    std::cout << std::endl;
}


void MapPoint::UpdateNormal()
{
    ASSERT_HAS_WRITE_LOCK;
    normal.setZero();
    int n = 0;
    for (auto obs : mObservations)
    {
        if (!(*obs.first)) continue;
        auto kfposition = obs.first->CameraPosition();

        normal += (kfposition - position).normalized();
        ++n;
    }
    //    normal /= double(n);
    normal.normalize();
}

void MapPoint::UpdateNormalLocal()
{
    normal = -position;
    normal.normalize();
}

void MapPoint::UpdateDepthLocal(int id)
{
    SAIGA_ASSERT(referenceKF);
    auto& reffeatures = referenceKF->frame;

    reference_depth = position.norm();
    auto refIdx     = id;
    SAIGA_ASSERT(refIdx >= 0);
    reference_scale_level = reffeatures->undistorted_keypoints[refIdx].octave;
}

void MapPoint::UpdateDepth()
{
    ASSERT_HAS_WRITE_LOCK;
    // no reason to update that every time
    // the expected distance should be somewhat constant
    SAIGA_ASSERT(referenceKF);
    auto& reffeatures = referenceKF->frame;

    reference_depth = (position - referenceKF->CameraPosition()).norm();
    auto refIdx     = indexInternal(referenceKF);
    SAIGA_ASSERT(refIdx >= 0);
    reference_scale_level = reffeatures->undistorted_keypoints[refIdx].octave;
}

bool MapPoint::IsInKeyFrame(Keyframe* pKF) const
{
    ASSERT_HAS_READ_LOCK;
    //        UniqueLock lock(mMutexFeatures);
    //        return (mObservations.count(pKF));
    return GetIndexInKeyFrame(pKF) != -1;
}

Vec3 MapPoint::getPosition()
{
    ASSERT_HAS_READ_LOCK;
    return position;
}

Vec3 MapPoint::GetNormal()
{
    ASSERT_HAS_READ_LOCK;
    return normal;
}

void MapPoint::SetWorldPos(const Vec3& Pos)
{
    ASSERT_HAS_READ_LOCK;
    position = Pos;
}

const MapPoint::ObservationType& MapPoint::GetObservationList() const
{
    ASSERT_HAS_READ_LOCK;
    return mObservations;
}

int MapPoint::GetIndexInKeyFrame(Keyframe* pKF) const
{
    ASSERT_HAS_READ_LOCK;
    return indexInternal(pKF);
}



void MapPoint::Replace(MapPoint* mp)
{
    ASSERT_HAS_WRITE_LOCK;
    if (mp->id() == this->id()) return;
    if (!valid) return;

    int nvisible, nfound;
    //    std::map<KeyFrame*, int> obs;
    ObservationType obs;
    {
        obs = mObservations;
        mObservations.clear();
        valid    = false;
        nvisible = mnVisible;
        nfound   = mnFound;
    }

    for (auto mit = obs.begin(), mend = obs.end(); mit != mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        if (!mp->IsInKeyFrame(pKF))
        {
            if (pKF->ReplaceMapPointMatch(mit->second, mp))
            {
                mp->AddObservation(pKF, mit->second);
            }
        }
        else
        {
            pKF->EraseMapPointMatch(mit->second);
        }
    }
    mp->IncreaseFound(nfound);
    mp->IncreaseVisible(nvisible);
    mp->ComputeDistinctiveDescriptors();

    map.EraseMapPoint(*this);
}


void MapPoint::EraseObservation(Keyframe* pKF)
{
    ASSERT_HAS_WRITE_LOCK;
    bool bBad = false;
    {
        auto idx = indexInternal(pKF);
        if (idx == -1) return;

        if (pKF->frame->right_points[idx] >= 0)
            numObservations -= 2;
        else
            numObservations--;


        //            mObservations.erase(pKF);
        removeInternal(pKF);


        // If only 2 observations or less, discard point
        if (numObservations <= 2 || mObservations.empty())
        {
            bBad = true;
        }
        else if (referenceKF == pKF)
        {
            SAIGA_ASSERT(!mObservations.empty());
            referenceKF   = nullptr;
            int valid_obs = 0;
            for (auto o : mObservations)
            {
                if (!o.first->isBad())
                {
                    valid_obs++;
                    referenceKF = o.first;
                }
            }

            if (!referenceKF || valid_obs <= 2)
            {
                bBad = true;
            }
        }
    }

    if (bBad) SetBadFlag();
}

void MapPoint::Transform(const SE3& t, double scale)
{
    reference_depth *= scale;
    position = DSim3(t, scale) * position;
    normal   = t.so3() * normal;
    normal.normalize();
}


bool MapPoint::isInInternal(Keyframe* kf) const
{
    ASSERT_HAS_READ_LOCK;
    return indexInternal(kf) != -1;
}

int MapPoint::indexInternal(Keyframe* kf) const
{
    ASSERT_HAS_READ_LOCK;
    for (auto&& i : mObservations)
    {
        if (i.first == kf) return i.second;
    }
    return -1;
}

void MapPoint::addInternal(Keyframe* kf, int id)
{
    ASSERT_HAS_WRITE_LOCK;
    mObservations.emplace_back(kf, id);
}

void MapPoint::removeInternal(Keyframe* kf)
{
    ASSERT_HAS_WRITE_LOCK;
    int j = 0;
    for (auto&& i : mObservations)
    {
        if (i.first == kf) break;
        ++j;
    }
    SAIGA_ASSERT(j != (int)mObservations.size());
    mObservations[j] = mObservations.back();
    mObservations.pop_back();
}

// int MapPoint::PredictScale(const float& currentDist)
//{
//    float ratio;
//    {
//        UniqueLock lock(tsanOnlyMutex);
//        ratio = mfMaxDistance / currentDist;
//    }

//    int nScale = ceil(log(ratio) / scalePyramid.log_scale_factor);

//    if (nScale < 0)
//        nScale = 0;
//    else if (nScale >= scalePyramid.levels)
//        nScale = scalePyramid.levels - 1;

//    return nScale;
//}

}  // namespace Snake
