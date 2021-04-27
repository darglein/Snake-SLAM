/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */
#include "Keyframe.h"

#include "saiga/core/image/ImageDraw.h"
#include "saiga/vision/util/HistogramImage.h"

#include "LoopClosing/KeyframeDatabase.h"
#include "Map.h"
#include "MapPoint.h"

namespace Snake
{
void Keyframe::init(FramePtr frame, Keyframe* reference, Keyframe* previous)
{
    SAIGA_ASSERT(frame);
    frame->computeBoW();
    this->frame       = frame;
    observations      = frame->mvpMapPoints;
    frame->isKeyframe = true;

    relToRef    = frame->getRel();
    referenceKF = frame->referenceKF();
    SAIGA_ASSERT(reference == frame->referenceKF());
    previousKF = previous;
    SAIGA_ASSERT(!previousKF || previousKF->id() == id() - 1);


    if (!previous)
    {
        return;
    }

    previous->nextKF = this;

    rpc.img1 = previous->id();
    rpc.img2 = id();

#if WITH_IMU
    // including the current frame
    std::vector<FramePtr> frames_since_last_kf;
    while (frame != nullptr && frame != previous->frame)
    {
        frames_since_last_kf.push_back(frame);
        frame = frame->previousFrame;
    }

    velocity_and_bias = previous->velocity_and_bias;
    imudata           = frames_since_last_kf.back()->imu_data;

    // reverse iterate through frames and insert imu data into current vector
    for (int i = (int)frames_since_last_kf.size() - 2; i >= 0; --i)
    {
        auto f = frames_since_last_kf[i];
        imudata.Add(f->imu_data);
    }

//    frame->previousFrame = nullptr;
//    std::cout << "KF " << id() << std::endl;
//    std::cout << imudata << std::endl;
#endif
}


int Keyframe::MatchCount(int min_obs) const
{
    ASSERT_HAS_READ_LOCK;
    int c = 0;
    for (auto mp : observations)
    {
        if (mp && (!mp->isBad()) && mp->GetNumObservations() >= min_obs) ++c;
    }
    return c;
}

void Keyframe::addMappoint(MapPoint* mp, int id)
{
    ASSERT_HAS_WRITE_LOCK;
    SAIGA_ASSERT(mp);
    SAIGA_ASSERT(id >= 0);
    SAIGA_ASSERT(id < (int)observations.size());
    observations[id] = mp;
}


void Keyframe::UpdateConnections()
{
    ASSERT_HAS_WRITE_LOCK;
    std::unordered_map<Keyframe*, float> KFcounter;

    vector<MapPoint*> vpMP;

    vpMP = observations;


    // For all map points in keyframe check in which other keyframes are they seen
    // Increase counter for those keyframes
    for (auto i : frame->featureRange())
    {
        MapPoint* mp = vpMP[i];

        if (!mp) continue;

        if (!(*mp)) continue;

        float weight = 1;

        if (frame->depth[i] > th_depth || mp->far_stereo_point)
        {
            continue;
            weight = 0.2;
        }

        for (auto obs : mp->GetObservationList())
        {
            if (obs.first->id() == id_) continue;
            KFcounter[obs.first] += weight;
        }
    }

    // This should not happen
    if (KFcounter.empty())
    {
        // can happen because we update in setBad now.
        return;
    }

    // If the counter is greater than threshold add connection
    // In case no keyframe counter is over threshold add the one with maximum counter
    int nmax         = 0;
    KeyFrame* pKFmax = NULL;
    int th           = 15;

    vector<std::pair<int, KeyFrame*>> vPairs;
    vPairs.reserve(KFcounter.size());
    for (auto kf_count : KFcounter)
    {
        if (kf_count.first->isBad()) continue;
        if (kf_count.second > nmax)
        {
            nmax   = kf_count.second;
            pKFmax = kf_count.first;
        }
        if (kf_count.second >= th)
        {
            vPairs.emplace_back(kf_count.second, kf_count.first);
            (kf_count.first)->AddConnection(this, kf_count.second);
        }
    }

    if (vPairs.empty() && pKFmax)
    {
        vPairs.emplace_back(nmax, pKFmax);
        pKFmax->AddConnection(this, nmax);
    }

    // Only sort by the key
    sort(vPairs.begin(), vPairs.end(), [](auto& a, auto& b) { return a.first < b.first; });

    connections.clear();
    connections = vPairs;

    if (!parent && children.empty())
    {
        parent = connections.back().second;
        parent->AddChild(this);
    }
}



void Keyframe::ComputeDepthRange(bool always)
{
    if (!always && median_depth > 0)
    {
        return;
    }

    ASSERT_HAS_READ_LOCK;
    vector<float> depths;
    depths.reserve(frame->N);


    for (auto i : frame->featureRange())
    {
        if (observations[i])
        {
            MapPoint* pMP = observations[i];
            // project to normalized
            Vec3 np = Pose() * pMP->getPosition();
            float z = np.z();
            depths.push_back(z);
        }
    }

    if (depths.empty())
    {
        return;
    }

    sort(depths.begin(), depths.end());
    median_depth = depths[(depths.size() - 1) / 2];
}

std::pair<double, int> Keyframe::reprojectionError()
{
    ASSERT_HAS_READ_LOCK;
    double ch = 0;
    int c     = 0;
    for (auto i : frame->featureRange())
    {
        auto mp = observations[i];
        if (!mp || mp->isBad()) continue;

        c++;
        if (frame->hasDepth(i))
        {
            SAIGA_ASSERT(stereo_cam.fx != 1);
            Vec3 ip   = stereo_cam.projectStereo(Pose() * mp->position);
            Vec3 diff = ip - Vec3(frame->undistorted_keypoints[i].point(0), frame->undistorted_keypoints[i].point(1),
                                  frame->right_points[i]);
            ch += diff.squaredNorm();
        }
        else
        {
            Vec2 ip   = mono_intrinsics.model.K.project(Pose() * mp->position);
            Vec2 diff = ip - frame->undistorted_keypoints[i].point;
            ch += diff.squaredNorm();
        }
    }
    return {ch, c};
}

std::pair<double, int> Keyframe::reprojectionErrorDebug()
{
    ASSERT_HAS_READ_LOCK;
    std::cout << "reprojectionErrorDebug " << *this << std::endl;
    double ch = 0;
    int c     = 0;
    for (auto i : frame->featureRange())
    {
        auto mp = observations[i];
        if (!mp || mp->isBad()) continue;

        c++;
        double r;
        if (frame->hasDepth(i))
        {
            SAIGA_ASSERT(stereo_cam.fx != 1);
            Vec3 ip   = stereo_cam.projectStereo(Pose() * mp->position);
            Vec3 diff = ip - Vec3(frame->undistorted_keypoints[i].point(0), frame->undistorted_keypoints[i].point(1),
                                  frame->right_points[i]);
            r         = diff.squaredNorm();
        }
        else
        {
            Vec2 ip   = mono_intrinsics.model.K.project(Pose() * mp->position);
            Vec2 diff = ip - frame->undistorted_keypoints[i].point;
            r         = diff.squaredNorm();
        }

        if (i < 200)
        {
            std::cout << i << " " << mp->id() << " Error: " << sqrt(r) << std::endl;
        }

        ch += r;
    }
    std::cout << "Total: " << c << " rmse " << sqrt(ch / c) << std::endl;
    std::cout << std::endl;
    return {ch, c};
}


bool Keyframe::IsValid()
{
    ASSERT_HAS_READ_LOCK;
    for (auto i : frame->featureRange())
    {
        auto mp = observations[i];
        if (!mp || mp->isBad()) continue;
        Vec3 ip = mono_intrinsics.model.K.project3(Pose() * mp->position);
        if (ip.z() < 0)
        {
            std::cout << "Point Projection with negative z detected " << *this << " z " << ip.z() << std::endl;
            mp->print();
            return false;
        }
    }
    return true;
}

int Keyframe::removeOutliers(double thresholdMono, double thresholdStereo)
{
    ASSERT_HAS_WRITE_LOCK;
    int removed_points = 0;
    for (auto i : frame->featureRange())
    {
        auto mp = observations[i];
        if (!mp || mp->isBad()) continue;

        if (frame->hasDepth(i))
        {
            SAIGA_ASSERT(stereo_cam.fx != 1);
            Vec3 p     = Pose() * mp->position;
            Vec3 ip    = stereo_cam.projectStereo(p);
            Vec3 diff  = ip - Vec3(frame->undistorted_keypoints[i].point(0), frame->undistorted_keypoints[i].point(1),
                                  frame->right_points[i]);
            auto error = diff.squaredNorm();
            if (p.z() <= 0 || error > thresholdStereo)
            {
                EraseMapPointMatch(i);
                mp->EraseObservation(this);
                removed_points++;
            }
        }
        else
        {
            Vec3 p     = Pose() * mp->position;
            Vec2 ip    = mono_intrinsics.model.K.project(p);
            Vec2 diff  = ip - frame->undistorted_keypoints[i].point;
            auto error = diff.squaredNorm();
            if (p.z() <= 0 || error > thresholdMono)
            {
                EraseMapPointMatch(i);
                mp->EraseObservation(this);
                removed_points++;
            }
        }
    }
    return removed_points;
}

MapPoint* Keyframe::GetMapPoint(int idx) const
{
    ASSERT_HAS_READ_LOCK;
    return observations[idx];
}

void Keyframe::setMapPoint(MapPoint* mp, int idx)
{
    ASSERT_HAS_WRITE_LOCK;
    observations[idx] = mp;
}

bool KeyFrame::ReplaceMapPointMatch(int idx, MapPoint* mp)
{
    ASSERT_HAS_WRITE_LOCK;

#if 1
    auto wp   = mp->position;
    auto pose = Pose();

    // Only replace if error low. Otherwise remove observation.
    auto ip3     = K.project3(pose * wp);
    double error = (frame->undistorted_keypoints[idx].point - ip3.head<2>()).squaredNorm();
    auto th      = reprojectionErrorThresholdMono * 2.0;
    if (ip3.z() > 0 && error < th * th)
    {
        observations[idx] = mp;
        return true;
    }
    else
    {
        observations[idx] = nullptr;
        return false;
    }
#else
    mvpMapPoints[idx] = mp;
    return true;
#endif
}

void Keyframe::EraseMapPointMatch(MapPoint* pMP)
{
    ASSERT_HAS_WRITE_LOCK;
    int idx = pMP->GetIndexInKeyFrame(this);
    if (idx >= 0) observations[idx] = nullptr;
}

void Keyframe::EraseMapPointMatch(int idx)
{
    ASSERT_HAS_WRITE_LOCK;
    observations[idx] = nullptr;
}

void Keyframe::PreintegrateFromPrevious(bool update_vb)
{
    ASSERT_HAS_WRITE_LOCK;

    SAIGA_ASSERT(has_imu);

    auto kf1 = previousKF;
    auto kf2 = this;

    if (kf1 == nullptr)
    {
        return;
    }

    //    std::cout << "recompute rpc " << *this << std::endl;

    SAIGA_ASSERT(!kf1->isBad());
    SAIGA_ASSERT(!kf2->isBad());

    SAIGA_ASSERT(kf1->frame->timeStamp == kf2->imudata.time_begin);
    SAIGA_ASSERT(kf2->frame->timeStamp == kf2->imudata.time_end);

    // 1. Preintegrate
    preint = Imu::Preintegration(kf1->velocity_and_bias);
    preint.IntegrateMidPoint(kf2->imudata, true);

    // 2. Predict
    auto p1          = kf1->PoseInv() * mono_intrinsics.camera_to_body;
    auto p_v         = preint.Predict(p1, kf1->velocity_and_bias.velocity, imu_gravity.Get());
    SE3 predicted_p2 = p_v.first;

    // 3. Convert IMU space -> camera space
    SE3 delta_R_body   = p1.inverse() * predicted_p2;
    SE3 delta_R_camera = (mono_intrinsics.camera_to_body * delta_R_body * mono_intrinsics.camera_to_body.inverse());
    delta_R_camera     = delta_R_camera.inverse();

    //    std::cout << delta_R_camera << std::endl;

    //    auto delta_R_camera2 =
    //        mono_intrinsics.camera_to_body * predicted_p2.inverse() * p1 * mono_intrinsics.camera_to_body.inverse();
    //    std::cout << delta_R_camera2 << std::endl;
    //    exit(0);

    kf2->rpc.img1     = kf1->id();
    kf2->rpc.img2     = kf2->id();
    kf2->rpc.rel_pose = delta_R_camera;
    kf2->preint_valid = true;

    if (update_vb)
    {
        kf2->velocity_and_bias          = kf1->velocity_and_bias;
        kf2->velocity_and_bias.velocity = p_v.second;
        imu_state                       = 1;
    }
}



std::ostream& operator<<(std::ostream& strm, const Keyframe& kf)
{
    strm << "(" << kf.id() << ", " << kf.frame->id << ")";
    return strm;
}



void Keyframe::SetBadFlag()
{
    ASSERT_HAS_WRITE_LOCK;

    if (isBad()) return;

    //    std::cout << "Remove " << *this << std::endl;


    // remove all bad children
    //    std::vector<Keyframe*> cpy(children.begin(), children.end());
    //    cpy.erase(std::remove_if(cpy.begin(), cpy.end(), [](const Keyframe* c) { return c->isBad(); }), cpy.end());
    //    children.clear();
    //    children.insert(cpy.begin(), cpy.end());


    if (parent && !*parent)
    {
        parent = nullptr;
    }

    if (parent)
    {
        // The default case: The keyframe to remove has a parent.
        // -> Set the parent of the children to the parent of the removed kf.
        // -> Compute relative pose so this keyframe can be localized later.
        SAIGA_ASSERT(*parent, "Parents must always be valid.");
        for (auto child : children)
        {
            child->ChangeParent(parent);
        }

        parent->EraseChild(this);
    }
    else if (!children.empty())
    {
        // Special case: This keyframe doesn't have a parent.
        // -> This keyframe is the root of a subgraph.
        // -> Make the child with the most connections to the new root node
        // -> Reference update parent of other childs to new root
        // -> set parent of this kf to the child (reverse dependency)
        // -> compute relative pose

        Keyframe* best_child = nullptr;
        int best_matches     = -1;
        for (auto child : children)
        {
            auto m = getWeightOfConnection(child);
            if (m > best_matches)
            {
                best_matches = m;
                best_child   = child;
            }
        }
        SAIGA_ASSERT(best_child);

        for (auto child : children)
        {
            if (child != best_child) child->ChangeParent(best_child);
        }
        best_child->parent = nullptr;

        SAIGA_ASSERT(*best_child);
        parent = best_child;
    }


    if (!parent && children.empty())
    {
        // We removed the last keyframe from the map
        // -> well, now the parent is actually 0
    }
    else
    {
        SAIGA_ASSERT(parent, "A removed keyframe must have a parent.");
        SAIGA_ASSERT(!parent->isBad(), "The parent of a removed keyframe must be initially valid.");
        children.clear();
        relToRef = Pose() * parent->PoseInv();
    }


    auto oldconnections = connections;
    // Erase connections in graph and observation of mappoints.
    for (auto p : connections) p.second->EraseConnection(this);
    connections.clear();
    for (size_t i = 0; i < observations.size(); i++)
    {
        if (observations[i] && !observations[i]->isBad()) observations[i]->EraseObservation(this);
    }

    auto next = nextKF;
    while (next && next->isBad())
    {
        next = next->nextKF;
    }

    //    std::cout << "rm " << *this << " next " << *next << std::endl;
    //    if (next && !next->isBad())
    //    {
    //    std::cout <<

    if (next)
    {
        next->previousKF = previousKF;
#if WITH_IMU
        if (has_imu)
        {
            imudata.Add(next->imudata);
            next->imudata      = imudata;
            next->preint_valid = false;
            // next->PreintegrateFromPrevious(false);
            SAIGA_ASSERT(next->frame->timeStamp == next->imudata.time_end);
        }
#endif

        //        next->rpc.rel_pose = rpc.rel_pose * next->rpc.rel_pose;
        //        next->rpc.img1     = rpc.img1;
    }

    if (previousKF)
    {
        previousKF->nextKF = next;
    }

    //    std::cout << previousKF->frame->timeStamp << " " << frame->timeStamp << " " << next->frame->timeStamp <<
    //    std::endl; std::cout << imudata << std::endl;



    //    }



    bad_              = true;
    frame->isKeyframe = false;

    map.EraseKeyFrame(*this);
    keyFrameDB->Remove(this);

#if 0
        for (auto kf : oldconnections)
        {
            kf.second->UpdateConnections();
        }
#endif
}



SE3 Keyframe::poseFromRef()
{
    ASSERT_HAS_READ_LOCK;
    auto refpos = referenceKF ? referenceKF->Pose() : SE3();
    return relToRef * refpos;
}

SE3 Keyframe::PoseGlobal()
{
    ASSERT_HAS_READ_LOCK;
    if (isBad())
    {
        SAIGA_ASSERT(parent, "A deleted keyframe must have a parent.");
        auto parentPose = parent->PoseGlobal();
        return relToRef * parentPose;
    }

    if (inMap)
        return Pose();
    else
        return poseFromRef();
}


TemplatedImage<ucvec4> Keyframe::createActiveFeatureImage()
{
    ASSERT_HAS_READ_LOCK;
    TemplatedImage<ucvec4> image(frame->image.dimensions());
    ImageTransformation::Gray8ToRGBA(frame->image.getImageView(), image.getImageView());


    for (auto i : frame->featureRange())
    {
        auto p = frame->keypoints[i].point;

        ucvec4 color(255, 0, 0, 255);

        if (observations[i] && !observations[i]->isBad())
        {
            color = ucvec4(0, 255, 0, 255);
        }

        ImageDraw::drawCircle(image.getImageView(), p.cast<float>(), 3, color);
    }
    return image;
}

void Keyframe::Transform(const SE3& t, double scale)
{
    if (median_depth > 0)
    {
        median_depth *= scale;
    }
    SE3 rel = relToRef.inverse();
    rel.translation() *= scale;
    relToRef = rel.inverse();

#if 0
// Scale in body space
    SE3 gp      = PoseInv() * mono_intrinsics.camera_to_body;
    DSim3 gpsim = DSim3(t, scale) * DSim3(gp, 1.0);
    gp          = gpsim.se3() * globalCamera->GroundTruthToCamera();
    SetPose(gp.inverse());
#else
    // scale in camera world space
    SE3 gp                     = PoseInv();
    DSim3 gpsim                = DSim3(t, scale) * DSim3(gp, 1.0);
    gp                         = gpsim.se3();
    velocity_and_bias.velocity = DSim3(t, scale) * velocity_and_bias.velocity;
    SetPose(gp.inverse());
#endif
}



const vector<MapPoint*>& Keyframe::GetMapPointMatches() const
{
    ASSERT_HAS_READ_LOCK;
    return observations;
}

void Keyframe::GetMapPointMatches(vector<MapPoint*>& points) const
{
    ASSERT_HAS_READ_LOCK;
    points = observations;
}

}  // namespace Snake
