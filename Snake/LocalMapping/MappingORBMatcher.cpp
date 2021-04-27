/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#include "MappingORBMatcher.h"

#include "saiga/vision/reconstruction/Epipolar.h"

#include "Map/Map.h"
namespace Snake
{
int MappingORBMatcher::SearchForTriangulation2(const Keyframe* kf1, const Keyframe* kf2, const Mat3& E,
                                               std::vector<std::pair<int, int>>& vMatchedPairs, float epipolarDistance,
                                               int featureDistance)
{
    auto& vFeatVec1 = kf1->frame->bow_feature_vec;
    auto& vFeatVec2 = kf2->frame->bow_feature_vec;

    double th_chi1 = epipolarDistance * 2 / K.fx;
    double th_chi2 = th_chi1 * th_chi1;


    int nmatches = 0;
    tmp_flags.clear();
    tmp_flags.resize(kf2->frame->N, false);

    auto f1it  = vFeatVec1.begin();
    auto f2it  = vFeatVec2.begin();
    auto f1end = vFeatVec1.end();
    auto f2end = vFeatVec2.end();

    while (f1it != f1end && f2it != f2end)
    {
        if (f1it->first == f2it->first)
        {
            for (auto idx1 : f1it->second)
            {
                MapPoint* pMP1 = kf1->GetMapPoint(idx1);

                // If there is already a MapPoint skip
                if (pMP1) continue;


                auto np1 = kf1->frame->normalized_points[idx1];

                auto& d1 = kf1->frame->descriptors[idx1];

                int bestDist = TH_LOW;
                int bestIdx2 = -1;


                for (auto idx2 : f2it->second)
                {
                    MapPoint* pMP2 = kf2->GetMapPoint(idx2);

                    // If we have already matched or there is a MapPoint skip
                    if (tmp_flags[idx2] || pMP2) continue;


                    auto& d2 = kf2->frame->descriptors[idx2];

                    const int dist = distance(d1, d2);

                    if (dist > featureDistance || dist > bestDist) continue;

                    auto np2 = kf2->frame->normalized_points[idx2];

                    double disepi = (Saiga::EpipolarDistanceSquared(np1, np2, E));

                    if (disepi < th_chi2)
                    {
                        bestIdx2 = idx2;
                        bestDist = dist;
                    }
                }

                if (bestIdx2 >= 0)
                {
                    vMatchedPairs.emplace_back(idx1, bestIdx2);
                    nmatches++;
                }
            }

            f1it++;
            f2it++;
        }
        else if (f1it->first < f2it->first)
        {
            f1it = std::lower_bound(f1it, vFeatVec1.end(), *f2it,
                                    [](const auto& a, const auto& b) { return a.first < b.first; });
        }
        else
        {
            f2it = std::lower_bound(f2it, vFeatVec2.end(), *f1it,
                                    [](const auto& a, const auto& b) { return a.first < b.first; });
        }
    }
    return nmatches;
}


int MappingORBMatcher::SearchForTriangulationBF(const SE3& pose1, const SE3& pose2, const Keyframe* pKF1,
                                                const Keyframe* pKF2, const Mat3& E12,
                                                std::vector<std::pair<int, int>>& vMatchedPairs, float epipolarDistance,
                                                int featureDistance)
{
    double th_chi1 = 10 / K.fx;
    double th_chi2 = th_chi1 * th_chi1;

    int nmatches = 0;

    tmp_flags.clear();
    tmp_flags.resize(pKF2->frame->N, false);


    for (auto idx1 : pKF1->frame->featureRange())
    {
        MapPoint* pMP1 = pKF1->GetMapPoint(idx1);
        // If there is already a MapPoint skip
        if (pMP1) continue;

        auto& np1 = pKF1->frame->normalized_points[idx1];
        auto& d1  = pKF1->frame->descriptors[idx1];

        int bestDist = TH_LOW;
        int bestIdx2 = -1;


        for (auto idx2 : pKF2->frame->featureRange())
        {
            MapPoint* pMP2 = pKF2->GetMapPoint(idx2);

            // If we have already matched or there is a MapPoint skip
            if (tmp_flags[idx2] || pMP2) continue;


            auto& np2     = pKF2->frame->normalized_points[idx2];
            double disepi = (Saiga::EpipolarDistanceSquared(np1, np2, E12));

            if (disepi > th_chi2)
            {
                continue;
            }

            auto& d2       = pKF2->frame->descriptors[idx2];
            const int dist = distance(d1, d2);

            if (dist > featureDistance || dist > bestDist) continue;


            bestIdx2 = idx2;
            bestDist = dist;
        }

        if (bestIdx2 >= 0)
        {
            vMatchedPairs.emplace_back(idx1, bestIdx2);
            nmatches++;
        }
    }

    return nmatches;
}


int MappingORBMatcher::SearchForTriangulationProject(const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>& grid,
                                                     const SE3& pose1, const SE3& pose2, const Keyframe* pKF1,
                                                     const Keyframe* pKF2, const Mat3& E12,
                                                     std::vector<std::pair<int, int>>& vMatchedPairs,
                                                     float epipolarDistance, int featureDistance)
{
    double th_chi1 = epipolarDistance / K.fx;
    double th_chi2 = th_chi1 * th_chi1;

    SAIGA_ASSERT(grid.rows() >= featureGridBounds.Rows / 4);
    SAIGA_ASSERT(grid.cols() >= featureGridBounds.Cols / 4);


    int nmatches = 0;

    tmp_flags.clear();
    tmp_flags.resize(pKF2->frame->N, false);


    for (auto idx1 : pKF1->frame->featureRange())
    {
        MapPoint* pMP1 = pKF1->GetMapPoint(idx1);
        // If there is already a MapPoint skip
        if (pMP1) continue;


        auto cell = featureGridBounds.cellClamped(pKF1->frame->undistorted_keypoints[idx1].point);
        auto z    = grid(cell.second / 4, cell.first / 4);

        Vec3 wp  = pKF1->PoseInv() * K.unproject(pKF1->frame->undistorted_keypoints[idx1].point, z);
        Vec2 ip2 = K.project(pKF2->Pose() * wp);


        if (!featureGridBounds.inImage(ip2)) continue;

        auto& np1 = pKF1->frame->normalized_points[idx1];
        auto& d1  = pKF1->frame->descriptors[idx1];

        int bestDist = TH_LOW;
        int bestIdx2 = -1;


        tmp_indices.clear();
        pKF2->frame->GetFeaturesInArea(tmp_indices, ip2, 20);

        for (auto idx2 : tmp_indices)
        {
            MapPoint* pMP2 = pKF2->GetMapPoint(idx2);

            // If we have already matched or there is a MapPoint skip
            if (tmp_flags[idx2] || pMP2) continue;


            auto& np2     = pKF2->frame->normalized_points[idx2];
            double disepi = (Saiga::EpipolarDistanceSquared(np1, np2, E12));

            if (disepi > th_chi2)
            {
                continue;
            }

            auto& d2       = pKF2->frame->descriptors[idx2];
            const int dist = distance(d1, d2);

            if (dist > featureDistance || dist > bestDist) continue;



            {
                bestIdx2 = idx2;
                bestDist = dist;
            }
        }

        if (bestIdx2 >= 0)
        {
            vMatchedPairs.emplace_back(idx1, bestIdx2);
            nmatches++;
        }
    }
    return nmatches;
}


int MappingORBMatcher::Fuse(Keyframe* kf, const vector<MapPoint*>& points,
                            std::vector<std::pair<int, int>>& fuseCandidates, float th, float obs_factor,
                            int feature_th)
{
    auto pose           = kf->Pose();
    auto cameraposition = pose.inverse().translation();
    int fusedPoints     = 0;
    for (auto mp : points)
    {
        if (!mp || mp->isBad() || mp->IsInKeyFrame(kf)) continue;
        Vec3 wp = mp->getPosition();
        Vec3 np = pose * wp;
        if (np.z() <= 0) continue;

        auto ip = K.project(np);
        if (!featureGridBounds.inImage(ip)) continue;

        Vec3 PO   = cameraposition - wp;
        auto dist = PO.norm();


#ifdef MATCHING_MIN_MAX_DISTANCE2
        // Check distance is in the scale invariance region of the MapPoint
        auto [min_distance, max_distance] =
            scalePyramid.EstimateMinMaxDistance(mp->reference_depth, mp->reference_scale_level);
        if (dist < min_distance || dist > max_distance)
        {
            continue;
        }
#endif

        // Viewing angle must be less than 60 deg
        auto Pn = mp->GetNormal();
        if (PO.transpose() * Pn < 0.5 * dist) continue;



        // allow a higher threshold for points with only 2 observations
        // if we don't fuse them soon they will be removed
        float observationFactor = 1.0;
        if (mp->GetNumObservations() <= 2)
        {
            observationFactor = obs_factor;
        }



        // Search in a radius
        auto radius = observationFactor * th;

        tmp_indices.clear();
#ifdef MATCHING_CHECK_SCALE_CONSISTENCY2
        auto prediction = scalePyramid.PredictScaleLevel(mp->reference_depth, mp->reference_scale_level, dist);
        radius *= scalePyramid.ScaleForContiniousLevel(prediction);
        kf->frame->GetFeaturesInAreaAndCheckScale(tmp_indices, ip, radius, prediction);
#else
        kf->frame->GetFeaturesInArea(tmp_indices, ip, radius);
#endif

        if (tmp_indices.empty()) continue;

        // Match to the most similar keypoint in the radius

        auto dMP = mp->GetDescriptor();

        int bestDist = 256;
        int bestIdx  = -1;


        for (auto idx : tmp_indices)
        {
            auto& kp = kf->frame->undistorted_keypoints[idx];
            if (kf->frame->hasDepth(idx))
            {
                Vec3 ips = stereo_cam.projectStereo(np);
                // Check reprojection error in stereo
                Vec3 ips2(kp.point.x(), kp.point.y(), kf->frame->right_points[idx]);
                //                SAIGA_EXIT_ERROR("not implemented");
                auto e2 = (ips - ips2).squaredNorm();
                if (e2 * scalePyramid.InverseScale(kp.octave) > reprojectionErrorThresholdStereo2) continue;
            }
            else
            {
                auto e2 = (ip - kp.point).squaredNorm();
                if (e2 * scalePyramid.InverseScale(kp.octave) > reprojectionErrorThresholdMono2) continue;
            }

            auto& dKF = kf->frame->descriptors[idx];
            auto dist = distance(dMP, dKF);
            if (dist < bestDist)
            {
                bestDist = dist;
                bestIdx  = idx;
            }
        }

        if (bestDist <= feature_th)
        {
            fuseCandidates.emplace_back(bestIdx, mp->id());
            fusedPoints++;
        }
    }
    SAIGA_EXIT_ERROR("sdf");
    return fusedPoints;
}

#if 1
int MappingORBMatcher::Fuse(Keyframe* kf, const SE3& pose, std::vector<bool>* point_mask,
                            const LocalMap<FusionPoint>& points, std::vector<std::pair<int, int>>& fuseCandidates,
                            float th, float obs_factor, int feature_th)
{
    //    int tid           = OMP::getThreadNum();
    //    auto& tmp_flags   = tmp_flags_per_thread[tid];
    //    auto& tmp_indices = tmp_indices_per_thread[tid];

    fuseCandidates.clear();
    SAIGA_ASSERT(!point_mask || point_mask->size() == points.points.size());
    auto th_squared = th * th;
    //    auto pose           = kf->Pose();
    auto cameraposition = pose.inverse().translation();
    int fusedPoints     = 0;
    //    for (auto lmp : points.points)
    for (int i = 0; i < points.points.size(); ++i)
    {
        auto& lmp = points.points[i];


        if (point_mask && (*point_mask)[i] == false) continue;
        //         SAIGA_ASSERT(!point_mask || (*point_mask)[i] == !map.getMapPoint(lmp.id()).IsInKeyFrame(kf));
        //        SAIGA_ASSERT(!map.getMapPoint(lmp.id).IsInKeyFrame(kf));
        //        SAIGA_ASSERT(!map.getMapPoint(lmp.id).isBad());



        Vec3 wp = lmp.position;
        Vec3 np = pose * wp;

        if (np.z() <= 0) continue;

        auto ip = K.project(np);
        if (!featureGridBounds.inImage(ip)) continue;

        Vec3 PO   = cameraposition - wp;
        auto dist = PO.norm();


#    ifdef MATCHING_MIN_MAX_DISTANCE2
        // Check distance is in the scale invariance region of the MapPoint
        auto [min_distance, max_distance] =
            scalePyramid.EstimateMinMaxDistance(lmp.reference_depth, lmp.reference_scale_level);
        if (dist < min_distance || dist > max_distance)
        {
            continue;
        }
#    endif

        // Viewing angle must be less than 60 deg
        auto Pn = lmp.normal;

        //        if (PO.dot(Pn) < 0.5 * dist3D) continue;
        if (PO.transpose() * Pn < 0.5 * dist) continue;



        // allow a higher threshold for points with only 2 observations
        // if we don't fuse them soon they will be removed
        float observationFactor = 1.0;
        if (lmp.observations <= 2)
        {
            observationFactor = obs_factor;
        }



        // Search in a radius
        auto radius = observationFactor * th;

        tmp_indices.clear();
#    ifdef MATCHING_CHECK_SCALE_CONSISTENCY2
        auto prediction = scalePyramid.PredictScaleLevel(lmp.reference_depth, lmp.reference_scale_level, dist);
        kf->frame->GetFeaturesInAreaAndCheckScale(tmp_indices, ip, radius, prediction);
#    else
        kf->frame->GetFeaturesInArea(tmp_indices, ip, radius);
#    endif

        if (tmp_indices.empty()) continue;

        // Match to the most similar keypoint in the radius

        auto dMP = lmp.descriptor;

        int bestDist = 256;
        int bestIdx  = -1;


        for (auto idx : tmp_indices)
        {
            auto& kp = kf->frame->undistorted_keypoints[idx];
            if (kf->frame->hasDepth(idx))
            {
                //                Vec3 ips = rgbd_intrinsics->model.K.project3(np);
                Vec3 ips = stereo_cam.projectStereo(np);
                // Check reprojection error in stereo
                Vec3 ips2(kp.point.x(), kp.point.y(), kf->frame->right_points[idx]);
                //                SAIGA_EXIT_ERROR("not implemented");
                auto e2 = (ips - ips2).squaredNorm();
                if (e2 > th_squared * observationFactor) continue;
            }
            else
            {
                auto e2 = (ip - kp.point).squaredNorm();
                if (e2 > th_squared * observationFactor) continue;
            }

            auto& dKF = kf->frame->descriptors[idx];
            auto dist = distance(dMP, dKF);
            if (dist < bestDist)
            {
                bestDist = dist;
                bestIdx  = idx;
            }
        }

        if (bestDist <= feature_th)
        {
            fuseCandidates.emplace_back(bestIdx, lmp.id);
            fusedPoints++;
        }
    }
    return fusedPoints;
}
#endif

int fuseCandidatesIntoKf(const std::vector<std::pair<int, int>>& fcs, Keyframe* kf)
{
    if (fcs.empty()) return 0;

    int fusedPoints = 0, addedConnections = 0;

    int test1 = 0;
    int test2 = 0;
    for (auto fc : fcs)
    {
        int kpid = fc.first;



        MapPoint* newmp     = &map.getMapPoint(fc.second);
        MapPoint* currentmp = kf->GetMapPoint(kpid);


        // From the triangulation matcher all points are valid.
        // But fusing the (below) can erase points so we need to check this here again.
        if (!newmp || newmp->isBad())
        {
            test1++;
            continue;
        }


        // don't add double connections (one of them must be an oulier)
        if (newmp->IsInKeyFrame(kf))
        {
            test2++;
            continue;
        }

        if (currentmp && !currentmp->isBad())
        {
            // there is already a point at this location
            // replace the "weaker" point
            if (newmp->GetNumObservations() > currentmp->GetNumObservations())
                currentmp->Replace(newmp);
            else
                newmp->Replace(currentmp);
            fusedPoints++;
        }
        else
        {
            // this keypoint doesn't have a match
            // -> just add the connection
            newmp->AddObservation(kf, kpid);
            kf->addMappoint(newmp, kpid);
            addedConnections++;
        }
    }

    //    std::cout << "fuseCandidatesIntoKf " << fcs.size() << " " << test1 << " " << test2 << " " << addedConnections
    //    << " "
    //              << fusedPoints << std::endl;
    return fusedPoints + addedConnections;
}

}  // namespace Snake
