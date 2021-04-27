/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */
#include "ViewerInterface.h"

#include "saiga/core/image/ImageDraw.h"
#include "saiga/core/imgui/imgui.h"
#include "saiga/core/util/ini/ini.h"

#include "Map/Map.h"
namespace Snake
{
ViewerFrame::ViewerFrame(FramePtr frame)
{
    SAIGA_ASSERT(frame);
    id           = frame->id;
    valid_pose   = frame->validPose;
    pose         = frame->tmpPose.inverse();
    ground_truth = frame->groundTruth;
    is_keyframe  = frame->isKeyframe;

    if (frame->isKeyframe)
    {
        auto kf = frame->referenceKF();
        SAIGA_ASSERT(kf);
        keyframe_id = kf->id();
    }

    if (viewer_requests.frame_image_copy || (viewer_requests.keyframe_image_copy && frame->isKeyframe))
    {
        image       = frame->image;
        left_rgb    = frame->image_rgb;
        right_image = frame->right_image;
        depth_image = frame->depth_image;
    }

    if (viewer_requests.frame_feature_image)
    {
        auto img = frame->image.getImageView();
        feature_image.create(img.dimensions());

        for (auto i : img.rowRange())
        {
            for (auto j : img.colRange())
            {
                auto v              = img(i, j);
                feature_image(i, j) = ucvec4(v, v, v, 255);
            }
        }

        {
            auto lock = map.LockReadOnly();
            for (auto i : frame->featureRange())
            {
                auto p     = frame->keypoints[i].point;
                auto color = ucvec4(255, 0, 0, 255);

                if (frame->mvpMapPoints[i] && !frame->mvpMapPoints[i]->isBad())
                {
                    color = ucvec4(0, 255, 0, 255);
                }
                else
                {
                    //                continue;
                }

                ImageDraw::drawCircle(feature_image.getImageView(), p.cast<float>(), 3, color);
            }
        }
    }
}

ViewerMap::ViewerMap()
{
    auto lock = map.LockReadOnly();



    if (viewer_requests.map_points)
    {
        std::vector<int> mapPoints;
        map.GetAllMapPoints(mapPoints);
        for (auto mpid : mapPoints)
        {
            auto mp = &map.getMapPoint(mpid);
            Vec3 wp = mp->position;
            points.push_back(wp);
        }

        if (viewer_requests.bounding_box && !points.empty())
        {
            std::vector<double> point_x, point_y, point_z;
            for (auto p : points)
            {
                point_x.push_back(p.x());
                point_y.push_back(p.y());
                point_z.push_back(p.z());
            }
            std::sort(point_x.begin(), point_x.end());
            std::sort(point_y.begin(), point_y.end());
            std::sort(point_z.begin(), point_z.end());

            bounding_box.first  = Vec3(point_x.front(), point_y.front(), point_z.front());
            bounding_box.second = Vec3(point_x.back(), point_y.back(), point_z.back());


            //            double ratio = 0.05;
            //            int o                      = point_x.size() * ratio;
            int o = std::max(int(point_x.size()) - 20, 0);

            bounding_box_median5.first = Vec3(point_x[o], point_y[o], point_z[o]);

            //            o                           = point_x.size() * (1.0 - ratio);
            o                           = std::min(20, int(point_x.size()));
            bounding_box_median5.second = Vec3(point_x[o], point_y[o], point_z[o]);

            Vec3 diff = (bounding_box_median5.second - bounding_box_median5.first) * 0.1;
            diff.y()  = 0;
            bounding_box_median5.first -= diff;
            bounding_box_median5.second += diff;
        }
    }

    if (viewer_requests.keyframe_updates)
    {
        for (int kfid = 0; kfid < map.MaxNumberOfKeyframes(); ++kfid)
        {
            auto mkf = &map.getKeyframe(kfid);
            if (!mkf->inMap) continue;
            SAIGA_ASSERT(mkf->frame);
            Keyframe vkf;
            vkf.pose         = mkf->Pose().inverse();
            vkf.ground_truth = mkf->frame->groundTruth;
            vkf.id           = mkf->id();
            vkf.valid        = !mkf->isBad();
            vkf.velocity     = mkf->velocity_and_bias.velocity;
            vkf.debug_flag   = mkf->debug_flag;
            keyframes.push_back(vkf);
        }


        if (viewer_requests.keyframe_graph)
        {
            for (int kfid = 0; kfid < map.MaxNumberOfKeyframes(); ++kfid)
            {
                auto kf = &map.getKeyframe(kfid);
                if (kf->isBad()) continue;


                if (kf->previousKF)
                {
                    covisibilty_edges.push_back({kf->previousKF->id(), kf->id()});
                }

#if 1
                //                auto neighs = kf->GetCovisiblesByWeight(50);
                auto neighs = kf->GetBestCovisibilityKeyFrames(5);


                //                neighs.push_back(kf->GetParent());

                for (auto na : neighs)
                {
                    auto n = na.second;
                    if (!n || n->isBad()) continue;

                    if (na.first < 25)
                    {
                        continue;
                    }
                    int i = kfid;
                    int j = n->id();
                    if (i > j) continue;
                    covisibilty_edges.push_back({i, j});
                }
#endif
            }
        }

        if (viewer_requests.all_frame_updates)
        {
            std::vector<FramePtr> snake_frames;
            auto c = map.getKeyframe(map.MaxNumberOfKeyframes() - 1).frame;
            for (; c; c = c->previousFrame)
            {
                c->UpdateReference();
                if (!c->validPose) continue;

                auto kf = c->referenceKF();
                if (!kf || kf->isBad()) continue;


                c->tmpPose = c->getPoseFromReference();
                snake_frames.push_back(c);
            }
            std::reverse(snake_frames.begin(), snake_frames.end());


            for (auto f : snake_frames)
            {
                ViewerMap::Frame vf;
                vf.ground_truth = f->groundTruth;
                vf.id           = f->id;
                vf.is_keyframe  = f->isKeyframe;
                vf.pose         = f->tmpPose.inverse();
                vf.valid        = f->validPose;
                frames.push_back(vf);
            }
        }
    }
}

}  // namespace Snake
