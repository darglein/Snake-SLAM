/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once

#include "System/SnakeGlobal.h"
namespace Snake
{
// The viewer has to enable these flags so the system sends the data.
// This is done to reduce overhead for stuff which is not needed by the viewer.
struct ViewerRequests
{
    // Current keyframe pose each time a new kf is inserted
    bool keyframe_updates = true;

    // Every frame's pose is sent after every keyframe.
    bool all_frame_updates = false;

    // All current map points (is resent every time a kf is inserted)
    bool map_points = true;

    bool bounding_box = false;

    // The covisibilty connections between the keyframes
    // This requires also 'keyframe_updates'.
    bool keyframe_graph = false;

    // Keyframe data as they are inserted
    bool new_keyframes = false;

    // The current frame overlayed with the feature points
    bool frame_feature_image = false;

    // A copy of the input images
    bool frame_image_copy = false;

    // Copy image only for keyframes
    bool keyframe_image_copy = false;
};

// A global variable. Set the flags in the constructor of your viewer.
inline ViewerRequests viewer_requests;



// The data sent for every frame
// A few things has to be enabled in the requests
struct ViewerFrame
{
    // Always set
    int id;
    bool valid_pose;
    SE3 pose;
    std::optional<SE3> ground_truth;
    bool is_keyframe = false;
    int keyframe_id  = -1;

    // Set if requested
    TemplatedImage<ucvec4> feature_image;

    TemplatedImage<unsigned char> image;
    TemplatedImage<ucvec4> left_rgb;
    TemplatedImage<unsigned char> right_image;
    TemplatedImage<float> depth_image;


    ViewerFrame() {}

    // Called by Snake-SLAM. Uses the request-settings.
    ViewerFrame(FramePtr frame);
};

// A reduced map for the viewer which is updated every keyframe
// This map includes all keyframes including the removed ones.
// The valid flag indicates if a keyframe is actually used.
struct ViewerMap
{
    struct Keyframe
    {
        Vec3 velocity;
        int id;
        SE3 pose;
        std::optional<SE3> ground_truth;
        int valid      = false;
        int debug_flag = 0;
    };
    std::vector<Keyframe> keyframes;


    // Only sent if requested.
    struct Frame
    {
        bool is_keyframe = false;
        int id;
        bool valid;
        SE3 pose;
        std::optional<SE3> ground_truth;
    };
    std::vector<Frame> frames;

    // Axis aligned bounding box of all mappoints
    std::pair<Vec3, Vec3> bounding_box = {Vec3::Zero(), Vec3::Zero()};

    // Bounding box using the 5% quantiles to filter outliers
    std::pair<Vec3, Vec3> bounding_box_median5 = {Vec3::Zero(), Vec3::Zero()};

    // edge between two keyframes (given by the id)
    std::vector<std::pair<int, int>> covisibilty_edges;
    std::vector<Vec3> points;

    ViewerMap();
};



/**
 * If you want to build a custom viewer for snake SLAM, derive
 * from this class. The functions will be called by the snake system
 * automatically. These functions should be implemented non-blocking,
 * for example, by copying the data. The snake system waits for these
 * functions to finish.
 */
class ViewerInterface
{
   public:
    ViewerInterface(const ViewerSettings& vparams) : params(vparams) {}
    virtual ~ViewerInterface() {}

    /**
     * Execute the viewer loop.
     * This function should return when the window is closed.
     * The threading is handled by the Snake-SLAM system.
     */
    virtual void run() = 0;


    virtual void setFrame(std::unique_ptr<ViewerFrame> frame) = 0;
    virtual void setMap(std::unique_ptr<ViewerMap> map)       = 0;

    // A signal from Snake SLAM to the viewer that no more frames are coming.
    virtual void InputFinished() {}

    virtual void setSnakeImguiFunction(std::function<void(void)> f) { imguiCallback = f; }
    const ViewerSettings& getParams() { return params; }

   protected:
    ViewerSettings params;
    std::function<void(void)> imguiCallback;
};

// Owned by system
inline ViewerInterface* viewer = nullptr;


}  // namespace Snake
