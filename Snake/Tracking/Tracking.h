/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once
#include "saiga/core/Core.h"
#include "saiga/core/imgui/imgui_saiga.h"

#include "Initialization/Initializer.h"
#include "Map/Map.h"
#include "PoseRefinement.h"
#include "SnakeORBMatcher.h"
#include "StatePredictor.h"
#include "System/Module.h"
#include "System/SnakeGlobal.h"
namespace Saiga
{
class MotionModel;

}

namespace Snake
{
class LocalMapping;
class Tracking : public Module
{
   public:
#define REF_SWITCH 0
#if REF_SWITCH
    struct TrackingReferenceSwitch
    {
        Keyframe* source_keyframe;
        Keyframe* target_keyframe;
        DSim3 source_to_target;
        std::atomic_bool need_switch_coarse = false;
        std::atomic_bool need_switch_fine   = false;
    };

    TrackingReferenceSwitch trackingReferenceSwitch;
#endif

    Tracking();
    ~Tracking();
    void run();

    void waitForFinish()
    {
        if (thread.joinable())
        {
            thread.join();
        }
    }


    void SignalRescale(double scale) { predictor.Rescale(scale); }

    // Waits until the tracking thread is at the barrier
    // The tracking thread then waits until BarrierResume is called.
    void BarrierWait()
    {
        SAIGA_BLOCK_TIMER();
        tracking_barrier = true;
        while (!at_barrier)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
    }

    // Resumes a waiting tracking thread
    void BarrierResume() { tracking_barrier = false; }

    std::atomic_bool at_barrier       = false;
    std::atomic_bool tracking_barrier = false;



   public:
    StatePredictor predictor;

    bool is_rescaled    = false;
    double scale_factor = 1;

    Keyframe* track(FramePtr frame);

    void createFeatureImage(Frame& frame);
    enum class State
    {
        NOT_INITIALIZED,
        OK,
        RECOVERING,  // this state is only valid after a few frames when tracking is lost
        RELOCALIZE_AND_INITIALIZE,
    };
    State state                             = State::NOT_INITIALIZED;
    int recoverCounter                      = 0;
    static constexpr int MAX_RECOVER_FRAMES = 3;

    int validInARow = 3;



    std::shared_ptr<Initializer> initializer;


    Thread thread;

    // meassures
    int trackFrame = 0;
    Saiga::Timer trackTimer;


    void createInitializer();


    // =========== Coarse tracking ===========

    SE3 referencePose, lastPose;
    KeyFrame* reference_keyframe = nullptr;

    FramePtr lastTrackedFrame;
    LocalMap<CoarseTrackingPoint> last_frame_lm;
    SnakeORBMatcher localMapMatcher;
    PoseRefinement poseRefinement;


    void PredictFramePosition(FramePtr frame);
    bool TrackCoarse(FramePtr frame);
    void BuildCoarseLocalMap(FramePtr frame);


    // Predict the new position and try to find matches by projecting world points into the the frame.
    bool TrackWithPrediction(Frame& frame, float radius);

    // Use a brute force matcher + ransac-pnp solver to find the frame position relative to a keyframe.
    bool TrackBruteForce(Frame& frame, Keyframe* ref, int min_matches, bool has_local_map);

    bool try_localize(FramePtr frame);



    // =========== Fine tracking ===========

    std::unordered_map<KeyFrame*, int> keyframeCounter;

    struct PerKeyframeData
    {
        int in_local_keyframes = 0;
        int point_counter      = 0;
    };
    std::vector<PerKeyframeData> perKeyframeData;


    std::vector<std::pair<int, Keyframe*>> count_keyframe;
    std::vector<int> mnTrackReferenceForFrame;
    std::vector<KeyFrame*> local_keyframes;
    std::vector<Keyframe*> tmp_keyframes;
    std::vector<Keyframe*> indirect_neighbor;
    //    std::vector<MapPoint*> newLocalPoints;
    vector<MapPoint*> tmpPoints;


    KeyFrame* last_keyframe = nullptr;
    LocalMap<FineTrackingPoint> lm;

    std::tuple<bool, Keyframe*> TrackFine(FramePtr framep);

    void updateLocalMap(Frame& frame);
    int computePose(Frame& frame);
    void UpdateLocalKeyFrames2(Frame& frame);
    int UpdateLocalPoints(Frame& frame);


    // =========== Inserter ===========

    std::pair<bool, double> NeedNewKeyframe(FramePtr frame);
    Keyframe* CreateNewKeyFrame(FramePtr frame, double cull_factor);


    // Settings which are not supposed to be changed by the user.
    static constexpr int coarse_min_inliers_last_frame            = 20;
    static constexpr int coarse_min_inliers_last_keyframe         = 15;
    static constexpr float coarse_projection_search_radius_mono   = 15;
    static constexpr float coarse_projection_search_radius_stereo = 10;

    static constexpr int fine_min_projection_matches            = 25;
    static constexpr int fine_min_pose_optimziation_inliers     = 25;
    static constexpr float fine_projection_search_radius_mono   = 5;
    static constexpr float fine_projection_search_radius_stereo = 4;
};


// Owned by System. Global variable to enable easy access from different modules.
inline Tracking* tracking;



}  // namespace Snake
