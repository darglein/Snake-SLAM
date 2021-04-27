/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once

#include "saiga/core/util/Thread/all.h"

#include "System/SnakeGlobal.h"

namespace Snake
{
class DelayedParallelMapOptimization
{
   public:
    DelayedParallelMapOptimization(const std::string& name, bool enable, int delay, bool parallel = false);
    virtual ~DelayedParallelMapOptimization();

    // Adds a keyframe to the queue.
    // This call is non-blocking
    // If max_kf_size is set, the new kf is only added if the queue size is less then the given value
    void Add(Keyframe* kf, int max_kf_size = -1);

    // Adds the complete map to the queue (doesn't process it)
    void AddAllKeyframesToQueue();

    int QueueSize();
    void Join();
    void Update();

    // Runs on all remaining items in the queue. This ignores the value of delay.
    // The queue will will be empty after this call.
    void ForceCleanQueue();

    virtual void Process(Keyframe* kf) = 0;

    // Signals this module to pause. (non-blocking)
    void Pause();

    // Waits until this module is actually pausing
    void WaitUntilPaused();

    // Resumes this module operation
    void Resume();

    bool IsParallel() { return parallel; }

    int Busy();

   private:
    std::string name;
    bool enable   = true;
    int delay     = 5;
    bool parallel = false;

    std::vector<Keyframe*> keyframes;

    // The critical sections are very small -> spinlock
    mutable SpinLock mutex;

    int latest_kf_id = -1;
    std::vector<Keyframe*> tmp_keyframes;

    std::atomic_bool doing_work = false;
    std::atomic_bool running    = false;
    ScopedThread thread;
    Saiga::Semaphore sem;

    // Calls Process() on all keyframe that need to be processed.
    // This call blocks until it is completed.
    void UpdateInternal();

    // === Pause stuff ===
    std::atomic_bool need_pause = false;
    Saiga::Semaphore wait_for_pause;
    Saiga::Semaphore resume_sem;
};



}  // namespace Snake
