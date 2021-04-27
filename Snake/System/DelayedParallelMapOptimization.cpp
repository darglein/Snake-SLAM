/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */


#include "DelayedParallelMapOptimization.h"

#include "Map/Map.h"

namespace Snake
{
DelayedParallelMapOptimization::DelayedParallelMapOptimization(const std::string& name, bool enable, int delay,
                                                               bool parallel)
    : name(name), enable(enable), delay(delay), parallel(parallel)
{
    keyframes.reserve(1000);
    tmp_keyframes.reserve(1000);

    running = true;
    if (parallel)
    {
        thread = Saiga::ScopedThread([this]() {
            Saiga::setThreadName(this->name);
            Saiga::Random::setSeed(settings.randomSeed);
            while (running)
            {
                sem.wait();
                UpdateInternal();
            }
        });
    }
}

DelayedParallelMapOptimization::~DelayedParallelMapOptimization()
{
    running = false;
    sem.notify();
}

void DelayedParallelMapOptimization::Add(Keyframe* kf, int max_kf_size)
{
    if (!enable)
    {
        return;
    }

    if (!kf)
    {
        return;
    }
    {
        std::unique_lock l(mutex);
        if (kf->id() > latest_kf_id)
        {
            latest_kf_id = kf->id();
        }

        if (max_kf_size > 0)
        {
            if (keyframes.size() < max_kf_size)
            {
                keyframes.push_back(kf);
            }
        }
        else
        {
            keyframes.push_back(kf);
        }
    }
    sem.notify();
}

void DelayedParallelMapOptimization::AddAllKeyframesToQueue()
{
    std::vector<int> keyframes, points;
    map.GetAllKeyFrames(keyframes);
    for (auto i : keyframes)
    {
        auto& kf = map.getKeyframe(i);
        Add(&kf);
    }
}

int DelayedParallelMapOptimization::QueueSize()
{
    std::unique_lock l(mutex);
    return keyframes.size();
}

void DelayedParallelMapOptimization::Join()
{
    running    = false;
    need_pause = false;
    sem.notify();
    resume_sem.notify();

    if (parallel && thread.joinable())
    {
        thread.join();
    }
    parallel = false;
}

void DelayedParallelMapOptimization::Update()
{
    if (!parallel)
    {
        UpdateInternal();
    }
}

void DelayedParallelMapOptimization::UpdateInternal()
{
    if (need_pause)
    {
        wait_for_pause.notify();
        need_pause = false;
        resume_sem.wait();
    }

    doing_work = true;

    while (running)
    {
        tmp_keyframes.clear();
        {
            std::unique_lock l(mutex);
            for (auto kf : keyframes)
            {
                SAIGA_ASSERT(kf);
                if (kf->id() + delay <= latest_kf_id)
                {
                    tmp_keyframes.push_back(kf);
                }
            }
            keyframes.erase(std::remove_if(keyframes.begin(), keyframes.end(),
                                           [=](auto kfp) { return kfp->id() + delay <= latest_kf_id; }),
                            keyframes.end());
        }

        if (tmp_keyframes.empty())
        {
            break;
        }

        std::sort(keyframes.begin(), keyframes.end());
        keyframes.erase(std::unique(keyframes.begin(), keyframes.end()), keyframes.end());

        for (auto kf : tmp_keyframes)
        {
            Process(kf);
        }
    }
    doing_work = false;
}

void DelayedParallelMapOptimization::ForceCleanQueue()
{
    SAIGA_ASSERT(!parallel);
    tmp_keyframes.clear();
    {
        std::unique_lock l(mutex);
        tmp_keyframes = keyframes;
        keyframes.clear();
    }

    for (auto kf : tmp_keyframes)
    {
        Process(kf);
    }
}

void DelayedParallelMapOptimization::Pause()
{
    need_pause = true;
    sem.notify();
}

void DelayedParallelMapOptimization::WaitUntilPaused()
{
    wait_for_pause.wait();
}

void DelayedParallelMapOptimization::Resume()
{
    resume_sem.notify();
}

int DelayedParallelMapOptimization::Busy()
{
    std::unique_lock l(mutex);
    return int(doing_work) + keyframes.size();
}

}  // namespace Snake
