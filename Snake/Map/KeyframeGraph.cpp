/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#include "KeyframeGraph.h"

#include "Map/Map.h"

namespace Snake
{
Keyframe* KeyframeGraph::GetParent()
{
    ASSERT_HAS_READ_LOCK;
    return parent;
}

void KeyframeGraph::AddChild(KeyframeGraph::PointerType pKF)
{
    ASSERT_HAS_WRITE_LOCK;
    children.insert(pKF);
}

bool KeyframeGraph::empty()
{
    ASSERT_HAS_READ_LOCK;
    return connections.empty();
}

int KeyframeGraph::size() const
{
    ASSERT_HAS_READ_LOCK;
    return connections.size();
}

KeyframeGraph::Weight KeyframeGraph::getWeightOfConnection(KeyframeGraph::PointerType v)
{
    ASSERT_HAS_READ_LOCK;
    auto pos = find(v);
    if (pos == size())
        return 0;
    else
        return connections[pos].first;
}



ArrayView<const std::pair<KeyframeGraph::Weight, KeyframeGraph::PointerType> >
KeyframeGraph::GetBestCovisibilityKeyFrames(int N) const
{
    ASSERT_HAS_READ_LOCK;
    //    vector<PointerType> result;

    N = std::min(N, size());

    //    for (int i = size() - N; i < size(); ++i)
    //    {
    //        result.push_back(connections[i].second);
    //    }
    //    return result;
    return {connections.data() + size() - N, size_t(N)};
}

void KeyframeGraph::GetBestCovisibilityKeyFrames(vector<KeyframeGraph::PointerType>& v, int N)
{
    ASSERT_HAS_READ_LOCK;
    SAIGA_ASSERT(v.empty());
    // Return the last N elements
    //        v = GetBestCovisibilityKeyFrames(N);

    N = std::min(N, size());

    for (int i = size() - N; i < size(); ++i)
    {
        v.push_back(connections[i].second);
    }
}

vector<KeyframeGraph::PointerType> KeyframeGraph::GetVectorCovisibleKeyFrames()
{
    ASSERT_HAS_READ_LOCK;
    return GetConnectedKeyFrames();
}

void KeyframeGraph::GetVectorCovisibleKeyFrames(vector<KeyframeGraph::PointerType>& v)
{
    ASSERT_HAS_READ_LOCK;
    GetConnectedKeyFrames(v);
}


void KeyframeGraph::ChangeParent(KeyframeGraph::PointerType p)
{
    ASSERT_HAS_WRITE_LOCK;
    SAIGA_ASSERT(p);
    SAIGA_ASSERT(p != node_id);
    SAIGA_ASSERT(*p);
    parent = p;
    p->AddChild(node_id);
}

void KeyframeGraph::EraseChild(KeyframeGraph::PointerType pKF)
{
    ASSERT_HAS_WRITE_LOCK;
    children.erase(pKF);
}

void KeyframeGraph::AddConnection(KeyframeGraph::PointerType p, KeyframeGraph::Weight weight)
{
    ASSERT_HAS_WRITE_LOCK;
    auto pos = find(p);
    if (pos == size())
    {
        // add new
        connections.emplace_back(weight, p);
    }
    else
    {
        // update
        connections[pos].first = weight;
    }

    std::sort(connections.begin(), connections.end());
}

void KeyframeGraph::EraseConnection(KeyframeGraph::PointerType p)
{
    ASSERT_HAS_WRITE_LOCK;
    auto pos = find(p);

    if (pos == size())
    {
        // this connection doesn't even exist
        return;
    }
    SAIGA_ASSERT(pos < size());

    // move everything after pos 1 position to the left.
    // -> The list stays ordered.
    for (auto i = pos + 1; i < size(); ++i)
    {
        connections[i - 1] = connections[i];
    }
    connections.resize(connections.size() - 1);
}

void KeyframeGraph::GetCovisiblesByWeight(vector<KeyframeGraph::PointerType>& s, KeyframeGraph::Weight w)
{
    ASSERT_HAS_READ_LOCK;
    s.clear();
    for (auto p : connections)
    {
        if (p.first >= w)
        {
            s.push_back(p.second);
        }
    }
}

vector<KeyframeGraph::PointerType> KeyframeGraph::GetCovisiblesByWeight(KeyframeGraph::Weight w)
{
    ASSERT_HAS_READ_LOCK;
    std::vector<PointerType> s;
    for (auto p : connections)
    {
        if (p.first >= w)
        {
            s.push_back(p.second);
        }
    }
    return s;
    //        if (empty()) return {};

    //        int i = 0;
    //        for (auto p : connections)
    //        {
    //            if (p.first >= w) break;
    //            ++i;
    //        }

    //        // Return vector from i to end.
    //        vector<PointerType> result;
    //        for (; i < (int)size(); ++i)
    //        {
    //            result.push_back(connections[i].second);
    //        }
    //        return result;
}

std::vector<KeyframeGraph::PointerType> KeyframeGraph::GetConnectedKeyFrames()
{
    ASSERT_HAS_READ_LOCK;
    std::vector<PointerType> s;
    for (auto p : connections) s.push_back(p.second);
    return s;
}

void KeyframeGraph::GetConnectedKeyFrames(std::vector<KeyframeGraph::PointerType>& s)
{
    ASSERT_HAS_READ_LOCK;
    s.clear();
    for (auto p : connections) s.push_back(p.second);
}

std::vector<std::pair<KeyframeGraph::Weight, KeyframeGraph::PointerType> > KeyframeGraph::GetConnections()
{
    ASSERT_HAS_READ_LOCK;
    return connections;
}

int KeyframeGraph::find(KeyframeGraph::PointerType kf)
{
    ASSERT_HAS_READ_LOCK;
    int i = 0;
    for (auto p : connections)
    {
        if (p.second == kf) return i;
        ++i;
    }
    return i;
}



}  // namespace Snake
