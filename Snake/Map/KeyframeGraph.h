/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once
#include "saiga/core/Core.h"

#include "Frame.h"
#include "System/SnakeGlobal.h"
#include "LocalMapping/Bow.h"

#include <set>

namespace Snake
{
/**
 * A graph with weighted edges.
 * The edges are always stored ordered.
 */
class Keyframe;


class KeyframeGraph
{
    using T      = Keyframe;
    using Weight = int;

    using KeyframeType = T;
    using PointerType  = T*;

    static constexpr PointerType InvalidNode = nullptr;

   public:
    KeyframeGraph(PointerType node_id) : node_id(node_id) {}



    Keyframe* GetParent();

    void AddChild(PointerType pKF);


    bool empty();
    int size() const;

    Weight getWeightOfConnection(PointerType v);



    ArrayView<const std::pair<Weight, PointerType>> GetBestCovisibilityKeyFrames(int N) const;

    void GetBestCovisibilityKeyFrames(vector<PointerType>& v, int N);

    vector<PointerType> GetVectorCovisibleKeyFrames();
    void GetVectorCovisibleKeyFrames(vector<PointerType>& v);


    void ChangeParent(PointerType p);



    void EraseChild(PointerType pKF);


    void AddConnection(PointerType p, Weight weight);


    void EraseConnection(PointerType p);

    void GetCovisiblesByWeight(vector<PointerType>& s, Weight w);

    vector<PointerType> GetCovisiblesByWeight(Weight w);

    std::vector<PointerType> GetConnectedKeyFrames();

    void GetConnectedKeyFrames(std::vector<PointerType>& s);

    std::vector<std::pair<Weight, PointerType>> GetConnections();

   protected:
    PointerType node_id = InvalidNode;

    std::vector<std::pair<Weight, PointerType>> connections;

    // Spanning Tree
    PointerType parent = InvalidNode;
    std::set<PointerType> children;

    int find(PointerType kf);
};



}  // namespace Snake
