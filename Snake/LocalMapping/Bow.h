/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once

#include "saiga/vision/slam/MiniBow2.h"
namespace Snake
{
using ORBVocabulary = MiniBow2::TemplatedVocabulary<MiniBow2::Descriptor>;
using MiniBow2::BowVector;
using MiniBow2::FeatureVector;

inline ORBVocabulary vocabulary;


}  // namespace Snake
