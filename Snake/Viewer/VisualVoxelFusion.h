/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once
#include "saiga/opengl/window/SampleWindowDeferred.h"
#include "saiga/vision/reconstruction/VoxelFusion.h"

#include <set>
namespace Saiga
{
class VisualFusion : public FusionScene
{
   public:
    // immiediatley returns
    void Fuse() override;

    // called from render thread
    void Render(Camera* cam);

    ScopedThread thread;

    std::vector<vec3> voxel_block_positions;
    std::atomic_int voxel_blocks_ready = 0;
    Timer voxel_block_timer;


    std::atomic_int mesh_ready = 0;

    SimpleAssetObject voxel_asset_object;

    std::shared_ptr<ColoredAsset> tri_asset;
};



}  // namespace Saiga
