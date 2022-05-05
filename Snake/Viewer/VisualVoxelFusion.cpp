/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */
#include "VisualVoxelFusion.h"

#include "saiga/core/model/model_from_shape.h"
namespace Saiga
{
void VisualFusion::Fuse()
{
    std::cout << "VisualFusion " << Size() << " depth maps..." << std::endl;
    if (thread.joinable())
    {
        thread.join();
    }

    voxel_blocks_ready = 0;
    voxel_block_positions.clear();

    thread = Saiga::ScopedThread([this]() {
        Saiga::setThreadName("Fusion");
        Preprocess();
        AnalyseSparseStructure();
        ComputeWeight();
        // Allocate();

        for (auto& block : tsdf->blocks)
        {
            voxel_block_positions.push_back(tsdf->GlobalBlockOffset(block.index));
        }
        voxel_blocks_ready = 1;


        Integrate();
        ExtractMesh();

        mesh_ready = 1;
        std::cout << *tsdf << std::endl;
    });
}

void VisualFusion::Render(Camera* cam)
{
    static const int indices_per_cube = 12 * 3;
    vec4 gray_color                   = vec4(0.7, 0.7, 0.7, 1);

    float time_voxel    = 15;
    float time_triangle = 20;

    glDisable(GL_CULL_FACE);


    if (voxel_blocks_ready == 1)
    {
        // create mesh

        UnifiedMesh mesh;


        for (auto p : voxel_block_positions)
        {
            float s = 1.0 / tsdf->block_size_inv;
            vec3 p2 = p + vec3(s, s, s);
            AABB box(p, p2);

            auto mesh2 = BoxMesh(box);
            //            std::cout << *mesh2 << std::endl;
            mesh = UnifiedMesh(mesh, mesh2);
        }


        mesh.SetVertexColor(gray_color);
        voxel_asset_object.asset = std::make_shared<ColoredAsset>(mesh);

        voxel_block_timer.start();
        voxel_blocks_ready = 2;
    }

    if (voxel_blocks_ready == 2)
    {
        voxel_block_timer.stop();
        double alpha = (voxel_block_timer.getTimeMS() / 1000.0) / time_voxel;

        if (alpha < 1)
        {
            std::shared_ptr<ColoredAsset> asset = std::dynamic_pointer_cast<ColoredAsset>(voxel_asset_object.asset);

            asset->deferredShader->bind();
            asset->deferredShader->uploadModel(mat4::Identity());
            asset->unified_buffer->Bind();

            int count = alpha * asset->unified_buffer->num_elements;
            asset->unified_buffer->Draw(0, count);
            asset->deferredShader->unbind();
        }
        else
        {
            voxel_blocks_ready = 3;
        }
    }

    if (voxel_blocks_ready == 3)
    {
        voxel_asset_object.render(cam);

        if (mesh_ready == 1)
        {
            tri_asset          = std::make_shared<ColoredAsset>(mesh);
            mesh_ready         = 2;
            voxel_blocks_ready = 4;

            voxel_block_timer.start();
        }
    }

    if (mesh_ready == 2)
    {
        voxel_block_timer.stop();
        double alpha = (voxel_block_timer.getTimeMS() / 1000.0) / time_triangle;

        if (alpha < 1)
        {
            int blocks = alpha * tsdf->current_blocks;

            {
                std::shared_ptr<ColoredAsset> asset = std::dynamic_pointer_cast<ColoredAsset>(voxel_asset_object.asset);

                int offset          = blocks * indices_per_cube;
                int remaining_count = asset->unified_buffer->num_elements;

                //                int offset = 0;
                //                int remaining_count = blocks * indices_per_cube

                SAIGA_ASSERT(remaining_count >= 0);

                asset->deferredShader->bind();
                asset->deferredShader->uploadModel(mat4::Identity());
                asset->unified_buffer->Bind();
                asset->unified_buffer->Draw(offset / 3, remaining_count);
                asset->unified_buffer->Unbind();
                asset->deferredShader->unbind();
            }



            {
                int count = triangle_soup_inclusive_prefix_sum[blocks] ;

                if(tri_asset->deferredShader->bind())
                {
                    tri_asset->deferredShader->uploadModel(mat4::Identity());
                    tri_asset->unified_buffer->Bind();
                    tri_asset->unified_buffer->Draw(0, count);
                    tri_asset->unified_buffer->Unbind();
                    tri_asset->deferredShader->unbind();
                }
            }
        }
        else
        {
            mesh_ready = 3;
        }
    }

    if (mesh_ready == 3)
    {
        tri_asset->render(cam, mat4::Identity());
    }

    glEnable(GL_CULL_FACE);
}



}  // namespace Saiga
