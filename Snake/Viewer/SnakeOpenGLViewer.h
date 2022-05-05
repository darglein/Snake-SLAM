/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#pragma once

#include "saiga/opengl/assets/all.h"
#include "saiga/opengl/rendering/deferredRendering/deferredRendering.h"
#include "saiga/opengl/rendering/forwardRendering/forwardRendering.h"
#include "saiga/opengl/rendering/renderer.h"
#include "saiga/opengl/window/WindowTemplate.h"
#include "saiga/opengl/world/LineSoup.h"
#include "saiga/opengl/world/TextureDisplay.h"
#include "saiga/opengl/world/pointCloud.h"
#include "saiga/opengl/world/proceduralSkybox.h"

#include "Map/Frame.h"
#include "Map/LocalMap.h"
#include "Map/ViewerInterface.h"
#include "System/SnakeGlobal.h"
#include "Viewer/VisualVoxelFusion.h"
using namespace Saiga;

namespace Snake
{
class SnakeOpenGLViewer : public StandaloneWindow<WindowManagement::GLFW, DeferredRenderer>,
                          public glfw_KeyListener,
                          public ViewerInterface
{
   public:
    Glfw_Camera<PerspectiveCamera> offset_camera;
    PerspectiveCamera render_camera;
    Object3D interCam;
    Object3D interCam2;

    std::shared_ptr<GLPointCloud> pointCloud;
    LineSoup lineSoup;
    LineSoup velocityLines, velocityLines2;
    std::shared_ptr<LineVertexColoredAsset> frustum;
    std::shared_ptr<LineVertexColoredAsset> kf_frustum;

    AABB bounding_box;
    AABB smooth_bounding_box;
    std::shared_ptr<LineVertexColoredAsset> bounding_box_mesh;

    SimpleAssetObject groundPlane;

    std::shared_ptr<Texture> texture;
    TextureDisplay display;

    ProceduralSkybox skybox;

    std::shared_ptr<Texture> t;

    SnakeOpenGLViewer(const ViewerSettings& vparams, const std::string& config);
    ~SnakeOpenGLViewer();

    void update(float dt) override;
    void interpolate(float dt, float interpolation) override;


    void render(RenderInfo render_info) override;

    void keyPressed(int key, int scancode, int mods) override;

    virtual void run() override { StandaloneWindow::run(); }

    virtual void setMap(std::unique_ptr<ViewerMap> map) override
    {
        std::unique_lock l(mut);
        this->newMap = std::move(map);
    }

    virtual void setFrame(std::unique_ptr<ViewerFrame> frame) override
    {
        std::unique_lock l(mut);
        this->newImage = std::move(frame);
    }

    std::mutex mut;

   private:
    VisualFusion scene;
    std::shared_ptr<DirectionalLight> sun;
    bool view_shading = false;
    SE3 currentPose;
    mat4 camera_follow_pose = mat4::Identity();
    Vec3 currentColor;
    std::unique_ptr<ViewerMap> newMap, map;
    std::unique_ptr<ViewerFrame> newImage;
};
}  // namespace Snake
