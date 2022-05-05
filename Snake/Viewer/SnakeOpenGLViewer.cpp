/**
 * Copyright (c) 2021 Darius Rückert
 * Licensed under the MIT License.
 * See LICENSE file for more information.
 */

#include "SnakeOpenGLViewer.h"

#include "saiga/core/imgui/imgui.h"
#include "saiga/core/math/CoordinateSystems.h"
#include "saiga/core/math/random.h"
#include "saiga/core/model/model_from_shape.h"
#include "saiga/opengl/shader/shaderLoader.h"

#include "Map/Map.h"
namespace Snake
{
double scale                   = 1;
static vec3 defaultRelPosition = vec3(0, 0.5, 4) * scale;
static quat defaultRelRotation = quat::Identity();

SnakeOpenGLViewer::SnakeOpenGLViewer(const ViewerSettings& vparams, const std::string& config)
    : StandaloneWindow(config), ViewerInterface(vparams)
{
    viewer_requests.frame_feature_image = true;
    viewer_requests.keyframe_graph      = true;
    viewer_requests.map_points          = true;
    viewer_requests.keyframe_updates    = true;
    viewer_requests.all_frame_updates   = true;
    viewer_requests.bounding_box        = true;

    // Define GUI layout
    auto editor_layout = std::make_unique<EditorLayoutL>();
    editor_layout->RegisterImguiWindow("Viewer", EditorLayoutL::WINDOW_POSITION_LEFT_BOTTOM);
    editor_layout->RegisterImguiWindow("System", EditorLayoutL::WINDOW_POSITION_LEFT);
    editor_gui.SetLayout(std::move(editor_layout));
    editor_gui.enabled = true;

    float aspect = window->getAspectRatio();
    offset_camera.setProj(70.0f, aspect, 0.1f, 200.0f, false);
    offset_camera.enableInput();
    offset_camera.movementSpeed     = 4 * scale;
    offset_camera.movementSpeedFast = 10 * scale;
    offset_camera.mouseTurnLocal    = true;
    offset_camera.mousemap[0]       = GLFW_MOUSE_BUTTON_RIGHT;
    offset_camera.setPosition(defaultRelPosition);
    offset_camera.rot           = defaultRelRotation;
    offset_camera.rotationPoint = vec3(0, 0, 0);
    offset_camera.calculateModel();

    renderer->timer->Enable(false);


    render_camera.setProj(70.0f, aspect, 0.1f, 200.0f, false);
    //    camera.proj = camera.proj * CV2GLView();

    window->setCamera(&render_camera);

    groundPlane.asset = std::make_shared<ColoredAsset>(
        CheckerBoardPlane(make_ivec2(20, 20), 2.0f, Colors::darkgray, Colors::lightgray));
    groundPlane.rotateLocal(vec3(1, 0, 0), 180);
    groundPlane.translateGlobal(vec3(0, 2, 0));
    groundPlane.calculateModel();


    bounding_box      = AABB(vec3(-1, -1, -1), vec3(1, 1, 1));
    bounding_box_mesh = std::make_shared<LineVertexColoredAsset>(
        GridBoxLineMesh(bounding_box, ivec3(10, 10, 10)).SetVertexColor(vec4(1, 1, 1, 1)));

    frustum = std::make_shared<LineVertexColoredAsset>(
        FrustumCVLineMesh(Snake::K.matrix().cast<float>(), 0.1 * scale, 640, 480).SetVertexColor(vec4(1, 1, 1, 1)));

    lineSoup.lineWidth           = params.GraphLineWidth;

    sun = std::make_shared<DirectionalLight>();
    renderer->lighting.AddLight(sun);

    // create one directional light
    sun->setDirection(vec3(1, 5, 5));
    sun->setColorDiffuse(LightColorPresets::DirectSunlight);
    sun->setIntensity(1.0);
    sun->setAmbientIntensity(0.1f);
    // sun->createShadowMap(2048, 2048, 3);
}

SnakeOpenGLViewer::~SnakeOpenGLViewer() {}

void SnakeOpenGLViewer::update(float dt)
{
    if (renderer->use_keyboard_input_in_3dview) offset_camera.update(dt);

    std::unique_ptr<ViewerMap> currentMap;
    std::unique_ptr<ViewerFrame> currentImage;
    {
        std::unique_lock l(mut);
        currentImage = std::move(newImage);
        currentMap   = std::move(newMap);
    }

    if (currentMap)
    {
        map = std::move(currentMap);


        bounding_box.min = map->bounding_box_median5.first.cast<float>();
        bounding_box.max = map->bounding_box_median5.second.cast<float>();

        sun->fitShadowToCamera(&render_camera);
        sun->fitNearPlaneToScene(bounding_box);

        // offset_camera.rotationPoint = bounding_box.getPosition();


        UnifiedMesh point_mesh;
        for (auto wp : map->points)
        {
            PointVertex v;
            v.position = wp.cast<float>();
            v.color    = params.color_points;
            vec4 color(1,1,1,1);
            color.head<3>() = v.color;

            point_mesh.position.push_back(v.position);
            point_mesh.color.push_back(color);
        }
        pointCloud = std::make_shared<GLPointCloud>(point_mesh);


        lineSoup.lines.clear();
        for (auto e : map->covisibilty_edges)
        {
            auto p1 = map->keyframes[e.first].pose.translation();
            auto p2 = map->keyframes[e.second].pose.translation();


            PointVertex pc1, pc2;


            pc1.position = p1.cast<float>();
            pc2.position = p2.cast<float>();

            pc1.color = params.color_graph;
            pc2.color = pc1.color;

            lineSoup.lines.push_back(pc1);
            lineSoup.lines.push_back(pc2);
        }
        lineSoup.updateBuffer();

        velocityLines.lines.clear();

        for (auto kf : map->keyframes)
        {
            auto p = kf.pose.translation().cast<float>();
            auto v = kf.velocity.cast<float>();


            PointVertex pc1, pc2;
            pc1.position = p;
            pc2.position = p + v * 1;

            pc1.color = vec3(1, 0, 0);
            pc2.color = pc1.color;

            velocityLines.lines.push_back(pc1);
            velocityLines.lines.push_back(pc2);
        }
        velocityLines.updateBuffer();
    }

    if (currentImage)
    {
        if (currentImage->valid_pose)
        {
            currentPose = currentImage->pose;
        }

        auto& feaut_img = currentImage->feature_image;
        feaut_img.getImageView().flipY();
        if (!texture)
        {
            texture = std::make_shared<Texture>(feaut_img, false, false);
        }
        else
        {
            texture->updateFromImage(feaut_img);
        }
    }
}

void SnakeOpenGLViewer::interpolate(float dt, float interpolation)
{
    // Update the camera rotation. This could also be done in 'update' but
    // doing it in the interpolate step will reduce latency
    if (renderer->use_mouse_input_in_3dview) offset_camera.interpolate(dt, interpolation);
    offset_camera.recalculateMatrices();

    if (params.followCamera)
    {
        camera_follow_pose = currentPose.matrix().cast<float>();

        if (params.smoothCamera)
        {
            Object3D ob;
            ob.setModelMatrix(camera_follow_pose);
            interCam2 = Object3D::interpolate(interCam2, ob, 0.1);
            interCam2.calculateModel();
            camera_follow_pose = interCam2.model;
        }
    }
    render_camera.setView(inverse(offset_camera.model) * CV2GLView() * inverse(camera_follow_pose));
    render_camera.recalculatePlanes();

    float bb_sm             = 0.02;
    smooth_bounding_box.min = bb_sm * bounding_box.min + (1.0 - bb_sm) * smooth_bounding_box.min;
    smooth_bounding_box.max = bb_sm * bounding_box.max + (1.0 - bb_sm) * smooth_bounding_box.max;

    groundPlane.position.y() = smooth_bounding_box.min.y();
    groundPlane.calculateModel();


    AABB render_bb = smooth_bounding_box;
    render_bb.min.y() -= 0.05;


    if (view_shading)
    {
        auto d = render_camera.getDirection().head<3>();
        sun->setDirection(-d);
    }
}

void SnakeOpenGLViewer::render(RenderInfo render_info)
{
    if (render_info.render_pass == RenderPass::Deferred || render_info.render_pass == RenderPass::Shadow)
    {
        if (params.renderFloor)
        {
            groundPlane.render(render_info.camera);
        }
        scene.Render(render_info.camera);
    }
    if (render_info.render_pass == RenderPass::Forward)
    {
        // skybox.render(cam, CV2GLView());

        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);


        glLineWidth(4);
        if (render_info.render_pass == RenderPass::Shadow) glLineWidth(2);


        if (params.renderBoundingbox)
        {
            mat4 bounding_box_transform =
                Saiga::translate(smooth_bounding_box.getPosition()) * Saiga::scale(smooth_bounding_box.Size() * 0.5);
            bounding_box_mesh->SetShaderColor(vec4(1, 1, 1, 1));
            bounding_box_mesh->renderForward(render_info.camera, bounding_box_transform);
        }


        if (params.renderPoints && pointCloud)
        {
            pointCloud->point_size = params.PointSize;
            if (render_info.render_pass == RenderPass::Shadow) pointCloud->point_size = 2;
            pointCloud->render(render_info);
        }

        if (params.renderEdges)
        {
            lineSoup.lineWidth = params.GraphLineWidth;
            lineSoup.render(render_info.camera);
        }

        if (params.renderVelocity)
        {
            velocityLines.lineWidth = 3;
            velocityLines.render(render_info.camera);
        }

        // current
        glLineWidth(params.CameraLineWidth);
        if (params.renderCurrentCam)
        {
            mat4 m = currentPose.matrix().cast<float>();
            frustum->SetShaderColor(vec4(0, 1, 1, 1));
            frustum->renderForward(render_info.camera, m);
        }



        if (map)
        {
            if (params.renderKeyframes)
            {
                glLineWidth(params.KeyFrameLineWidth);
                for (auto kf : map->keyframes)
                {
                    if (!kf.valid) continue;
                    mat4 m = kf.pose.matrix().cast<float>();

                    if (kf.debug_flag == 1)
                    {
                        frustum->SetShaderColor(vec4(1, 0, 0, 1));
                    }
                    else if (kf.debug_flag == 2)
                    {
                        frustum->SetShaderColor(vec4(1, 1, 0, 1));
                    }
                    else
                    {
                        frustum->SetShaderColor(make_vec4(params.color_keyframes, 1));
                    }
                    frustum->renderForward(render_info.camera, m);
                }
            }
            if (params.renderFrames)
            {
                glLineWidth(params.KeyFrameLineWidth);
                for (auto f : map->frames)
                {
                    if (!f.valid) continue;
                    mat4 m = f.pose.matrix().cast<float>() * Saiga::scale(make_vec3(0.3));
                    frustum->renderForward(render_info.camera, m);
                }
            }
        }

        float inputH = 380;
        if (params.renderInput && texture)
        {
            float ratio  = float(texture->getWidth()) / texture->getHeight();
            float inputW = ratio * inputH;
            display.render(texture.get(), {0, renderer->viewport_size.y() - inputH}, {inputW, inputH});
        }
    }
    else if (render_info.render_pass == RenderPass::GUI)
    {
        if (ImGui::Begin("Viewer"))
        {
            if (ImGui::CollapsingHeader("Fusion"))
            {
                scene.imgui();
                if (ImGui::Button("Fuse Depth Maps"))
                {
                    SAIGA_ASSERT(settings.inputType == InputType::RGBD);

                    params.renderEdges      = false;
                    params.renderPoints     = false;
                    params.renderKeyframes  = false;
                    params.renderCurrentCam = false;
                    params.renderFrames     = false;
                    {
                        auto lock = Snake::map.LockReadOnly();
                        auto kfs  = Snake::map.GetAllFrames();

                        scene.K   = rgbd_intrinsics.depthModel.K;
                        scene.dis = rgbd_intrinsics.depthModel.dis;
                        scene.images.clear();
                        for (auto kf : kfs)
                        {
                            //                        if (kf->isBad()) continue;
                            if (!kf->validPose) continue;

                            FusionImage dm;
                            dm.depthMap = kf->depth_image;
                            dm.V        = kf->Pose();
                            scene.images.push_back(dm);
                        }
                    }
                    scene.Fuse();
                }
            }

            params.imgui();
            if (ImGui::Checkbox("view_shading", &view_shading))
            {
                sun->castShadows = !view_shading;
            }

            if (ImGui::Checkbox("followCamera", &params.followCamera))
            {
                if (params.followCamera)
                {
                    offset_camera.setPosition(defaultRelPosition);
                    offset_camera.rot = defaultRelRotation;
                    offset_camera.calculateModel();
                }
                else
                {
                    offset_camera.setModelMatrix(inverse(offset_camera.view));
                }
            }
        }
        ImGui::End();

        if (imguiCallback) imguiCallback();
    }
}



void SnakeOpenGLViewer::keyPressed(int key, int scancode, int mods)
{
    switch (key)
    {
        case GLFW_KEY_SPACE:
            pause = !pause;
            break;
        case GLFW_KEY_ESCAPE:
            if (stop_camera)
            {
                window->close();
            }
            else
            {
                stop_camera = true;
            }
            break;
        default:
            break;
    }
}


}  // namespace Snake
