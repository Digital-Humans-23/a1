#pragma once

#include <gui/application.h>
#include <gui/camera.h>
#include <gui/light.h>
#include <gui/renderer.h>
#include <gui/shader.h>
#include <kinematics/IK_Solver.h>
#include <locomotion/FootFallPattern.h>
#include <locomotion/KinematicTrackingController.h>
#include <locomotion/SimpleLocomotionTrajectoryPlanner.h>
#include <robot/Robot.h>
#include <utils/logger.h>

#include "robots.h"

using namespace crl::gui;

namespace crl {
namespace app {
namespace locomotion {

class App : public Basic3DAppWithShadows {
public:
    App(const char *title = "CRL Playground - Locomotion App - kinematic",
        std::string iconPath = CRL_DATA_FOLDER "/crl_icon_grey.png")
        : Basic3DAppWithShadows(title, iconPath) {
        camera = TrackingCamera(5);
        camera.rotAboutUpAxis = -0.75;
        camera.rotAboutRightAxis = 0.5;
        light.s = 0.04f;
        shadowbias = 0.00001f;
        camera.aspectRatio = float(width) / height;
        glEnable(GL_DEPTH_TEST);

        showConsole = true;
        automanageConsole = true;
        Logger::maxConsoleLineCount = 10;
        consoleHeight = 225;

        this->targetFramerate = 30;
        this->limitFramerate = true;

        // setup
        setupRobotAndController(robotModels[0]);
    }

    virtual ~App() override {
        if (controller) {
            delete controller->planner;
            delete controller;
        }
    }

    virtual void resizeWindow(int width, int height) override {
        camera.aspectRatio = float(width) / height;
        return Application::resizeWindow(width, height);
    }

    bool mouseMove(double xpos, double ypos) override {
        camera.processMouseMove(mouseState, keyboardState);
        return true;
    }

    virtual bool scrollWheel(double xoffset, double yoffset) override {
        camera.processMouseScroll(xoffset, yoffset);
        return true;
    }

    void process() override {
        if (appIsRunning == false) return;

        controller->planner->appendPeriodicGaitIfNeeded(getPeriodicGait(robot));

        double simTime = 0;
        while (simTime < 1.0 / targetFramerate) {
            simTime += dt;
            controller->computeAndApplyControlSignals(dt);
            controller->advanceInTime(dt);

            if (cameraShouldFollowRobot) {
                camera.target.x = robot->trunk->state.pos.x;
                camera.target.z = robot->trunk->state.pos.z;
            }

            light.target.x() = robot->trunk->state.pos.x;
            light.target.z() = robot->trunk->state.pos.z;

            if (slowMo) break;
        }

        controller->generateMotionTrajectories();
    }

    virtual void drawAuxiliaryInfo() override {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        drawFPS();

        drawConsole();

        drawImGui();

        ImGui::EndFrame();
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    }

    // objects drawn with a shadowMapRenderer (during shadow pass) will cast a
    // shadow
    virtual void drawShadowCastingObjects() override {
        robot->draw(shadowMapRenderer);
    }

    // objects drawn with a shadowShader (during the render pass) will have
    // shadows cast on them
    virtual void drawObjectsWithShadows() override {
        ground.draw(shadowShader, V3D(0.6, 0.6, 0.8));
        robot->draw(shadowShader);
    }

    // objects drawn with basic shadowShader (during the render pass) will not
    // have shadows cast on them
    virtual void drawObjectsWithoutShadows() override {
        // draw sphere around feet
        // if Ex.1 is correctly implemented, green sphere will be rendered around the feet
        GeneralizedCoordinatesRobotRepresentation gcrr(robot);
        for (uint i = 0; i < robot->limbs.size(); i++) {
            P3D eePos = gcrr.getWorldCoordinates(
                robot->limbs[i]->ee->endEffectorOffset, robot->limbs[i]->eeRB);
            drawSphere(eePos, 0.025, basicShader, V3D(0, 1, 0));
        }

        if (drawDebugInfo) controller->drawDebugInfo(&basicShader);
    }

    virtual bool keyPressed(int key, int mods) override {
        bool dirty = false;
        if (key == GLFW_KEY_SPACE) {
            appIsRunning = !appIsRunning;
        }
        if (key == GLFW_KEY_BACKSPACE) {
            screenIsRecording = !screenIsRecording;
        }
        if (key == GLFW_KEY_UP) {
            controller->planner->speedForward += 0.1;
            dirty = true;
        }
        if (key == GLFW_KEY_DOWN) {
            controller->planner->speedForward -= 0.1;
            dirty = true;
        }
        if (key == GLFW_KEY_LEFT) {
            controller->planner->turningSpeed += 0.1;
            dirty = true;
        }
        if (key == GLFW_KEY_RIGHT) {
            controller->planner->turningSpeed -= 0.1;
            dirty = true;
        }

        if (dirty) {
            controller->planner->appendPeriodicGaitIfNeeded(
                getPeriodicGait(robot));
            controller->generateMotionTrajectories();

            return true;
        }

        return false;
    }

    virtual void drawImGui() {
        ImGui::SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Once);
        ImGui::Begin("Main Menu");

        ImGui::Text("Robot:");

        static const char *current_item = robotModels[0].name.c_str();

        if (ImGui::BeginCombo("##combo", current_item)) {
            for (uint n = 0; n < robotModels.size(); n++) {
                bool is_selected =
                    (current_item == robotModels[n].name.c_str());
                if (ImGui::Selectable(robotModels[n].name.c_str(),
                                      is_selected)) {
                    current_item = robotModels[n].name.c_str();
                    setupRobotAndController(robotModels[n]);
                }
                if (is_selected) {
                    ImGui::SetItemDefaultFocus();
                }
            }
            ImGui::EndCombo();
        }

        ImGui::Text("Play:");
        ImGui::SameLine();
        PlayPauseButton("Play App", &appIsRunning);

        if (appIsRunning == false) {
            ImGui::SameLine();
            if (ImGui::ArrowButton("tmp", ImGuiDir_Right)) {
                appIsRunning = true;
                process();
                appIsRunning = false;
            }
        }

        ImGui::Text("Slow Motion Mode:");
        ImGui::SameLine();
        ToggleButton("SlowMo", &slowMo);

        ImGui::Checkbox("Follow Robot With Camera", &cameraShouldFollowRobot);

        ImGui::Text("Controller options...");

        ImGui::Text("Locomotion Control options...");

        bool rebuildPlan = false;

        rebuildPlan |= ImGui::InputDouble(
            "Speed forward", &controller->planner->speedForward, 0.1);
        rebuildPlan |= ImGui::InputDouble(
            "Speed sideways", &controller->planner->speedSideways, 0.1);
        rebuildPlan |= ImGui::InputDouble(
            "Turning speed", &controller->planner->turningSpeed, 0.1);

        rebuildPlan |= ImGui::InputDouble(
            "Body Height", &controller->planner->trunkHeight, 0.01);
        rebuildPlan |= ImGui::InputDouble(
            "Swing Foot Height", &controller->planner->targetStepHeight, 0.01);

        bool updateDrawing = false;

        if (ImGui::TreeNode("Draw options...")) {
            updateDrawing |= ImGui::Checkbox("Draw Meshes", &showMeshes);
            updateDrawing |= ImGui::Checkbox("Draw Skeleton", &showSkeleton);
            updateDrawing |= ImGui::Checkbox("Draw Joint Axes", &showJointAxes);
            updateDrawing |=
                ImGui::Checkbox("Draw Joint Limits", &showJointLimits);
            updateDrawing |= ImGui::Checkbox("Draw Collision Primitives",
                                             &showCollisionSpheres);
            updateDrawing |=
                ImGui::Checkbox("Draw End Effectors", &showEndEffectors);
            if (showEndEffectors)
                updateDrawing |= ImGui::SliderFloat(
                    "End Effector R", &eeDrawRadius, 0.01f, 0.1f, "R = %.2f");
            updateDrawing |= ImGui::Checkbox("Draw MOI box", &showMOI);

            ImGui::TreePop();
        }

        if (updateDrawing) {
            updateDrawingOption(robot);
        }

        if (ImGui::TreeNode("Debug options...")) {
            ImGui::Checkbox("Draw debug info", &drawDebugInfo);
            ImGui::TreePop();
        }

        ImGui::End();

        controller->planner->visualizeContactSchedule();
        controller->planner->visualizeParameters();
    }

    virtual bool drop(int count, const char **fileNames) override {
        return true;
    }

private:
    void setupRobotAndController(const RobotModel &model) {
        // kinematic
        if (robot) delete robot;
        if (controller) {
            delete controller->planner;
            delete controller;
        }
        robot =
            new LeggedRobot(model.filePath.c_str(), model.statePath.c_str());
        robot->setRootState(P3D(0, model.baseDropHeight, 0),
                            Quaternion::Identity());
        robot->addLimb("fl", robot->getRBByName(model.fl.c_str()));
        robot->addLimb("hl", robot->getRBByName(model.hl.c_str()));
        robot->addLimb("fr", robot->getRBByName(model.fr.c_str()));
        robot->addLimb("hr", robot->getRBByName(model.hr.c_str()));
        controller = new KinematicTrackingController(
            new SimpleLocomotionTrajectoryPlanner(robot));

        controller->planner->trunkHeight = model.baseTargetHeight;
        controller->planner->targetStepHeight = model.swingFootHeight;
        controller->planner->appendPeriodicGaitIfNeeded(getPeriodicGait(robot));

        controller->generateMotionTrajectories();

        updateDrawingOption(robot);
    }

    PeriodicGait getPeriodicGait(LeggedRobot *robot) {
        PeriodicGait pg;
        double tOffset = -0.0;
        pg.addSwingPhaseForLimb(robot->limbs[0], 0 - tOffset, 0.5 + tOffset);
        pg.addSwingPhaseForLimb(robot->limbs[1], 0.5 - tOffset, 1.0 + tOffset);
        pg.addSwingPhaseForLimb(robot->limbs[2], 0.5 - tOffset, 1.0 + tOffset);
        pg.addSwingPhaseForLimb(robot->limbs[3], 0 - tOffset, 0.5 + tOffset);
        pg.strideDuration = 0.7;
        return pg;
    }

    void updateDrawingOption(LeggedRobot *robot) {
        robot->showMeshes = showMeshes;
        robot->showSkeleton = showSkeleton;
        robot->showJointAxes = showJointAxes;
        robot->showJointLimits = showJointLimits;
        robot->showCollisionSpheres = showCollisionSpheres;
        robot->showEndEffectors = showEndEffectors;
        robot->showMOI = showMOI;

        for (auto rb : robot->rbList) {
            rb->rbProps.endEffectorRadius = eeDrawRadius;
        }
    }

public:
    SimpleGroundModel ground;

    LeggedRobot *robot = nullptr;

    // controllers
    KinematicTrackingController *controller = nullptr;

    double dt = 1 / 30.0;

    bool checkMeshesForMouseHit = true;
    bool testGCRR = false;
    bool drawDebugInfo = true;

    bool appIsRunning = false;
    bool cameraShouldFollowRobot = true;

    bool optMPC = false;

    // UI
    bool slowMo = false;

    bool showMeshes = true;
    bool showSkeleton = false;
    bool showJointAxes = false;
    bool showJointLimits = false;
    bool showCollisionSpheres = false;
    bool showEndEffectors = false;
    bool showMOI = false;
    float eeDrawRadius = 0.01f;
};

}  // namespace locomotion
}  // namespace app
}  // namespace crl