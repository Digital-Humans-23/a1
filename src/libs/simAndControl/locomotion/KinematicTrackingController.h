#pragma once

#include <locomotion/LocomotionController.h>
#include <locomotion/LocomotionTrajectoryPlanner.h>
#include <robot/RB.h>
#include <robot/RBJoint.h>

namespace crl {

/**
 * A controller that kinematically "tracks" the objectives output by a
 * locomotion trajectory generator
 */
class KinematicTrackingController : public LocomotionController {
public:
    LeggedRobot *robot;
    IK_Solver *ikSolver = nullptr;

public:
    /**
     * constructor
     */
    KinematicTrackingController(LocomotionTrajectoryPlanner *planner)
        : LocomotionController(planner) {
        this->robot = planner->robot;
        ikSolver = new IK_Solver(robot);
    }

    /**
     * destructor
     */
    virtual ~KinematicTrackingController(void) { delete ikSolver; }

    void generateMotionTrajectories(double dt = 1.0 / 30) override {
        planner->planGenerationTime = planner->simTime;
        planner->generateTrajectoriesFromCurrentState(dt);
    }

    void computeAndApplyControlSignals(double dt) override {
        // set base pose. in this assignment, we just assume the base perfectly
        // follow target base trajectory.
        P3D targetPos =
            planner->getTargetTrunkPositionAtTime(planner->getSimTime() + dt);
        Quaternion targetOrientation = planner->getTargetTrunkOrientationAtTime(
            planner->getSimTime() + dt);

        robot->setRootState(targetPos, targetOrientation);

        // now we solve inverse kinematics for each limbs
        for (uint i = 0; i < robot->limbs.size(); i++) {
            P3D target = planner->getTargetLimbEEPositionAtTime(
                robot->limbs[i], planner->getSimTime() + dt);

            ikSolver->addEndEffectorTarget(
                robot->limbs[i]->eeRB, robot->limbs[i]->ee->endEffectorOffset,
                target);
        }

        ikSolver->solve();
    }

    void advanceInTime(double dt) override { planner->advanceInTime(dt); }

    void drawDebugInfo(gui::Shader *shader) override {
        planner->drawTrajectories(shader);
    }

    void plotDebugInfo() override {
        // add plot if you need...
    }
};

}  // namespace crl