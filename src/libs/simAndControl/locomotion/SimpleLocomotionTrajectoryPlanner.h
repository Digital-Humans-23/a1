#pragma once

#include <gui/renderer.h>
#include <locomotion/BodyFrame.h>
#include <locomotion/FootFallPattern.h>
#include <locomotion/LocomotionPlannerHelpers.h>
#include <locomotion/LocomotionTrajectoryPlanner.h>
#include <robot/RB.h>
#include <robot/RBJoint.h>
#include <robot/RBUtils.h>
#include <utils/trajectory.h>

namespace crl {

/**
 * A basic trajectory generator for locomotion tasks. We will assume a simple
 * structure for the robot, namely a whole lotta legs connected to a trunk.
 * This generator can be querried for high level objectives, such as target
 * position for the feet, position/orientation/velocity for the trunk, etc...
 *
 * This simplest of models assumes the trunk motion follows the motion of the body frame
 *
 * Note: it is convenient to specify some quantities in a coordinate frame that is
 * located at the CoM of the trunk and shares its heading, but otherwise ignores
 * pitch, roll, parasitic components of its motion (e.g. periodic, transient or
 * accidental fluctuations in yaw/tangential location of COM that should not be
 * affecting the robot's COM position or heading). We call this the body frame.
 *
 */

class SimpleLocomotionTrajectoryPlanner : public LocomotionTrajectoryPlanner {
protected:
    //store cartesian trajectories for each foot
    std::map<const RobotLimb*, Trajectory3D> limbTrajectories;

    //store reference trajectory for the robot's body frame
    bFrameReferenceMotionPlan bFrameMotionPlan;
    LimbMotionProperties lmProps;
    FootstepPlan fsp;

    //assumption is that walking happens on flat ground here
    double groundHeight = 0;

public:
    /**
         * constructor
         */
    SimpleLocomotionTrajectoryPlanner(LeggedRobot* bot)
        : LocomotionTrajectoryPlanner(bot), bFrameMotionPlan(bot) {
        generateTrajectoriesFromCurrentState();
    }

    void initializeMotionPlan(double dt) {
        //set properties/targets needed to generate body frame motion trajectory
        bFrameMotionPlan.dt = dt;
        bFrameMotionPlan.targetbFrameHeight = groundHeight + trunkHeight;
        bFrameMotionPlan.targetForwardSpeed = speedForward;
        bFrameMotionPlan.targetSidewaysSpeed = speedSideways;
        bFrameMotionPlan.targetTurngingSpeed = turningSpeed;
        bFrameMotionPlan.tStart = simTime;
        bFrameMotionPlan.tEnd =
            simTime + tPlanningHorizon + tPlanningHorizonBuffer;

        lmProps.stepWidthOffsetX = stepWidthModifier;
        lmProps.swingFootHeight = targetStepHeight;
    }

    void generateBFrameTrajectory() {
        //now generate the motion of the body frame - and do account for the differences between the planned motion of the trunk and the reference motion for the body frame
        bFrameMotionPlan.generateTrajectory();
    }

    void generateSteppingLocations() {
        //and the contact locations for the limbs
        bFrameMotionPlan.populateFootstepPlan(fsp, lmProps, &cpm, groundHeight);
    }

    void generateLimbTrajectories(double dt) {
        //and full motion trajectories for each limb
        for (uint i = 0; i < robot->limbs.size(); i++) {
            limbTrajectories[robot->limbs[i]] = fsp.generateLimbTrajectory(
                robot->limbs[i], lmProps, simTime, simTime + tPlanningHorizon,
                dt, groundHeight);
        }
    }

    virtual void generateTrajectoriesFromCurrentState(double dt = 1 / 30.0) {
        initializeMotionPlan(dt);

        generateBFrameTrajectory();

        generateSteppingLocations();

        generateLimbTrajectories(dt);
    }

    virtual P3D getTargetLimbEEPositionAtTime(const RobotLimb* l, double t) {
        return P3D() + limbTrajectories[l].evaluate_linear(t);
    }

    virtual P3D getTargetTrunkPositionAtTime(double t) {
        return P3D() +
               bFrameMotionPlan.bFramePosTrajectory.evaluate_linear(
                   t);  // +RBGlobals::worldUp.cross(robot->forward) * 0.05 * sin(5 * t);
    }

    virtual double getTargetTrunkHeadingAtTime(double t) {
        return bFrameMotionPlan.bFrameHeadingTrajectory.evaluate_linear(t) +
               trunkYaw;
    }

    virtual Quaternion getTargetTrunkOrientationAtTime(double t) {
        return getRotationQuaternion(getTargetTrunkHeadingAtTime(t),
                                     V3D(0, 1, 0)) *
               getRotationQuaternion(trunkPitch,
                                     RBGlobals::worldUp.cross(robot->forward)) *
               getRotationQuaternion(trunkRoll, robot->forward);
    }

    virtual void drawTrajectories(gui::Shader* shader) {
        for (int i = 0; i < bFrameMotionPlan.bFramePosTrajectory.getKnotCount();
             i++)
            drawSphere(
                P3D() + bFrameMotionPlan.bFramePosTrajectory.getKnotValue(i),
                0.02, *shader);

        for (uint i = 0; i < robot->limbs.size(); i++)
            for (int j = 0;
                 j < limbTrajectories[robot->limbs[i]].getKnotCount(); j++)
                drawSphere(
                    P3D() + limbTrajectories[robot->limbs[i]].getKnotValue(j),
                    0.01, *shader, V3D(1, 1, 0));
    }
};

}  // namespace crl
