#pragma once

#include <gui/renderer.h>
#include <locomotion/FootFallPattern.h>
#include <locomotion/LeggedRobot.h>
#include <robot/RB.h>
#include <robot/RBJoint.h>
#include <robot/RBUtils.h>
#include <utils/trajectory.h>

namespace crl {

/**
 * Trajectory generator interface for locomotion tasks. We will assume a simple
 * structure for the robot, namely a whole lotta legs connected to a trunk.
 * This generator can be querried for high level objectives, such as target
 * position for the feet, position/orientation/velocity for the trunk, etc...
 *
 */

class LocomotionTrajectoryPlanner {
public:
    //this is the "current" time the planner holds
    double simTime = 0;

    //and this is the time when the current set of motions was generated
    double planGenerationTime = 0;

    LeggedRobot* robot = nullptr;
    ContactPlanManager cpm;
    // this is the length (in seconds) of the planning horizon that we are
    // considering here...
    double tPlanningHorizon = 3.0;
    //so that we don't get weird noise/discontinuities at the end of the plan, we'll go a bit longer
    double tPlanningHorizonBuffer = 1.0;

    // high-level targets that tracking objectives will be made of
    double trunkHeight = 0.48;

    double trunkPitch = 0;
    double trunkRoll = 0;
    double trunkYaw = 0;

    double speedForward = 0;
    double speedSideways = 0;
    double turningSpeed = 0.0;

    double stepWidthModifier = 0.7;

    double targetStepHeight = 0.1;

public:
    /**
     * constructor
     */
    LocomotionTrajectoryPlanner(LeggedRobot* bot) { this->robot = bot; }

    virtual ~LocomotionTrajectoryPlanner() {}

    double getSimTime() { return simTime; }

    virtual void advanceInTime(double dt) {
        simTime += dt;
        cpm.tidyUp(simTime - 2.0);
    }

    virtual void appendPeriodicGaitIfNeeded(const PeriodicGait& p) {
        while (cpm.timeUntilEndOfPlanningHorizon(simTime) <
               tPlanningHorizon + tPlanningHorizonBuffer)
            appendPeriodicGait(p);
    }

    virtual void appendPeriodicGait(const PeriodicGait& p) {
        // and add the footfall pattern to the timeline...
        cpm.appendPeriodicGaitToPlanningHorizon(p);
    }

    ContactPhaseInfo getCPInformationFor(const RobotLimb* l, double t) {
        return cpm.getCPInformationFor(l, t);
    }

    void visualizeContactSchedule(double gridStartTime = -1,
                                  double dt_grid = 1 / 30.0,
                                  int nGridSteps = 30) {
        cpm.visualizeContactSchedule(simTime, gridStartTime, dt_grid,
                                     nGridSteps);
    }

    virtual void visualizeParameters() {}

    virtual void generateTrajectoriesFromCurrentState(double dt = 1 / 30.0) = 0;

    virtual void refineCurrentMPCTrajectory() {}

    virtual P3D getTargetLimbEEPositionAtTime(const RobotLimb* l, double t) = 0;

    virtual P3D getTargetTrunkPositionAtTime(double t) = 0;

    virtual double getTargetTrunkHeadingAtTime(double t) = 0;

    virtual RBState getTargetTrunkState(double t) {
        RBState targetState;
        targetState.pos = getTargetTrunkPositionAtTime(t);
        targetState.orientation = getTargetTrunkOrientationAtTime(t);
        return targetState;
    }

    virtual Quaternion getTargetTrunkOrientationAtTime(double t) {
        return getRotationQuaternion(getTargetTrunkHeadingAtTime(t),
                                     V3D(0, 1, 0)) *
               getRotationQuaternion(trunkPitch,
                                     RBGlobals::worldUp.cross(robot->forward)) *
               getRotationQuaternion(trunkRoll, robot->forward);
    }

    virtual V3D getTargetLimbEEVelocityAtTime(const RobotLimb* l, double t,
                                              double dt = 1 / 30.0) {
        P3D delta = getTargetLimbEEPositionAtTime(l, t + dt) -
                    getTargetLimbEEPositionAtTime(l, t);
        return V3D(delta) / dt;
    }

    virtual V3D getTargetTrunkVelocityAtTime(double t, double dt = 1 / 30.0) {
        P3D delta = getTargetTrunkPositionAtTime(t + dt) -
                    getTargetTrunkPositionAtTime(t);
        return V3D(delta) / dt;
    }

    virtual V3D getTargetTrunkAngularVelocityAtTime(double t,
                                                    double dt = 1 / 30.0) {
        return estimateAngularVelocity(getTargetTrunkOrientationAtTime(t),
                                       getTargetTrunkOrientationAtTime(t + dt),
                                       dt);
    }

    virtual void drawTrajectories(gui::Shader* shader) {}
};

}  // namespace crl
