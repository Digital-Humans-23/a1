#pragma once

#include <robot/Robot.h>
#include <robot/RobotState.h>

namespace crl {

/**
 * This class represents a generic limb (i.e. leg or arm). Each limb has an
 * origin RB, a bunch of links connected by joints, and an end effector RB.
 */
class RobotLimb {
public:
    // this is the name of the limb
    std::string name;
    // and all limbs have an end effector
    RB *eeRB = nullptr;
    // and this is a list of all the limb's joints - for easy access...
    DynamicArray<RBJoint *> jointList;

    // this is the vector from the CoM of the trunk to the end effector; it
    // corresponds to a default step offset. Expressed in the coordinate frame
    // of the trunk...
    V3D defaultEEOffset;
    // this is ptr to ee we use for locomotion related tasks
    RBEndEffector *ee = nullptr;

public:
    /**
     * constructor: looking for EE
     */
    RobotLimb(std::string name, RB *eeRB, RB *limbRoot) {
        this->name = name;
        this->eeRB = eeRB;
        this->ee = &eeRB->rbProps.endEffectorPoints[0];  

        RB *tmpRB = eeRB;
        while (tmpRB != limbRoot) {
            jointList.insert(jointList.begin(), tmpRB->pJoint);
            tmpRB = tmpRB->pJoint->parent;
        }

        P3D eePos = ee->endEffectorOffset;
        defaultEEOffset = limbRoot->state.getLocalCoordinates(
            V3D(limbRoot->state.pos, eeRB->state.getWorldCoordinates(eePos)));
    }

    /**
     * this corresponds to the hip or shoulder joint...
     */
    RBJoint *getJointToTrunk() { return jointList[0]; }

    P3D getEEWorldPos() {
        return eeRB->state.getWorldCoordinates(this->ee->endEffectorOffset);
    }

    /**
     * destructor
     */
    virtual ~RobotLimb(void) {}
};

/**
 * Each legged robot has a set of limbs and a trunk.
 */
class LeggedRobot : public Robot {
public:
    // we will assume that the trunk is just one rigid body for now, and that it
    // is the root of the robot...
    RB *trunk = nullptr;
    DynamicArray<RobotLimb *> limbs;

    // it's useful to store standing pose as a nominal state
    // if it is not specified then just set initial state of robot (but it might
    // be dangerous...)
    RobotState standingState;

    LeggedRobot(const char *filePath, const char *statePath = nullptr,
                bool loadVisuals = true)
        : Robot(filePath, statePath, loadVisuals), standingState(this) {
        this->trunk = root;
    }

    virtual ~LeggedRobot() {
        for (uint i = 0; i < limbs.size(); i++) delete limbs[i];
        limbs.clear();
    }

    void addLimb(std::string name, RB *eeRB) {
        limbs.push_back(new RobotLimb(name, eeRB, trunk));
    }

    /**
     * Search the limb corresponding to the queried name
     */
    RobotLimb *getLimbByName(const std::string &name) const {
        for (auto limbPtr : limbs) {
            if (limbPtr->name == name) {
                return limbPtr;
            }
        }
        // No limb matched the name, so return a null pointer
        return nullptr;
    }
};

}  // namespace crl
