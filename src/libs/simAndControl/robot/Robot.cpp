#include <robot/GeneralizedCoordinatesRobotRepresentation.h>
#include <robot/RBJoint.h>
#include <robot/RBLoader.h>
#include <robot/Robot.h>
#include <robot/RobotState.h>

namespace crl {

Robot::Robot(const char *filePath, const char *statePath, bool loadVisuals) {
    // load robot from rbLoader
    RBLoader rbLoader(filePath, loadVisuals);
    rbLoader.populateRobot(this);

    // set initial state
    // load robot state from rs file or
    // set default state based on joint default angle
    if (statePath && strlen(statePath) > 0) {
        loadReducedStateFromFile(statePath);
    } else {
        RobotState rs(this, true);
        setState(&rs);
    }
}

Robot::~Robot() {
    for (uint i = 0; i < rbList.size(); i++) delete rbList[i];
    rbList.clear();
    for (uint i = 0; i < jointList.size(); i++) delete jointList[i];
    jointList.clear();
}

void Robot::populateState(RobotState *state, bool useDefaultAngles) {
    // we'll push the root's state information - ugly code....
    state->setPosition(root->state.pos);
    state->setOrientation(root->state.orientation);
    state->setHeadingAxis(RBGlobals::worldUp);

    state->setJointCount((int)jointList.size());

    // now each joint introduces one more rigid body, so we'll only record its
    // state relative to its parent. we are assuming here that each joint is
    // revolute!!!

    for (uint i = 0; i < jointList.size(); i++) {
        if (!useDefaultAngles) {
            state->setJointRelativeOrientation(
                getRelativeOrientationForJoint(jointList[i]), i);
        } else {
            state->setJointRelativeOrientation(
                getRotationQuaternion(jointList[i]->defaultJointAngle,
                                      jointList[i]->rotationAxis),
                i);
        }
    }
}

void Robot::setDefaultState() {
    RobotState rs;
    populateState(&rs, true);
    setState(&rs);
}

void Robot::setZeroState() {
    RobotState rs;
    populateState(&rs, true);
    for (uint i = 0; i < jointList.size(); i++) {
        rs.setJointRelativeOrientation(
            getRotationQuaternion(0, jointList[i]->rotationAxis), i);
    }
    setState(&rs);
}

void Robot::setState(RobotState *state) {
    // kinda ugly code....
    root->state.pos = state->getPosition();
    root->state.orientation = state->getOrientation();
    root->state.orientation.normalize();

    // now each joint introduces one more rigid body, so we'll only record its
    // state relative to its parent. we are assuming here that each joint is
    // revolute!!!
    for (uint j = 0; j < jointList.size(); j++) {
        setRelativeOrientationForJoint(
            jointList[j],
            state->getJointRelativeOrientation((int)j).normalized());
        // and now set the linear position and velocity
        jointList[j]->fixJointConstraints(true, true);
    }
}

void Robot::fixJointConstraints() {
    for (size_t j = 0; j < jointList.size(); j++)
        jointList[j]->fixJointConstraints(true, true);
}

P3D Robot::computeCOM() {
    P3D COM = root->state.pos * root->rbProps.mass;
    double totalMass = root->rbProps.mass;

    for (uint i = 0; i < jointList.size(); i++) {
        totalMass += jointList[i]->child->rbProps.mass;
        COM +=
            jointList[i]->child->state.pos * jointList[i]->child->rbProps.mass;
    }

    return COM / totalMass;
}

double Robot::getMass() {
    // compute the mass of the robot
    double mass = root->rbProps.mass;
    for (uint i = 0; i < jointList.size(); i++)
        mass += jointList[i]->child->rbProps.mass;
    return mass;
}

void Robot::setRootState(const P3D &position, const Quaternion &orientation) {
    RobotState state(this);
    populateState(&state);
    state.setPosition(position);
    state.setOrientation(orientation);
    setState(&state);
}

void Robot::loadReducedStateFromFile(const char *fName) {
    RobotState state(this);
    state.readFromFile(fName);
    setState(&state);
}

void Robot::saveReducedStateToFile(const char *fName) {
    RobotState state(this);
    state.writeToFile(fName, this);
}

RB *Robot::getRBByName(const char *jName) {
    for (uint i = 0; i < jointList.size(); i++) {
        if (strcmp(jointList[i]->parent->name.c_str(), jName) == 0)
            return jointList[i]->parent;
        if (strcmp(jointList[i]->child->name.c_str(), jName) == 0)
            return jointList[i]->child;
    }
    std::cout
        << "WARNING: Robot:getRBByName -> rigid body could not be found..."
        << std::endl;
    return nullptr;
}

}  // namespace crl
