#pragma once

#include <utils/utils.h>

#include "robot/RBJoint.h"
#include "robot/RBRenderer.h"
#include "robot/RBUtils.h"
#include "robot/RB.h"

namespace crl {

class RobotState;

/**
 * Robots are articulated figures (i.e. tree structures starting at a root)
 */

class Robot {
    friend class RobotState;
    friend class GeneralizedCoordinatesRobotRepresentation;

public:
    // root configuration
    RB *root = nullptr;
    // keep lists of all the joints and all the RBs of the robot, for easy
    // access
    std::vector<RBJoint *> jointList;
    std::vector<RB *> rbList;

    //useful to know which way is "forward" for this robot.
    V3D forward = V3D(0, 0, 1);

    // drawing flags
    bool showMeshes = true;
    bool showJointLimits = false;
    bool showCollisionSpheres = false;
    bool showEndEffectors = false;
    bool showJointAxes = false;
    bool showSkeleton = false;
    bool showMOI = false;
    bool showCoordFrame = false;
    bool showJointAngles = false;

public:
    /** the constructor */
    Robot(const char *filePath, const char *statePath = nullptr,
          bool loadVisuals = true);

    /** the destructor */
    virtual ~Robot(void);

    /**
     * This method computes the relative orientation of the parent and child
     * bodies of joint i.
     */
    inline Quaternion getRelativeOrientationForJoint(RBJoint *joint) {
        return joint->computeRelativeOrientation();
    }

    inline void setRelativeOrientationForJoint(RBJoint *joint,
                                               const Quaternion &qRel) {
        joint->child->state.orientation =
            joint->parent->state.orientation * qRel;
    }

    inline int getJointCount() { return (int)jointList.size(); }

    inline RBJoint *getJoint(int i) const {
        if (i < 0 || i > (int)jointList.size() - 1) return nullptr;
        return jointList[i];
    }

    inline int getRigidBodyCount() { return (int)jointList.size() + 1; }

    /**
     * returns a pointer to the ith rigid body of the virtual robot, where the
     * root is at 0, and the rest follow afterwards...
     */
    inline RB *getRigidBody(int i) {
        if (i == 0) return root;
        if (i <= (int)jointList.size()) return jointList[i - 1]->child;
        return nullptr;
    }

    /**
     * this method is used to return the current heading of the robot
     */
    inline Quaternion getHeading() {
        return computeHeading(root->state.orientation, RBGlobals::worldUp);
    }

    /**
     * this method is used to return the current heading of the robot, specified
     * as an angle measured in radians
     */
    inline double getHeadingAngle() {
        return getRotationAngle(
            computeHeading(root->state.orientation, RBGlobals::worldUp),
            RBGlobals::worldUp);
    }

    /**
     * this method is used to return a reference to the joint whose name is
     * passed as a parameter, or nullptr if it is not found.
     */
    inline RBJoint *getJointByName(const char *jName) {
        for (uint i = 0; i < jointList.size(); i++)
            if (strcmp(jointList[i]->name.c_str(), jName) == 0)
                return jointList[i];
        return nullptr;
    }

    /**
     * this method is used to return the index of the joint (whose name is
     * passed as a parameter) in the articulated figure hierarchy.
     */
    inline int getJointIndex(const char *jName) {
        for (uint i = 0; i < jointList.size(); i++)
            if (strcmp(jointList[i]->name.c_str(), jName) == 0) return i;
        return -1;
    }

    /**
     * returns the root of the current articulated figure.
     */
    inline RB *getRoot() { return root; }

    /**
     * This method is used to compute the center of mass of the robot.
     */
    P3D computeCOM();

    double getMass();

    /**
     * this method is used to read the reduced state of the robot from the file
     */
    void loadReducedStateFromFile(const char *fName);

    /**
     * this method is used to write the reduced state of the robot to a file
     */
    void saveReducedStateToFile(const char *fName);

    /**
     * uses the state of the robot to populate the input
     */
    void populateState(RobotState *state, bool useDefaultAngles = false);

    /**
     * sets the state of the robot using the input
     */
    void setState(RobotState *state);

    /**
     * sets the state of the robot using default joint configurations
     */
    void setDefaultState();

    /**
     * sets the state of the robot using all-zero joint configurations
     */
    void setZeroState();

    /**
     * makes sure the state of the robot is consistent with all the joint
     * types...
     */
    void fixJointConstraints();

    /**
     * this method updates the robot's root state and all connected rigid bodies
     */
    void setRootState(const P3D &position = P3D(0, 0, 0),
                      const Quaternion &orientation = Quaternion::Identity());

    /**
     * this method is used to return a reference to the articulated figure's
     * rigid body whose name is passed in as a parameter, or nullptr if it is
     * not found.
     */
    RB *getRBByName(const char *jName);

    /**
     * returns NULL if no RBs are hit by the ray...
     */
    RB *getFirstRBHitByRay(const Ray &ray, P3D &intersectionPoint,
                                bool checkMeshes, bool checkSkeleton) {
        RB *selectedRB = nullptr;
        double t = DBL_MAX;
        P3D tmpIntersectionPoint = P3D(0, 0, 0);

        for (uint i = 0; i < rbList.size(); i++) {
            if (rbList[i]->getRayIntersectionPoint(
                    ray, tmpIntersectionPoint, checkMeshes, checkSkeleton)) {
                double tTmp = ray.getRayParameterFor(tmpIntersectionPoint);
                if (tTmp < t) {
                    selectedRB = rbList[i];
                    t = tTmp;
                    intersectionPoint = tmpIntersectionPoint;
                }
            }
        }

        return selectedRB;
    }

    /**
     * draws the robot at its current state
     */
    inline void draw(const gui::Shader &rbShader) {
        // Draw abstract view first
        if (showSkeleton)
            for (uint i = 0; i < rbList.size(); i++)
                RBRenderer::drawSkeletonView(rbList[i], rbShader, showJointAxes,
                                             showJointLimits, showJointAngles);

        // Then draw meshes (because of blending)
        if (showMeshes)
            for (uint i = 0; i < rbList.size(); i++)
                RBRenderer::drawMeshes(rbList[i], rbShader);

        // Then draw collsion spheres
        if (showCollisionSpheres)
            for (uint i = 0; i < rbList.size(); i++)
                RBRenderer::drawCollisionSpheres(rbList[i], rbShader);

        // Then draw end effectors
        if (showEndEffectors)
            for (uint i = 0; i < rbList.size(); i++)
                RBRenderer::drawEndEffectors(rbList[i], rbShader);

        // and now MOIs
        if (showMOI)
            for (uint i = 0; i < rbList.size(); i++)
                RBRenderer::drawMOI(rbList[i], rbShader);

        // and now coordinate frames
        if (showCoordFrame) {
            for (uint i = 0; i < rbList.size(); i++)
                RBRenderer::drawCoordFrame(rbList[i], rbShader);
        }
    }
};

}  // namespace crl
