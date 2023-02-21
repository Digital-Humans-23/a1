#pragma once

#include <robot/RBProperties.h>
#include <robot/RBState.h>
#include <utils/geoms.h>
#include <utils/mathUtils.h>

namespace crl {

// forward deceleration
class RBJoint;

/**
 * Rigidbody class.
 */
class RB {
public:
    // state of the rigid body
    RBState state;
    // a list of properties for the rigid body
    RBProperties rbProps;
    // we will keep a list of the joints that have this rigid body as a parent
    // (child joints).
    std::vector<RBJoint *> cJoints;
    // and the parent joint.
    RBJoint *pJoint = NULL;
    // name of the rigid body
    std::string name;

public:
    /**
     * Default constructor
     */
    RB(void) {}

    /**
     * Default destructor
     */
    virtual ~RB(void) {}

    /**
     *  returns the world coords moment of inertia of the rigid body.
     */
    Matrix3x3 getWorldMOI();

    /**
     *  returns true if it is hit, false otherwise.
     */
    bool getRayIntersectionPoint(const Ray &ray, P3D &intersectionPoint,
                                 bool checkMeshes, bool checkSkeleton);
};

}  // namespace crl
