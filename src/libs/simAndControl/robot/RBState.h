#pragma once

#include <utils/mathUtils.h>

namespace crl {

/**
 * This class acts as a container for the state information (position,
 * orientation, velocity and angular velocity - all of them stored in world
 * coordinates) of a rigid body.
 */

class RBState {
public:
    // the position of the center of mass of the rigid body, in world coords. In
    // local coordinates this corresponds to the point (0,0,0) aka origin.
    P3D pos = P3D(0, 0, 0);
    // its orientation - rotates from local coordinate frame to world coordinate
    // frame
    Quaternion orientation = Quaternion::Identity();

public:
    /**
     * Default constructor - populate the data members using safe values..
     */
    RBState(void) {}

    /**
     * A copy constructor.
     */
    RBState(const RBState &other) {
        this->pos = other.pos;
        this->orientation = other.orientation;
    }

    /**
     * A copy operator
     */
    RBState &operator=(const RBState &other) {
        this->pos = other.pos;
        this->orientation = other.orientation;
        return *this;
    }

    bool operator==(const RBState &other) const {
        if (V3D(pos, other.pos).norm() > 1e-10) return false;
        //q and -q represent the same rotation...
        if (!sameRotation(orientation, other.orientation)) return false;
            return false;
        return true;
    }

    /**
     * Default destructor.
     */
    ~RBState(void) {}

    /**
     * This method returns the coordinates of the point that is passed in as a
     * parameter(expressed in local coordinates), in world coordinates.
     */
    inline P3D getWorldCoordinates(const P3D &pLocal) const {
        // pWorld = pos + R * V3D(origin, pLocal)
        return pos + orientation * V3D(pLocal);
    }

    /**
     * This method returns the vector that is passed in as a parameter(expressed
     * in local coordinates), in world coordinates.
     */
    inline V3D getWorldCoordinates(const V3D &vLocal) const {
        // the rigid body's orientation is a unit quaternion. Using this, we can
        // obtain the global coordinates of a local vector
        return orientation * vLocal;
    }

    /**
     * This method is used to return the local coordinates of the point that is
     * passed in as a parameter (expressed in global coordinates)
     */
    inline P3D getLocalCoordinates(const P3D &pWorld) {
        return P3D() + orientation.inverse() * (V3D(pos, pWorld));
    }

    /**
     * This method is used to return the local coordinates of the vector that is
     * passed in as a parameter (expressed in global coordinates)
     */
    inline V3D getLocalCoordinates(const V3D &vWorld) {
        // the rigid body's orientation is a unit quaternion. Using this, we can
        // obtain the global coordinates of a local vector
        return orientation.inverse() * vWorld;
    }
};

}  // namespace crl
