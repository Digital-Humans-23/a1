#pragma once

#include <robot/RB.h>
#include <utils/mathUtils.h>

namespace crl {

class EndEffector {
public:
    P3D localCoordinates;
    V3D localFrame[3];  // x- y- z- unit vectors of EE-local coordinate system

    V3D targetFrame[3];  // target direction for (EE-local) x- y- and
                         // z-direction
    P3D targetPosition;

    // end effector pos/orientation are specified on this rigid body...
    RB *rigidBody = nullptr;
    double objectiveWeight = 1.0;

    // if we want to control just the x,y or z component of the end effector,
    // use this mask...
    V3D positionMask = V3D(1.0, 1.0, 1.0);
    V3D orientationMask = V3D(1.0, 1.0, 1.0);

    // in case we have multiple states a planner is considering, this index will
    // tell us which state index this end effector info is associated with
    int sIndex = 0;

public:
    EndEffector() {}
    ~EndEffector() {}

    void setLocalCoordinates(const P3D &ee) { localCoordinates = ee; }

    void setTargetPosition(const P3D &target) {
        targetPosition = target;
    }

    void setLocalOrientation(const Matrix3x3 &localMatrix) {
        localFrame[0] = static_cast<V3D>(localMatrix.col(0));
        localFrame[1] = static_cast<V3D>(localMatrix.col(1));
        localFrame[2] = static_cast<V3D>(localMatrix.col(2));
    }
};

}  // namespace crl
