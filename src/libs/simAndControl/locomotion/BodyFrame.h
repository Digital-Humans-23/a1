#pragma once

#include "robot/GeneralizedCoordinatesRobotRepresentation.h"
#include "robot/Robot.h"

namespace crl {

// A body frame is defined only through the position and heading (rotation about
// the vertical axis), as well as their rates of change
class BodyFrame {
public:
    // this is the position, in world, of the body frame
    P3D p = P3D(0, 0, 0);
    // this is the heading, indicating the direction in which the robot is
    // facing
    double h = 0;

    BodyFrame(GCRR* rr) {
        this->p = P3D(rr->getQVal(0), rr->getQVal(1), rr->getQVal(2));
        this->h = rr->getQVal(3);
    }

    BodyFrame(const P3D& p, const Quaternion& q) {
        this->p = p;
        this->h = getRotationAngle(computeHeading(q, RBGlobals::worldUp),
                                   RBGlobals::worldUp);
    }

    BodyFrame(const P3D& p, double h) {
        this->p = p;
        this->h = h;
    }

    // the resulting vector points from the origin of the body frame to pW. Both
    // pW and the result are in world coordinates
    V3D getOffsetVectorToPoint(const P3D& pW) const { return V3D(p, pW); }

    //orientation/heading takes vectors from local coords of the body frame to world coords
    P3D getLocalCoordinatesFor(const P3D& pW) const {
        return P3D() + getRotationQuaternion(-h, RBGlobals::worldUp) *
                           getOffsetVectorToPoint(pW);
    }

    //orientation/heading takes vectors from local coords of the body frame to world coords
    V3D getLocalCoordinatesFor(const V3D& vW) const {
        return getRotationQuaternion(-h, RBGlobals::worldUp) * vW;
    }

    P3D getWorldCoordinatesFor(const P3D& pL) const {
        return p + getRotationQuaternion(h, RBGlobals::worldUp) *
                       V3D(P3D(0, 0, 0), pL);
    }

    V3D getWorldCoordinatesFor(const V3D& vL) {
        return getRotationQuaternion(h, RBGlobals::worldUp) * vL;
    }
};
}  // namespace crl
