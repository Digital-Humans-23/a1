#pragma once

#include <robot/Robot.h>
#include <utils/mathUtils.h>
#include <utils/utils.h>

namespace crl {

class JointState {
public:
    Quaternion qRel = Quaternion::Identity();
};

class RobotState {
private:
    Quaternion rootQ = Quaternion::Identity();
    P3D rootPos = P3D(0, 0, 0);
    
    // to compute headings, we need to know which axis defines it (the yaw)
    V3D headingAxis = RBGlobals::worldUp;

    std::vector<JointState> joints;

public:
    ~RobotState() {}

    inline RobotState(const RobotState &other) {
        rootQ = other.rootQ;
        rootPos = other.rootPos;
        headingAxis = other.headingAxis;
        joints = other.joints;
    }

    inline RobotState(int jCount = 0, int aJCount = 0) {
        joints.resize(jCount);
    }

    inline RobotState(Robot *robot, bool useDefaultAngles = false) {
        robot->populateState(this, useDefaultAngles);
    }

    inline void setJointCount(int jCount) { joints.resize(jCount); }

    inline void setHeadingAxis(V3D v) { headingAxis = v; }

    inline V3D getHeadingAxis() { return headingAxis; }

    inline int getJointCount() const { return (int)joints.size(); }

    inline P3D getPosition() const { return rootPos; }

    inline void setPosition(P3D p) { rootPos = p; }

    inline Quaternion getOrientation() const { return rootQ; }

    inline void setOrientation(Quaternion q) { rootQ = q; }

    inline Quaternion getJointRelativeOrientation(int jIndex) const {
        if ((uint)jIndex < joints.size()) return joints[jIndex].qRel;
        //	exit(0);
        return Quaternion::Identity();
    }

    inline void setJointRelativeOrientation(const Quaternion &q, int jIndex) {
        if ((uint)jIndex < joints.size()) joints[jIndex].qRel = q;
        //	else
        //		exit(0);
    }

    inline double getHeading() {
        // first we need to get the current heading of the robot.
        return getRotationAngle(computeHeading(getOrientation(), headingAxis),
                                headingAxis);
    }

    bool operator==(const RobotState &other) {
        if (getJointCount() != other.getJointCount()) {
            //			Logger::consolePrint("jCount: %d vs %d\n",
            // getJointCount(), other.getJointCount());
            return false;
        }

        if (V3D(getPosition(), other.getPosition()).norm() > 1e-10) {
            //			Logger::consolePrint("pos: %lf %lf %lf vs %lf
            //%lf %lf\n", 				getPosition().x(),
            // getPosition().y(), getPosition().z(),
            // other.getPosition().x(), other.getPosition().y(),
            // other.getPosition().z());
            return false;
        }

        Quaternion q1 = getOrientation();
        Quaternion q2 = other.getOrientation();

        if (!sameRotation(q1, q2)) {
            //	Logger::consolePrint("orientation: %lf %lf %lf %lf vs %lf %lf
            //%lf %lf\n", q1.s, q1.v.x(), q1.v.y(), q1.v.z(), q2.s, q2.v.x(),
            // q2.v.y(), q2.v.z());
            return false;
        }

        for (int i = 0; i < getJointCount(); i++) {
            Quaternion q1 = getJointRelativeOrientation(i);
            Quaternion q2 = other.getJointRelativeOrientation(i);

            if (!sameRotation(q1, q2)) {
                //				Logger::consolePrint("joint %d
                // orientation: %lf %lf %lf %lf vs %lf %lf %lf %lf\n", i, q1.s,
                // q1.v.x(), q1.v.y(), q1.v.z(), q2.s, q2.v.x(), q2.v.y(),
                // q2.v.z());
                return false;
            }
        }

        return true;
    }

    void writeToFile(const char *fName, Robot *robot = nullptr);
    void readFromFile(const char *fName);
    void setHeading(double heading);
};

}  // namespace crl
