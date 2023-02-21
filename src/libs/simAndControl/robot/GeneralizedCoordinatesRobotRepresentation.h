#pragma once

#include <robot/RobotState.h>
#include <utils/mathUtils.h>

namespace crl {

/**
 * This class implements a reduced representation of an robot (i.e. we represent
 * the configuration using vectors q and qDot, which represent the generalized
 * coordinates of the robot.
 */

class GeneralizedCoordinatesRobotRepresentation {
private:
    // this is the reference to the robot whose reduced representation is being stored
    Robot *robot = nullptr;

    //--- Reduced-coordinates state of the robot
    //NOTE: maximal coords representation stored in the base robot class needs to be manually sync'ed with the generalized representation
    dVector q;

public:
    /** the constructor */
    GeneralizedCoordinatesRobotRepresentation(Robot *a);

    /** the destructor */
    virtual ~GeneralizedCoordinatesRobotRepresentation(void) {}

    /**
     * returns the total number of degrees of freedom in the reduced representation
     */
    int getDOFCount() const;

    /**
     * returns the index of the generalized coordinate corresponding to this joint
     */
    int getQIdxForJoint(RBJoint *joint) const;

    /**
     * returns the index of the generalized coordinate corresponding to the index of 
     * this joint
     */
    int getQIdxForJointIdx(int jIdx) const;
    /**
    * returns a pointer to the joint corresponding to this generalized coordinate index.
    * If the index corresponds to a root DOF, this method will return NULL.
    */
    RBJoint *getJointForQIdx(int qIdx) const;

    /**
     * returns the axis correponding to the indexed generalized coordinate,
     * expressed in local coordinates.
     */
    V3D getQAxis(int qIndex) const;

    /**
     * returns the relative orientation that a specific q dof induces...
     */
    Quaternion getRelOrientationForQ(int qIndex);

    /**
     * returns the global orientation associated with a specific dof q...
     */
    Quaternion getWorldRotationForQ(int qIndex);

    /**
     * returns the translation or rotation axis for a specific dof q...
     */
    V3D getWorldCoordsAxisForQ(int qIndex);

    /**
     * In the tree-like hierarchy of joints/dofs, this method returns the parent index of the
     * dof corresponding to qIdx
     */
    int getParentQIdxOf(int qIdx);

    /**
     * given the current state of the generalized representation, output the
     * reduced state of the robot
     */
    void getReducedRobotState(RobotState &state);

    /**
     * updates robot state given current q and qDot values...
     */
    void syncRobotStateWithGeneralizedCoordinates();

    /**
     * updates q and qDot given current state of robot...
     */
    void syncGeneralizedCoordinatesWithRobotState();

    /**
     * sets the current q values
     */
    void setQ(const dVector &qNew);

    /**
     * gets the current q values
     */
    void getQ(dVector &q_copy);

    void getQFromReducedState(const RobotState &rs, dVector &q_copy);

    /**
        pLocal is expressed in the coordinate frame of the link that pivots about DOF qIdx.
        This method returns the point in the coordinate frame of the parent of qIdx after
        the DOF rotation has been applied.
    */
    P3D getCoordsInParentQIdxFrameAfterRotation(int qIdx, const P3D &pLocal);

    /**
     * returns the world coordinates for point p, which is specified in the
     * local coordinates of rb (relative to its COM). I.e. p(q)
     */
    P3D getWorldCoordinates(const P3D &p, RB *rb);

    /**
     * returns the world-relative orientation for rb
     */
    Quaternion getOrientationFor(RB *rb);

    /**
     * computes the jacobian dp/dq that tells you how the world coordinates of p
     * change with q. p is expressed in the local coordinates of rb
     */
    void compute_dpdq(const P3D &p, RB *rb, Matrix &dpdq);

    /**
     * estimates the jacobian dp/dq using finite differences
     */
    void estimate_linear_jacobian(const P3D &p, RB *rb, Matrix &dpdq);

    inline int getDimensionSize() { return (int)q.size(); }

    double getQVal(int idx) { return q[idx]; }
};

// use alias GCRR.
typedef GeneralizedCoordinatesRobotRepresentation GCRR;

}  // namespace crl