#pragma once

#include <utils/geoms.h>
#include <utils/mathUtils.h>

namespace crl {

enum RB_KEYWORDS {
    RB_NOT_IMPORTANT = -1,
    RB_RB,
    RB_END_RB,
    RB_NAME,
    RB_MASS,
    RB_MOI,
    RB_IS_FROZEN,
    RB_COLLISION_SPHERE,
    RB_COLLISION_PLANE,
    RB_MESH_NAME,
    RB_MESH_COLOR,
    RB_MESH_TRANSFORMATION,
    RB_MESH_DESCRIPTION,
    RB_MATCAP_TEXTURE,
    RB_CHILD,
    RB_PARENT,
    RB_PPOS,
    RB_CPOS,
    RB_END_EFFECTOR,
    RB_JOINT,
    RB_JOINT_LIMITS,
    RB_JOINT_AXIS,
    RB_DEFAULT_ANGLE,
    RB_JOINT_MOTOR_KP,
    RB_JOINT_MOTOR_KD,
    RB_JOINT_END,
    RB_FRICTION_COEFF,
    RB_REST_COEFF
};

/**
 * determine the type of a line that was used in the input file for a rigid body
 */
int getRRBLineType(char *&buffer);

/**
 * returns the string associated with the given token
 */
char *getRRBString(int token);

class RBGlobals {
public:
    // gravitational acceleration
    static double g;
    // this is the direction of the up-vector
    static V3D worldUp;
    // and the ground plane
    static Plane groundPlane;
};

/**
 * Assume that the current quaternion represents the relative orientation
 * between two coordinate frames P and C (i.e. q rotates vectors from the
 * child/local frame C into the parent/global frame P).
 *
 * With v specified in frame C's coordinates, this method decomposes the current
 * relative rotation, such that:
 *
 *      PqC = qA * qB, where qB represents a rotation about axis v.
 *
 * This can be thought of as a decomposition into a twist about axis v (qB), and
 * a swing (qA). Note that qB is a rotation from the C frame into a tmp,
 * twist-free frame T, and qA a rotation from T into P. In the T coordinate
 * frame, v is the same as in C, and qA is a rotation that aligns v from P to
 * that from T.
 */
inline void decomposeRotation(const Quaternion &PqC, Quaternion &Pq,
                              Quaternion &qC, const V3D &v) {
    // we need to compute v in P's coordinates
    V3D vP = PqC * v;
    // compute the rotation that alligns the vector v in the two coordinate
    // frames (P and T - remember that v has the same coordinates in C and in T)
    V3D rotAxis = vP.cross(v);
    rotAxis.normalize();
    double rotAngle = -safeACOS(vP.dot(v));

    Pq = getRotationQuaternion(rotAngle, rotAxis);
    // now qB = qAinv * PqC
    qC = Pq.inverse() * PqC;
}

/**
 * Same as above. This method directly returns the twist rotation
 */
inline Quaternion decomposeRotation(const Quaternion &PqC, const V3D &v) {
    Quaternion Pq = Quaternion::Identity(), qC = Quaternion::Identity();
    decomposeRotation(PqC, Pq, qC, v);
    return qC;
}

/**
 * decomposes the current quaternion as follows: *this = qHeading * qOther,
 * where qHeading is a rotation about the axis passed in as a parameter.
 */
inline Quaternion computeHeading(const Quaternion &q, const V3D &upAxis) {
    return decomposeRotation(q.inverse(), upAxis).inverse();
}

/**
    assumption here is that the y-axis gives the yaw direction
*/
/*inline double getHeadingAngle(const Quaternion& q, const V3D& upAxis = V3D(0, 1, 0)) {
    // same for heading angles
    return getRotationAngle(computeHeading(q, upAxis), upAxis);
}*/

/**
    assumption here is that the y-axis gives the yaw direction
*/
inline double getHeadingAngle(const Quaternion& q) {
    // same for heading angles
    double pitch = 0, roll = 0, yaw = 0;
    computeEulerAnglesFromQuaternion(q, V3D(1, 0, 0),
        V3D(0, 0, 1), V3D(0, 1, 0), roll,
        pitch, yaw);
    return yaw;
}

/**
 * compute the rotation angle about axis a such as to most closely match the
 * orientation in q
 */
inline void computeRotationAngleFromQuaternion(const Quaternion &q,
                                               const V3D &a, double &alpha) {
    //	alpha = getRotationAngle(decomposeRotation(q, a), a);

    V3D v1, v2;
    double tmp1, tmp2;
    getVectorsOrthogonalTo(a, v1, v2);
    computeEulerAnglesFromQuaternion(q, a, v1, v2, alpha, tmp1, tmp2);
}


/**
 * decompose the quaternion q as: q = R(b, beta) * R(a, alpha). Unknowns are:
 * alpha and beta. In general, there will be some residual
 */
inline void computeEulerAnglesFromQuaternion(const Quaternion &q, const V3D &a,
                                             const V3D &b, double &alpha,
                                             double &beta) {
    // we assume the quaternion corresponds to a rotation about the two axes
    // only... therefore the axis a gets rotated about b by angle beta...
    assert(IS_ZERO(a.dot(b)));
    assert(IS_ZERO(a.norm() - 1) && IS_ZERO(b.norm() - 1));

    Quaternion qA = Quaternion::Identity(), qB = Quaternion::Identity();
    decomposeRotation(q, qB, qA, a);
    alpha = getRotationAngle(qA, a);

    computeRotationAngleFromQuaternion(qB, b, beta);

    //	Quaternion residual = (Quaternion::getRotationQuaternion(beta, b) *
    // Quaternion::getRotationQuaternion(alpha, a) * q.getComplexConjugate());
    //	Logger::printStatic("residual: %lf (%lf %lf %lf %lf) --> %lf %lf\n",
    // residual.v.length(), residual.s, residual.v.x, residual.v.y,
    // residual.v.z, alpha, beta); 	assert(
    // IS_ZERO((Quaternion::getRotationQuaternion(beta, b)
    //* Quaternion::getRotationQuaternion(alpha, a) *
    // q.getComplexConjugate()).v.length()));
}

/**
 * returns the orientation obtained by applying a constant-angular-velocity
 * rotation starting from q
 */
inline Quaternion updateRotationGivenAngularVelocity(const Quaternion &q,
                                                     const V3D &angularVelocity,
                                                     double dt) {
    // q_p = rot(w, dt) * q
    double angularVelocityMagnitude = angularVelocity.norm();
    // avoid divide by close to zero...
    if (angularVelocityMagnitude > 1e-10) {
        // compute the rotation that would have happened during dt, and add it
        // (pre) to the current orientation
        double netRotationAngle = angularVelocityMagnitude * dt;
        V3D rotationAxis = angularVelocity.normalized();
        return getRotationQuaternion(netRotationAngle, rotationAxis) * q;
    }
    return q;
}

/**
 * returns the angular velocity that explains how we got from qStart to qEnd in
 * dt time (qEnd = qDueToAngVelOverDT * qStart)
 */
inline V3D estimateAngularVelocity(const Quaternion &qStart,
                                   const Quaternion &qEnd, double dt) {
    // qEnd = rot(w_p, dt) * qStart
    Quaternion qRot = qEnd * qStart.inverse();

    V3D rotAxis = Vector3d(qRot.vec());
    if (rotAxis.norm() < 1e-10) return V3D(0, 0, 0);
    rotAxis.normalize();
    double rotAngle = getRotationAngle(qRot, rotAxis);

    // this rotation angle is the result of applying the angular velocity for
    // some time dt...
    return rotAxis * rotAngle / dt;
}

/**
 * tests decompose rotation methods. In conjunction with the tests done for
 * decomposition along 3 euler angles, all is good for the three
 * computeEulerAnglesFromQuaternion and the two decomposeRotation methods...
 */
inline void testQuaternionDecomposition() {
    for (int i = 0; i < 100; i++) {
        V3D v = getRandomUnitVector();
        V3D a, b;
        getVectorsOrthogonalTo(v, a, b);
        double aVal = getRandomNumberInRange(-PI * 0.98, PI * 0.98);
        double bVal = getRandomNumberInRange(-PI * 0.98, PI * 0.98);
        double aValTest = 0, bValTest = 0;
        Quaternion qTest =
            getRotationQuaternion(bVal, b) * getRotationQuaternion(aVal, a);
        computeEulerAnglesFromQuaternion(qTest, a, b, aValTest, bValTest);
        Quaternion qErr = getRotationQuaternion(bValTest, b) *
                          getRotationQuaternion(aValTest, a) * qTest.inverse();
        assert(qErr.vec().norm() < 0.000001);
        assert(IS_EQUAL(aVal, aValTest) && IS_EQUAL(bVal, bValTest));
    }
}

}  // namespace crl