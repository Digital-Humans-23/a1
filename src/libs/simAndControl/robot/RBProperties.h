#pragma once

#include <gui/model.h>
#include <utils/mathUtils.h>

#include <memory>

namespace crl {

/**
 * This class holds gui model (mesh)
 */
class RB3DModel {
public:
    gui::Model *model = NULL;

    RigidTransformation transform;
    std::string description;
    std::string path;
    V3D color = V3D(0.9, 0.9, 0.9);
};

/**
 * Abstract class for collision primitives.
 */
class RRBCollsionShape {
public:
    virtual ~RRBCollsionShape() = 0;
};

/**
 * Collision primitive shaped a sphere.
 */
class RRBCollisionSphere : public RRBCollsionShape {
public:
    RRBCollisionSphere() : radius(0.01), localCoordinates(P3D(0, 0, 0)) {}
    RRBCollisionSphere(P3D localCoordinates, double radius)
        : radius(radius), localCoordinates(localCoordinates) {}
    ~RRBCollisionSphere() {}

public:
    double radius;
    P3D localCoordinates;
};

/**
 * Collision primitive shaped 2D plane.
 */
class RRBCollisionPlane : public RRBCollsionShape {
public:
    RRBCollisionPlane() : p(P3D(0, 0, 0)), n(0, 1, 0) {}
    RRBCollisionPlane(P3D p, V3D n) : p(p), n(n) {}
    ~RRBCollisionPlane() {}

public:
    P3D p = P3D(0, 0, 0);
    V3D n = V3D(0, 1, 0);
};

/**
 * End effector point. It's assumed to be a sphere with a certain radius.
 */
class RBEndEffector {
public:
    std::string name;
    double radius = 0.01;
    // center of ee in local frame
    P3D endEffectorOffset = P3D(0, 0, 0);
    // with ground, obstacle etc.
    bool inContact = false;
};

/**
 * This class represents a container for the various properties of a rigid body
 */
class RBProperties {
public:
    // the mass
    double mass = 1.0;
    // we'll store the moment of inertia of the rigid body, in the local
    // coordinate frame
    Matrix3x3 MOI_local = Matrix3x3::Identity();

    // collision primitives that are relevant for this rigid body
    std::vector<std::shared_ptr<RRBCollsionShape>> collisionShapes;

    // meshes that are used to visualize the rigid body
    std::vector<RB3DModel> meshes;

    // end effector points
    std::vector<RBEndEffector> endEffectorPoints;

    // id of the rigid body
    int id = -1;

    // for selection via GUI
    bool selected = false;

    // for drawing abstract view
    double abstractViewCylRadius = 0.01;
    double endEffectorRadius = 0.01;
    V3D jointDrawColor = V3D(0.0, 0.0, 0.9);
    V3D highlightColor = V3D(1.0, 0.5, 0.5);
    V3D colSphereDrawColor = V3D(0.75, 0.0, 0.0);
    V3D endEffectorDrawColor = V3D(0.0, 1.0, 0.0);
    V3D contactedEndEffectorDrawColor = V3D(0.0, 1.0, 1.0);

    // draw color for rb primitive
    V3D color = V3D(0.5, 0.5, 0.5);

    // is this body is fixed to world
    bool fixed = false;

    // physics related coefficients
    double restitutionCoeff = 0;
    double frictionCoeff = 0.8;

public:
    /**
     * default constructor.
     */
    RBProperties() {}

    /**
     * default destructor.
     */
    ~RBProperties() {}

    /**
     * set the moment of inertia of the rigid body - symmetric 3x3 matrix, so
     * we need the six values for it.
     */
    inline void setMOI(double moi00, double moi11, double moi22, double moi01,
                       double moi02, double moi12) {
        MOI_local << moi00, moi01, moi02, moi01, moi11, moi12, moi02, moi12,
            moi22;
    }

    /**
     * parallel axis theorem (translation)
     */
    inline void offsetMOI(double x, double y, double z) {
        setMOI(MOI_local(0, 0) + mass * (y * y + z * z),
               MOI_local(1, 1) + mass * (x * x + z * z),
               MOI_local(2, 2) + mass * (x * x + y * y),
               MOI_local(0, 1) - mass * x * y, MOI_local(0, 2) - mass * x * z,
               MOI_local(1, 2) - mass * y * z);
    }

    /**
     * similarity transformation (rotation)
     */
    inline void rotateMOI(double q, double x, double y, double z) {
        Quaternion quat(q, x, y, z);
        MOI_local = quat * MOI_local * quat.inverse();
    }

    Matrix3x3 getMOI(const Matrix3x3& RToWorld) {
        return RToWorld * MOI_local * RToWorld.transpose();
    }

    Matrix3x3 getMOI(const Quaternion& q) {
        return getMOI(q.toRotationMatrix());
    }

};

}  // namespace crl

