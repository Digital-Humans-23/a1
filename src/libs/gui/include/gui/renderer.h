#pragma once

#include <utils/logger.h>

#include "guiMath.h"
#include "model.h"

namespace crl {
namespace gui {

inline void drawSphere(const P3D &p, double r, const Shader &shader,
                       const V3D &color = V3D(1, 0, 0)) {
    static Model sphere = Model(CRL_DATA_FOLDER "/meshes/sphere.obj");

    sphere.position = p;
    sphere.scale = V3D(2 * r, 2 * r, 2 * r);
    sphere.draw(shader, color);
}

/**
 * p is center of ellipsode
 * orientation is orientation of ellipsoid
 * dims is radius along each axis (in ellipsoid's frame)
 */
inline void drawEllipsoid(const P3D &p, const Quaternion &orientation,
                          const V3D &dims, const Shader &shader,
                          const V3D &color = V3D(1, 0, 0)) {
    static Model sphere = Model(CRL_DATA_FOLDER "/meshes/sphere.obj");

    sphere.position = p;
    sphere.orientation = orientation;
    sphere.scale = V3D(2 * dims.x(), 2 * dims.y(), 2 * dims.z());
    sphere.draw(shader, color);
}

inline void drawCuboid(const P3D &p, const Quaternion &orientation,
                       const V3D &dims, const Shader &shader,
                       const V3D &color = V3D(1, 0, 0)) {
    static Model cube = Model(CRL_DATA_FOLDER "/meshes/cube.obj");

    cube.position = p;
    cube.orientation = orientation;
    cube.scale = dims;
    cube.draw(shader, color);
}

inline void drawWireFrameCuboid(const P3D& p, const Quaternion& orientation,
    const V3D& dims, const Shader& shader,
    const V3D& color = V3D(1, 0, 0)) {
    static Model cube = Model(CRL_DATA_FOLDER "/meshes/cube_frame.obj");

    cube.position = p;
    cube.orientation = orientation;
    cube.scale = dims;
    cube.draw(shader, color);
}

inline void drawCylinder(const P3D &startPosition, const P3D &endPosition,
                         const double &radius, const Shader &shader,
                         const V3D &color = V3D(1, 0, 0)) {
    static Model cylinder = Model(CRL_DATA_FOLDER "/meshes/cylinder.obj");

    V3D dir = V3D(startPosition, endPosition);
    double s = dir.norm();
    if (s < 10e-10) return;
    V3D a = dir.normalized();
    V3D b = V3D(0, 0, 1);
    V3D v = (b.cross(a)).normalized();
    // can happen that dir and the z-vector are directly aligned, in which case
    // the angle will be 0 or PI, but we need a valid axis...
    if (v.norm() < 0.5) v = V3D(1, 0, 0);
    float angle = acos(b.dot(a) / (b.norm() * a.norm()));

    cylinder.position = startPosition;
    cylinder.scale = V3D(radius, radius, s);
    cylinder.orientation = Quaternion(AngleAxisd(angle, v));
    cylinder.draw(shader, color);
}

inline void drawCylinder(const P3D &startPosition, const V3D &direction,
                         const double &radius, const Shader &shader,
                         const V3D &color = V3D(1, 0, 0)) {
    drawCylinder(startPosition, startPosition + direction, radius, shader,
                 color);
}

inline void drawCone(const P3D &origin, const V3D &direction,
                     const double &radius, const Shader &shader,
                     const V3D &color = V3D(1, 0, 0)) {
    static Model cone = Model(CRL_DATA_FOLDER "/meshes/cone.obj");

    // Transformation & Scale
    double s = direction.norm();
    if (s < 10e-10) return;
    V3D a = direction.normalized();
    V3D b = V3D(0, 1, 0);
    V3D v = (b.cross(a)).normalized();
    // can happen that dir and the y-vector are directly aligned, in which case
    // the angle will be 0 or PI, but we need a valid axis...
    if (v.norm() < 0.5) v = V3D(1, 0, 0);
    float angle = acos(b.dot(a) / (b.norm() * a.norm()));

    cone.position = origin;
    cone.scale = V3D(1e-3 * radius, 1e-3 * s, 1e-3 * radius);
    cone.orientation = Quaternion(AngleAxisd(angle, v));
    cone.draw(shader, color);
}

inline void drawArrow3d(const P3D &origin, const V3D &direction,
                        const double &radius, const Shader &shader,
                        const V3D &color = V3D(1, 0, 0)) {
    V3D dir = direction;
    for (uint i = 0; i < 3; i++)
        if (fabs(dir[i]) < 1e-6) dir[i] = 1e-6;

    double coneRadius = 1.5 * radius;
    V3D coneDir = dir / dir.norm() * coneRadius * 1.5;
    P3D cyl_endPos = origin + dir - coneDir;

    drawCylinder(origin, cyl_endPos, radius, shader, color);
    drawCone(cyl_endPos, coneDir, coneRadius, shader, color);
}

inline void drawCapsule(const P3D &sP, const P3D &eP, const double &radius,
                        const Shader &shader, const V3D &color = V3D(1, 0, 0)) {
    drawCylinder(sP, eP, radius, shader, color);
    drawSphere(sP, radius, shader, color);
    drawSphere(eP, radius, shader, color);
}

inline void drawCapsule(const P3D &startPosition, const V3D &direction,
                        const double &radius, const Shader &shader,
                        const V3D &color = V3D(1, 0, 0)) {
    drawCapsule(startPosition, P3D(startPosition + direction), radius, shader,
                color);
}

inline void drawCircle(const P3D &center, const V3D &normal,
                       const double &radius, const Shader &shader,
                       const V3D &color = V3D(1, 0, 0)) {
    // Compute start and end position
    double height = 1e-3;
    drawCylinder(center, center + (normal.normalized() * height), radius,
                 shader, color);
}

inline void drawRectangle(const P3D &p, const V3D &normal, const double &angle,
                          const Vector2d &dims, const Shader &shader,
                          const V3D &color = V3D(1, 0, 0)) {
    V3D dim;
    dim[0] = dims[0];
    dim[1] = 1e-3;
    dim[2] = dims[1];
    V3D a = normal.normalized();
    V3D b = V3D(0, 1, 0);
    V3D v = (b.cross(a)).normalized();
    float theta = acos(b.dot(a) / (b.norm() * a.norm()));

    drawCuboid(p,
               Quaternion(AngleAxisd(theta, v)) *
                   Quaternion(AngleAxisd(angle, V3D(0, 1, 0))),
               dim, shader, color);
}

// we could go from the vector "from" the long way, or the short way. The Vector
// up will tell us which one is meant
inline void drawSector(const P3D &p, const V3D &from, const V3D &to,
                       const V3D &up, const Shader &shader,
                       const V3D &color = V3D(1, 0, 0)) {
    static Model sector(CRL_DATA_FOLDER "/meshes/sector.obj");

    double radius = (from.norm() + to.norm()) / 2.0;
    double angleMax = angleBetween(from, to, up);

    drawArrow3d(p, from, radius / 40.0, shader, color);
    drawArrow3d(p, to, radius / 40.0, shader, color);

    // Transformation & Scale
    V3D a = from.cross(to).normalized();
    if (a.dot(up) < 0) a *= -1;

    sector.position = p;
    sector.scale = V3D(1e-3 * radius, 1e-3 * radius, 1e-3 * radius);
    Matrix3x3 rot;

    // the x axis will map to the vector "from" -- in the mesh coordinate frame,
    // x corresponds to the axis the arc is on
    rot.col(0) = from.normalized();
    // the y axis will map to the normal...
    rot.col(1) = a;
    // and the z axis will be the cross product of the two...
    rot.col(2) = from.cross(a).normalized();

    Quaternion qDefault(rot);

    sector.orientation = qDefault;

    // Draw
    for (double ang = 0; ang < angleMax; ang += PI / 180.0) {
        sector.orientation =
            qDefault * Quaternion(AngleAxisd(ang + PI / 2.0, V3D(0, 1, 0)));
        sector.draw(shader, color);
    }
}

inline Model getGroundModel(double s = 100) {
    std::vector<Vertex> vertices = {
        {glm::vec3(-s, 0, -s), glm::vec3(0, 1, 0), glm::vec2(0, 0)},
        {glm::vec3(-s, 0, s), glm::vec3(0, 1, 0), glm::vec2(0, 1)},
        {glm::vec3(s, 0, s), glm::vec3(0, 1, 0), glm::vec2(1, 1)},
        {glm::vec3(s, 0, -s), glm::vec3(0, 1, 0), glm::vec2(1, 0)},
    };

    std::vector<unsigned int> indices = {0, 2, 1, 0, 3, 2};
    Mesh groundMesh(vertices, indices);
    Model ground;
    ground.meshes.push_back(groundMesh);

    return ground;
}

class SimpleGroundModel {
public:
    Model ground = getGroundModel(20);
    Model grid1 = Model(CRL_DATA_FOLDER "/meshes/grid1.obj");
    Model grid2 = Model(CRL_DATA_FOLDER "/meshes/grid2.obj");

    void draw(const Shader &shader, const V3D &col = V3D(0.7, 0.7, 0.9)) {
        grid1.draw(shader, V3D(0.1, 0.1, 0.1));
        grid2.draw(shader, V3D(0.5, 0.5, 0.5));
        ground.draw(shader, col);
    }
};

}  // namespace gui
}  // namespace crl