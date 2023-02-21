#pragma once

#include <imgui.h>
#include <utils/mathUtils.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

namespace crl {
namespace gui {

inline glm::vec3 toGLM(const Vector3d &v) {
    return glm::vec3(v.x(), v.y(), v.z());
}

inline glm::vec3 toGLM(const P3D &p) { return glm::vec3(p.x, p.y, p.z); }

inline V3D toV3D(const glm::vec3 &v) { return V3D(v[0], v[1], v[2]); }

inline P3D toP3D(const glm::vec3 &v) { return P3D(v[0], v[1], v[2]); }

inline V3D toV3D(const float *v) { return V3D(v[0], v[1], v[2]); }

inline P3D toP3D(const float *v) { return P3D(v[0], v[1], v[2]); }

inline glm::mat4 getGLMTransform(const V3D &scale,
                                 const Quaternion &orientation,
                                 const P3D &position) {
    // scale, then rotate, then translate...
    glm::mat4 transform = glm::mat4(1.0);
    AngleAxisd rot(orientation);
    transform = glm::translate(transform, toGLM(position));
    transform = transform * glm::rotate(glm::mat4(1.0), (float)(rot.angle()),
                                        toGLM(rot.axis()));
    transform = transform * glm::scale(glm::mat4(1.0), toGLM(scale));
    return transform;
}

}  // namespace gui
}  // namespace crl