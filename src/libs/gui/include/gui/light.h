#pragma once

#include <glad/glad.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <vector>
#pragma once

#include <gui/guiMath.h>

#include <iostream>

#include "application.h"

namespace crl {
namespace gui {

/**
 * simple container for lights
 */
class Light {
protected:
    V3D worldUp = V3D(0, 1, 0);

public:
    V3D pos = V3D(20, 30, 20);
    V3D target = V3D(0, 0, 0);
    V3D col = V3D(1, 1, 1);

public:
    glm::vec3 position() const { return toGLM(pos); }

    glm::vec3 color() const { return toGLM(col); }
};

}  // namespace gui
}  // namespace crl