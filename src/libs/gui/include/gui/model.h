#pragma once

#include <gui/guiMath.h>
#include <gui/mesh.h>
#include <gui/shader.h>

namespace crl {
namespace gui {

class Model {
public:
    std::vector<Mesh::TextureMap>
        material_textures;  // stores all the textures loaded so far,
                            // optimization to make sure textures aren't loaded
                            // more than once.
    std::vector<Mesh> meshes;

    // this is the name of the model, in case it was loaded from a file
    std::string mName;

    // scale about the x, y, and z axes, applied before any other
    // transformations
    V3D scale = V3D(1, 1, 1);
    // the position of the model in world coordinates
    P3D position = P3D(0, 0, 0);
    // the orientation of the model - takes vectors from local coordinates to
    // world coordinates
    Quaternion orientation = Quaternion::Identity();

    bool highlighted = false;
    bool selected = false;

    Model();
    Model(std::string const &path);

    void draw(const Shader &shader, const V3D &color,
              const glm::mat4 &transform) const;
    void draw(const Shader &shader, const V3D &color) const;
    void draw(const Shader &shader) const;

    glm::mat4 getTransform() const;

private:
    // loads a model with tinyobjloader from file and stores the resulting
    // meshes in the meshes vector.
    void loadModel(std::string const &path_);

public:
    bool hitByRay(const P3D &r_o, const V3D &r_v, P3D &hitPoint, double &t,
                  V3D &n) const;

    bool hitByRay(const P3D &r_o, const V3D &r_v) const;

    bool hitByRay(const P3D &r_o, const V3D &r_v, double &t) const;

    bool hitByRay(const P3D &r_o, const V3D &r_v, P3D &hitPoint) const;

    bool hitByRay(const P3D &r_o, const V3D &r_v, P3D &hitPoint,
                  V3D &hitNormal) const;
};

inline unsigned int textureFromFile(const char *path,
                                    const std::string &directory);

}  // namespace gui
}  // namespace crl