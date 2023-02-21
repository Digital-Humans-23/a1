#pragma once

#include <gui/model.h>
#include <gui/shader.h>

#include "robot/RBJoint.h"

namespace crl {

class RBRenderer {
public:
    /* methods used to draw different types of views of the rigid body... */
    static void drawSkeletonView(const RB *rb, const gui::Shader &shader,
                                 bool showJointAxes, bool showJointLimits,
                                 bool showJointAngle);

    static void drawMeshes(const RB *rb, const gui::Shader &shader);

    static void drawCoordFrame(const RB *rb, const gui::Shader &shader);

    static void drawCollisionSpheres(const RB *rb,
                                     const gui::Shader &shader);

    static void drawMOI(const RB *rb, const gui::Shader &shader);

    static void drawMOI(const RBState& rbState, const RBProperties& rbProps, const gui::Shader& shader, bool wireFrame = false);

    static void drawEndEffectors(const RB *rb, const gui::Shader &shader);

    /**
     * draws the axes of rotation
     */
    static void drawAxis(RBJoint *j, const gui::Shader &shader);

    /**
     * draw joint limits
     */
    static void drawJointLimits(RBJoint *j, const gui::Shader &shader);

    /**
     * draw joint angle
     */
    static void drawJointAngle(RBJoint *j, const gui::Shader &shader);
};

}  // namespace crl