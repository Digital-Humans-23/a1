#include <robot/RBUtils.h>
#include <utils/utils.h>

namespace crl {

// assume that the gravity is in the y-direction (this can easily be changed if
// need be), and this value gives its magnitude.
double RBGlobals::g = -9.8;
// this is the direction of the up-vector
V3D RBGlobals::worldUp = V3D(0, 1, 0);
// and the ground plane
Plane RBGlobals::groundPlane = Plane(P3D(0, 0, 0), RBGlobals::worldUp);

KeyWord RRBkeywords[] = {{"RB", RB_RB},
                         {"/End_RB", RB_END_RB},
                         {"name", RB_NAME},
                         {"mass", RB_MASS},
                         {"moi", RB_MOI},
                         {"collisionSphere", RB_COLLISION_SPHERE},
                         {"collisionPlane", RB_COLLISION_PLANE},
                         {"frozen", RB_IS_FROZEN},
                         {"mesh", RB_MESH_NAME},
                         {"meshColor", RB_MESH_COLOR},
                         {"meshTransformation", RB_MESH_TRANSFORMATION},
                         {"meshDescription", RB_MESH_DESCRIPTION},
                         {"matcapTexture", RB_MATCAP_TEXTURE},
                         {"child", RB_CHILD},
                         {"parent", RB_PARENT},
                         {"endEffector", RB_END_EFFECTOR},
                         {"jointPPos", RB_PPOS},
                         {"jointCPos", RB_CPOS},
                         {"RBJoint", RB_JOINT},
                         {"jointLimits", RB_JOINT_LIMITS},
                         {"jointAxis", RB_JOINT_AXIS},
                         {"defaultAngle", RB_DEFAULT_ANGLE},
                         {"jointMotorKp", RB_JOINT_MOTOR_KP},
                         {"jointMotorKd", RB_JOINT_MOTOR_KD},
                         {"/End_Joint", RB_JOINT_END},
                         {"frictionCoefficient", RB_FRICTION_COEFF},
                         {"restitutionCoefficient", RB_REST_COEFF}

};

int getRRBLineType(char *&buffer) {
    return getLineType(buffer, RRBkeywords,
                       sizeof(RRBkeywords) / sizeof(RRBkeywords[0]));
}

char *getRRBString(int token) {
    return getKeyword(token, RRBkeywords,
                      sizeof(RRBkeywords) / sizeof(RRBkeywords[0]));
}

}  // namespace crl
