#pragma once

#include "robot/Robot.h"

namespace crl {

class RBSaver {
public:
    /**
     * This method saves all rigid bodies to file
     */
    static void writeRobotToFile(const Robot *robot, const char *fName) {
        FILE *fp = fopen(fName, "w");
        if (fp == nullptr) {
            std::cerr
                << "Error in RBEngine::writeRBsToFile: Cannot open the file "
                << fName << " for writing..." << std::endl;
            return;
        }

        // Write all rigid bodies to file
        for (RB *rrb : robot->rbList) {
            writeToFile(rrb, fp);
        }

        // Write all joints to file
        for (RBJoint *j : robot->jointList) {
            writeToFile(j, fp);
        }

        fclose(fp);
    }

private:
    /**
     * writes the RB to RBS file.
     */
    static void writeToFile(const RB *rb, FILE *fp) {
        char *str;

        fprintf(fp, "%s\n", getRRBString(RB_RB));

        fprintf(fp, "\t%s %s\n", getRRBString(RB_NAME), rb->name.c_str());

        fprintf(fp, "\t%s %lf\n", getRRBString(RB_MASS), rb->rbProps.mass);

        fprintf(fp, "\t%s %lf\n", getRRBString(RB_FRICTION_COEFF),
                rb->rbProps.frictionCoeff);

        fprintf(fp, "\t%s %lf\n", getRRBString(RB_REST_COEFF),
                rb->rbProps.restitutionCoeff);

        fprintf(fp, "\t%s %lf %lf %lf %lf %lf %lf\n", getRRBString(RB_MOI),
                rb->rbProps.MOI_local(0, 0), rb->rbProps.MOI_local(1, 1),
                rb->rbProps.MOI_local(2, 2), rb->rbProps.MOI_local(0, 1),
                rb->rbProps.MOI_local(0, 2), rb->rbProps.MOI_local(1, 2));

        for (uint i = 0; i < rb->rbProps.meshes.size(); i++) {
            // Hack path a bit...
            std::string newPath = rb->rbProps.meshes[i].path.empty()
                                      ? "None"
                                      : rb->rbProps.meshes[i].path.c_str();
            if (newPath.compare("None") != 0) {
                std::size_t pos = newPath.find("robots");
                newPath = newPath.substr(pos);
            }
            fprintf(fp, "\t%s %s\n", getRRBString(RB_MESH_NAME),
                    newPath.c_str());
            fprintf(fp, "\t%s %s\n", getRRBString(RB_MESH_DESCRIPTION),
                    rb->rbProps.meshes[i].description.c_str());
            fprintf(fp, "\t%s %lf %lf %lf \n", getRRBString(RB_MESH_COLOR),
                    rb->rbProps.meshes[i].color[0],
                    rb->rbProps.meshes[i].color[1],
                    rb->rbProps.meshes[i].color[2]);
            Quaternion q = rb->rbProps.meshes[i].transform.R;
            P3D T = rb->rbProps.meshes[i].transform.T;
            fprintf(fp, "\t%s %lf %lf %lf %lf %lf %lf %lf\n",
                    getRRBString(RB_MESH_TRANSFORMATION), q.w(), q.x(), q.y(),
                    q.z(), T.x, T.y, T.z);
        }

        for (uint i = 0; i < rb->rbProps.collisionShapes.size(); i++) {
            if (auto cs = std::dynamic_pointer_cast<RRBCollisionSphere>(
                    rb->rbProps.collisionShapes[i]))
                fprintf(fp, "\t%s %lf %lf %lf %lf\n",
                        getRRBString(RB_COLLISION_SPHERE),
                        cs->localCoordinates.x, cs->localCoordinates.y,
                        cs->localCoordinates.z, cs->radius);

            if (auto cp = std::dynamic_pointer_cast<RRBCollisionPlane>(
                    rb->rbProps.collisionShapes[i]))
                fprintf(fp, "\t%s %lf %lf %lf %lf %lf %lf\n",
                        getRRBString(RB_COLLISION_PLANE), cp->n.x(), cp->n.y(),
                        cp->n.z(), cp->p.x, cp->p.y, cp->p.z);
        }

        if (rb->rbProps.fixed)
            fprintf(fp, "\t%s\n", getRRBString(RB_IS_FROZEN));

        str = getRRBString(RB_END_RB);
        fprintf(fp, "%s\n\n\n", str);
    }

    /**
     * writes the joint info to file...
     */
    static void writeToFile(const RBJoint *j, FILE *fp) {
        fprintf(fp, "\t%s\n", getRRBString(RB_JOINT));

        fprintf(fp, "\t\t%s %lf %lf %lf\n", getRRBString(RB_JOINT_AXIS),
                j->rotationAxis[0], j->rotationAxis[1], j->rotationAxis[2]);

        fprintf(fp, "\t\t%s %lf\n", getRRBString(RB_DEFAULT_ANGLE),
                j->defaultJointAngle);

        fprintf(fp, "\t\t%s %lf\n", getRRBString(RB_JOINT_MOTOR_KP),
                j->motorKp);

        fprintf(fp, "\t\t%s %lf\n", getRRBString(RB_JOINT_MOTOR_KD),
                j->motorKd);

        char *str;
        str = getRRBString(RB_NAME);
        fprintf(fp, "\t\t%s %s\n", str, j->name.c_str());

        str = getRRBString(RB_PARENT);
        fprintf(fp, "\t\t%s %s\n", str, j->parent->name.c_str());

        str = getRRBString(RB_CHILD);
        fprintf(fp, "\t\t%s %s\n", str, j->child->name.c_str());

        str = getRRBString(RB_CPOS);
        fprintf(fp, "\t\t%s %lf %lf %lf\n", str, j->cJPos.x, j->cJPos.y,
                j->cJPos.z);

        str = getRRBString(RB_PPOS);
        fprintf(fp, "\t\t%s %lf %lf %lf\n", str, j->pJPos.x, j->pJPos.y,
                j->pJPos.z);

        str = getRRBString(RB_JOINT_END);
        fprintf(fp, "\t%s\n\n\n", str);
    }
};

}  // namespace crl
