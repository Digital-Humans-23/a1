#include "robot/RBLoader.h"

namespace crl {

RBLoader::RBLoader(const char *filePath, bool loadVisuals)
    : loadVisuals(loadVisuals) {
    loadRBsFromFile(filePath);

    // Set root and merge RBs that are fused together
    mergeFixedChildren(rbs[0]);
}

RBLoader::~RBLoader() {
    // Delete leftovers.
    for (uint i = 0; i < rbs.size(); i++) delete rbs[i];
    rbs.clear();
    for (uint i = 0; i < joints.size(); i++) delete joints[i];
    joints.clear();
}

void RBLoader::populateRobot(Robot *robot) {
    // Set root of robot
    robot->root = rbs[0];

    // Gather the list of jointList for the robot
    std::vector<RB *> tempRBs;
    tempRBs.push_back(rbs[0]);

    while (tempRBs.size() > 0) {
        auto rb = tempRBs[0];

        // Add and update all the children jointList to the list
        for (uint i = 0; i < rb->cJoints.size(); i++) {
            auto cJoint = rb->cJoints[i];
            if (cJoint->type == RBJoint::REVOLUTE) {
                robot->jointList.push_back(cJoint);

                // Erase cJoint from this->joints
                for (auto it = this->joints.begin(); it != this->joints.end();
                     it++) {
                    if (*it == cJoint) {
                        this->joints.erase(it);
                        break;
                    }
                }

                tempRBs.push_back(cJoint->child);
            } else {
                throwError("Not supported joint type \'%s\' for joint \'%s\'!",
                           cJoint->type, rb->cJoints[i]->name.c_str());
            }
        }

        // Add rb to robot and rbEngine
        robot->rbList.push_back(rb);

        // Erase rb from this->rbs and tempRBs
        tempRBs.erase(tempRBs.begin());
        for (auto it = this->rbs.begin(); it != this->rbs.end(); it++) {
            if (*it == rb) {
                this->rbs.erase(it);
                break;
            }
        }
    }

    // index the jointList properly...
    for (uint i = 0; i < robot->jointList.size(); i++)
        robot->jointList[i]->jIndex = i;

    // fix link states
    // this is necessary for the case that joints are created from random orders
    // this is not the case for rbs but for urdf the joints are created from
    // leaf node
    robot->fixJointConstraints();
}

void RBLoader::mergeFixedChildren(RB *rb) {
    double mass = rb->rbProps.mass;
    P3D com(0, 0, 0);

    for (auto cJoint : rb->cJoints) {
        auto ch = cJoint->child;
        mergeFixedChildren(ch);  
            
        // here we can assume ch is merged with its children
        if (cJoint->type == RBJoint::FIXED) {
            mass += ch->rbProps.mass;
            com += (cJoint->pJPos - cJoint->cJPos) * ch->rbProps.mass;
        }
    }

    // update rb->rbProps due to com change
    if (mass != 0) {
        // except for the case where the rb has zero mass
        // this rb would be merged with its parents
        com /= mass;
        rb->rbProps.offsetMOI(com.x, com.y, com.z);
        rb->rbProps.mass = mass;

        if (rb->pJoint) rb->pJoint->cJPos -= com;
    }

    // here we merge rb and its children
    std::vector<RBJoint *> cache;

    for (uint i = 0; i < rb->cJoints.size(); i++) {
        auto cJoint = rb->cJoints[i];
        auto ch = cJoint->child;

        // update cJoint's position due to new com
        cJoint->pJPos -= com;

        if (cJoint->type == RBJoint::FIXED) {
            P3D comFromCh = cJoint->cJPos - cJoint->pJPos;

            // update MOI
            ch->rbProps.offsetMOI(comFromCh.x, comFromCh.y, comFromCh.z);
            rb->rbProps.MOI_local += ch->rbProps.MOI_local;

            // update collision sphere
            for (auto &c : rb->rbProps.collisionShapes) {
                if (auto cs = std::dynamic_pointer_cast<RRBCollisionSphere>(c))
                    cs->localCoordinates -= com;
            }
            for (auto &c : ch->rbProps.collisionShapes) {
                if (auto cs =
                        std::dynamic_pointer_cast<RRBCollisionSphere>(c)) {
                    cs->localCoordinates -= comFromCh;
                    rb->rbProps.collisionShapes.push_back(cs);
                }
            }

            // update mesh transformation
            for (auto &m : rb->rbProps.meshes) {
                m.transform.T -= com;
            }
            for (auto &m : ch->rbProps.meshes) {
                m.transform.T -= comFromCh;
                rb->rbProps.meshes.push_back(m);
            }

            // update end effectors
            for (auto &ee : rb->rbProps.endEffectorPoints) {
                ee.endEffectorOffset -= com;
            }
            for (auto &ee : ch->rbProps.endEffectorPoints) {
                ee.endEffectorOffset -= comFromCh;
                rb->rbProps.endEffectorPoints.push_back(ee);
            }

            for (auto ccJoint : ch->cJoints) {
                // change parent 
                // store it to cache first and dump to parent later
                ccJoint->parent = rb;
                ccJoint->pJPos += (cJoint->pJPos - cJoint->cJPos);
                cache.push_back(ccJoint);
                // rb->cJoints.push_back(ccJoint);
            }

            // erase current fixed joints from rb->cjoints
            rb->cJoints.erase(rb->cJoints.begin() + i--);
        }
    }

    for(uint i = 0; i < cache.size(); i++)
        rb->cJoints.push_back(cache[i]);
}

RB *RBLoader::getRBByName(const char *name) {
    if (name == nullptr) return nullptr;
    for (int i = (int)rbs.size() - 1; i >= 0; i--)
        if (strcmp(name, rbs[i]->name.c_str()) == 0) return rbs[i];
    throwError(
        "RBLoader::getRBByName -> rigid body with name %s does not exist",
        name);
    return nullptr;
}

RBJoint *RBLoader::getJointByName(char *name) {
    if (name == nullptr) return nullptr;
    for (int i = (int)joints.size() - 1; i >= 0; i--)
        if (strcmp(name, joints[i]->name.c_str()) == 0) return joints[i];
    throwError("RBLoader::getJointByName -> joint with name %s does not exist",
               name);
    return nullptr;
}

void RBLoader::loadRBsFromFile(const char *fName) {
    if (fName == nullptr) throwError("nullptr file name provided.");

    std::string fn = std::string(fName);
    std::string fmt = fn.substr(fn.find_last_of(".") + 1);

    if (fmt == "rbs") {
        // Read rbs file
        FILE *f = fopen(fName, "r");
        if (f == nullptr) throwError("Could not open file: %s", fName);

        // have a temporary buffer used to read the file line by line...
        char buffer[200];
        RB *newBody = nullptr;
        RBJoint *j = nullptr;

        // this is where it happens.
        while (!feof(f)) {
            // get a line from the file...
            readValidLine(buffer, 200, f);
            char *line = lTrim(buffer);
            if (strlen(line) == 0) continue;
            int lineType = getRRBLineType(line);
            switch (lineType) {
                case RB_RB:
                    // create a new rigid body and have it load its own info...
                    newBody = new RB();
                    loadFromFile(newBody, f);
                    rbs.push_back(newBody);
                    break;
                case RB_JOINT:
                    j = new RBJoint();
                    loadFromFile(j, f);
                    joints.push_back(j);
                    break;
                case RB_NOT_IMPORTANT:
                    if (strlen(line) != 0 && line[0] != '#')
                        std::cout << "Ignoring input line: " << line
                                  << std::endl;
                    break;
                default:
                    throwError(
                        "Incorrect rigid body input file: \'%s\' - "
                        "unexpected line.",
                        buffer);
            }
        }

        fclose(f);
    } else {
        throwError("unsupported robot file.");
    }
}

void RBLoader::loadFromFile(RB *rb, FILE *fp) {
    if (fp == nullptr) throwError("Invalid file pointer.");

    // have a temporary buffer used to read the file line by line...
    char buffer[200];

    // this is where it happens.
    while (!feof(fp)) {
        // get a line from the file...
        readValidLine(buffer, 200, fp);
        char *line = lTrim(buffer);
        int lineType = getRRBLineType(line);
        switch (lineType) {
            case RB_NAME: {
                rb->name = std::string() + trim(line);
            } break;
            case RB_MESH_NAME: {
                if (loadVisuals) {
                    char tmpStr[200];
                    sscanf(line, "%s", tmpStr);
                    std::string str(tmpStr);
                    rb->rbProps.meshes.push_back(RB3DModel());

                    if (str != "None") {  
                        rb->rbProps.meshes.back().model =
                            new gui::Model(CRL_DATA_FOLDER "/" + str);
                    }
                }
            } break;
            case RB_MESH_COLOR: {
                if (loadVisuals) {
                    double r, g, b;
                    if (sscanf(line, "%lf %lf %lf", &r, &g, &b) != 3)
                        throwError(
                            "Incorrect rigid body input file - mesh color "
                            "parameter expects 3 arguments (mesh color %s)\n",
                            line);
                    for (auto &m : rb->rbProps.meshes.back().model->meshes) {
                        m.defaultColor = V3D(r, g, b);
                    }
                }
            } break;
            case RB_MATCAP_TEXTURE: {
            } break;
            case RB_MASS: {
                double t = 1;
                if (sscanf(line, "%lf", &t) != 1)
                    std::cout
                        << "Incorrect rigid body input file - a mass needs "
                           "to be specified if the 'mass' keyword is used."
                        << std::endl;
                rb->rbProps.mass = t;
            } break;
            case RB_MOI: {
                double t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0, t6 = 0;
                sscanf(line, "%lf %lf %lf %lf %lf %lf", &t1, &t2, &t3, &t4, &t5,
                       &t6);
                if (t1 <= 0 || t2 <= 0 || t3 <= 0)
                    std::cout << "Incorrect values for the principal moments "
                                 "of inertia."
                              << std::endl;
                rb->rbProps.setMOI(t1, t2, t3, t4, t5, t6);
            } break;
            case RB_NOT_IMPORTANT: {
                if (strlen(line) != 0 && line[0] != '#') {
                    std::cout << "Ignoring input line: " << line << std::endl;
                }
            } break;
            case RB_COLLISION_SPHERE: {
                double t0 = 0.0, t1 = 0.0, t2 = 0.0, t3 = 0.0;
                if (sscanf(line, "%lf %lf %lf %lf", &t0, &t1, &t2, &t3) !=
                    4)  // First three entries: local coodinates, forth: radius
                    throwError(
                        "Incorrect rigid body input file - 4 arguments are "
                        "required to specify the local coordinates and the "
                        "radius of a collision sphere\n",
                        line);
                rb->rbProps.collisionShapes.push_back(
                    std::make_shared<RRBCollisionSphere>(P3D(t0, t1, t2), t3));
            } break;
            case RB_COLLISION_PLANE: {
                double t0 = 0.0, t1 = 0.0, t2 = 0.0, t3 = 0.0, t4 = 0.0,
                       t5 = 0.0;
                if (sscanf(line, "%lf %lf %lf %lf %lf %lf", &t0, &t1, &t2, &t3,
                           &t4,
                           &t5) !=
                    6)  // Normal vector and position of the origin
                    throwError(
                        "Incorrect rigid body input file - 6 arguments are "
                        "required to specify the normal and the origin of a "
                        "collision plane\n",
                        line);
                rb->rbProps.collisionShapes.push_back(
                    std::make_shared<RRBCollisionPlane>(P3D(t3, t4, t5),
                                                        V3D(t0, t1, t2)));
            } break;

            case RB_MESH_DESCRIPTION: {
                if (loadVisuals) {
                    char tmpStr[200];
                    sscanf(line, "%s", tmpStr);
                    std::string str(tmpStr);
                    rb->rbProps.meshes.back().description = str;
                }
            } break;
            case RB_MESH_TRANSFORMATION: {
                if (loadVisuals) {
                    double qw, qx, qy, qz;
                    P3D T = P3D(0, 0, 0);
                    sscanf(line, "%lf %lf %lf %lf %lf %lf %lf", &qw, &qx, &qy,
                           &qz, &T.x, &T.y, &T.z);
                    rb->rbProps.meshes.back().transform.R =
                        Quaternion(qw, qx, qy, qz);
                    rb->rbProps.meshes.back().transform.T = T;
                }
            } break;
            case RB_END_EFFECTOR: {
                P3D EE = P3D(0, 0, 0);
                double radius = 0;
                // Only parse EE position and size. Ignore others.
                sscanf(line, "%lf %lf %lf %lf %*d %*lf %*lf %*lf %*d", &EE.x,
                       &EE.y, &EE.z, &radius);
                rb->rbProps.endEffectorPoints.emplace_back();
                rb->rbProps.endEffectorPoints.back().endEffectorOffset = EE;
                // rb->rbProps.endEffectorPoints.back().endEffectorTip = EE;
                rb->rbProps.endEffectorPoints.back().radius = radius;
                rb->rbProps.endEffectorPoints.back().name =
                    rb->name;  // just use rb's name for now...
            } break;
            case RB_FRICTION_COEFF: {
                sscanf(line, "%lf", &rb->rbProps.frictionCoeff);
            } break;
            case RB_REST_COEFF: {
                sscanf(line, "%lf", &rb->rbProps.restitutionCoeff);
            } break;
            case RB_IS_FROZEN:
                rb->rbProps.fixed = true;
                break;
            case RB_END_RB: {
                return;  // and... done
            } break;
            default:
                throwError(
                    "Incorrect rigid body input file: \'%s\' - unexpected "
                    "line.",
                    buffer);
        }
    }
    throwError("Incorrect articulated body input file! No /End found");
}

/**
        This method is used to load the details of a joint from file. The
   PhysicalWorld parameter points to the rbEngine in which the objects that need
   to be linked live in.
*/
void RBLoader::loadFromFile(RBJoint *j, FILE *f) {
    if (f == nullptr) throwError("Invalid file pointer.");
    // have a temporary buffer used to read the file line by line...
    char buffer[200];
    char tempName[100];

    // this is where it happens.
    while (!feof(f)) {
        // get a line from the file...
        if (!fgets(buffer, 200, f))
            throwError("The input file reading failed.");
        if (strlen(buffer) > 195)
            throwError(
                "The input file contains a line that is longer than "
                "~200 characters - not allowed");
        char *line = lTrim(buffer);

        if (processInputLine(j, line)) continue;

        int lineType = getRRBLineType(line);
        switch (lineType) {
            case RB_NAME:
                j->name = std::string() + trim(line);
                break;
            case RB_PARENT:
                sscanf(line, "%s", tempName);
                if (j->parent != nullptr)
                    throwError("This joint already has a parent");
                j->parent = getRBByName(tempName);
                if (j->parent == nullptr)
                    throwError(
                        "The articulated rigid body \'%s\' cannot be found!",
                        tempName);
                break;
            case RB_CHILD:
                sscanf(line, "%s", tempName);
                if (j->child != nullptr)
                    throwError("This joint already has a parent");
                j->child = getRBByName(tempName);
                if (j->child == nullptr)
                    throwError(
                        "The articulated rigid body \'%s\' cannot be found!",
                        tempName);
                break;
            case RB_CPOS:
                sscanf(line, "%lf %lf %lf", &j->cJPos.x, &j->cJPos.y,
                       &j->cJPos.z);
                break;
            case RB_PPOS:
                sscanf(line, "%lf %lf %lf", &j->pJPos.x, &j->pJPos.y,
                       &j->pJPos.z);
                break;
            case RB_JOINT_END:
                // we now have to link together the child and parent bodies
                if (j->child == nullptr)
                    throwError(
                        "A joint has been found that does not have a child "
                        "rigid body");
                if (j->parent == nullptr)
                    throwError(
                        "A joint has been found that does not have a parent "
                        "rigid body");
                if (j->child->pJoint != nullptr)
                    throwError(
                        "A joint has been found that does not have a parent "
                        "rigid body");

                j->child->pJoint = j;
                j->parent->cJoints.push_back(j);
                j->fixJointConstraints(true, false);

                return;  // and... done
                break;
            case RB_NOT_IMPORTANT:
                if (strlen(line) != 0 && line[0] != '#') {
                    Logger::print("Ignoring input line: \'%s\'\n", line);
                }
                break;
            default:
                throwError(
                    "Incorrect articulated body input file: \'%s\' - "
                    "unexpected line.",
                    buffer);
        }
    }
    throwError(
        "Incorrect articulated body input file! No /ArticulatedFigure found");
}

bool RBLoader::processInputLine(RBJoint *j, char *line) {
    int lineType = getRRBLineType(line);
    switch (lineType) {
        case RB_JOINT_AXIS:
            sscanf(line, "%lf %lf %lf", &j->rotationAxis[0],
                   &j->rotationAxis[1], &j->rotationAxis[2]);
            j->rotationAxis.normalize();
            return true;
            break;
        case RB_JOINT_LIMITS:
            sscanf(line, "%lf %lf", &j->minAngle, &j->maxAngle);
            j->jointLimitsActive = true;
            return true;
            break;
        case RB_DEFAULT_ANGLE:
            sscanf(line, "%lf", &j->defaultJointAngle);
            return true;
            break;
        default:
            return false;
    }
}

}  // namespace crl
