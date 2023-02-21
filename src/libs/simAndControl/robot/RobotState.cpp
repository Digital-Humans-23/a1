#include <robot/RobotState.h>

namespace crl {

void RobotState::writeToFile(const char *fName, Robot *robot) {
    if (fName == nullptr)
        throwError("cannot write to a file whose name is nullptr!");

    FILE *fp = fopen(fName, "w");

    if (fp == nullptr)
        throwError("cannot open the file \'%s\' for reading...", fName);

    V3D velocity(0, 0, 0);
    Quaternion orientation = getOrientation();
    V3D angVelocity(0, 0, 0);
    P3D position = getPosition();

    // double heading = getHeading();
    // setHeading(0);

    fprintf(fp,
            "# order is:\n# Heading Axis\n# Heading\n# Position\n# "
            "Orientation\n# Velocity\n# AngularVelocity\n\n# Relative "
            "Orientation\n# Relative Angular Velocity\n#----------------\n\n# "
            "Heading Axis\n %lf %lf %lf\n# Heading\n%lf\n\n",
            headingAxis[0], headingAxis[1], headingAxis[2], getHeading());

    if (robot != nullptr)
        fprintf(fp, "# Root(%s)\n", robot->root->name.c_str());

    fprintf(fp, "%lf %lf %lf\n", position.x, position.y, position.z);
    fprintf(fp, "%lf %lf %lf %lf\n", orientation.w(), orientation.x(),
            orientation.y(), orientation.z());
    fprintf(fp, "%lf %lf %lf\n", velocity[0], velocity[1], velocity[2]);
    fprintf(fp, "%lf %lf %lf\n\n", angVelocity[0], angVelocity[1],
            angVelocity[2]);

    fprintf(fp, "# number of joints\n%d\n\n", getJointCount());

    for (int i = 0; i < getJointCount(); i++) {
        orientation = getJointRelativeOrientation(i);
        if (robot != nullptr)
            fprintf(fp, "# %s\n", robot->jointList[i]->name.c_str());
        fprintf(fp, "%lf %lf %lf %lf\n", orientation.w(), orientation.x(),
                orientation.y(), orientation.z());
        fprintf(fp, "%lf %lf %lf\n\n", angVelocity[0], angVelocity[1],
                angVelocity[2]);
    }

    fclose(fp);
    // now restore the state of this reduced state...
    // setHeading(heading);
}

void RobotState::readFromFile(const char *fName) {
    if (fName == nullptr)
        throwError("cannot read a file whose name is nullptr!");

    FILE *fp = fopen(fName, "r");
    if (fp == nullptr)
        throwError("cannot open the file \'%s\' for reading...", fName);

    double temp1, temp2, temp3, temp4;

    char line[100];

    // read the heading first...
    double heading;
    readValidLine(line, 100, fp);
    sscanf(line, "%lf %lf %lf", &headingAxis[0], &headingAxis[1],
           &headingAxis[2]);

    readValidLine(line, 100, fp);
    sscanf(line, "%lf", &heading);

    readValidLine(line, 100, fp);
    sscanf(line, "%lf %lf %lf", &temp1, &temp2, &temp3);
    setPosition(P3D(temp1, temp2, temp3));
    readValidLine(line, 100, fp);
    sscanf(line, "%lf %lf %lf %lf", &temp1, &temp2, &temp3, &temp4);
    setOrientation(Quaternion(temp1, temp2, temp3, temp4).normalized());
    readValidLine(line, 100, fp);
    sscanf(line, "%lf %lf %lf", &temp1, &temp2, &temp3);
    readValidLine(line, 100, fp);
    sscanf(line, "%lf %lf %lf", &temp1, &temp2, &temp3);

    int jCount = 0;

    readValidLine(line, 100, fp);
    sscanf(line, "%d", &jCount);
    joints.resize(jCount);

    for (int i = 0; i < jCount; i++) {
        readValidLine(line, 100, fp);
        sscanf(line, "%lf %lf %lf %lf", &temp1, &temp2, &temp3, &temp4);
        setJointRelativeOrientation(
            Quaternion(temp1, temp2, temp3, temp4).normalized(), i);
        readValidLine(line, 100, fp);
        sscanf(line, "%lf %lf %lf", &temp1, &temp2, &temp3);
    }

    // now set the heading...
    setHeading(heading);

    fclose(fp);
}

// setting the heading...
void RobotState::setHeading(double heading) {
    // this means we must rotate the angular and linear velocities of the COM,
    // and augment the orientation
    Quaternion oldHeading = Quaternion::Identity(),
               newHeading = Quaternion::Identity(),
               qRoot = Quaternion::Identity();
    // get the current root orientation, that contains information regarding the
    // current heading
    qRoot = getOrientation();
    // get the twist about the vertical axis...
    oldHeading = computeHeading(qRoot, headingAxis);
    // now we cancel the initial twist and add a new one of our own choosing
    newHeading =
        getRotationQuaternion(heading, headingAxis) * oldHeading.inverse();
    // add this component to the root.
    setOrientation(newHeading * qRoot);
}

}  // namespace crl