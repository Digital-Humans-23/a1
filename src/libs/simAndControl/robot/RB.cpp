#include <robot/RBJoint.h>
#include <robot/RBUtils.h>
#include <robot/RB.h>
#include <utils/utils.h>

namespace crl {

Matrix3x3 RB::getWorldMOI() {
    return rbProps.getMOI(state.orientation);
}

#define UPDATE_RAY_INTERSECTION(P1, P2)                                        \
    if (ray.getDistanceToSegment(P1, P2, &tmpIntersectionPoint) < cylRadius) { \
        double t = ray.getRayParameterFor(tmpIntersectionPoint);               \
        if (t < tMin) {                                                        \
            intersectionPoint = ray.origin + ray.dir * t;                      \
            tMin = t;                                                          \
        }                                                                      \
    }

bool RB::getRayIntersectionPoint(const Ray &ray, P3D &intersectionPoint,
                                      bool checkMeshes, bool checkSkeleton) {
    P3D tmpIntersectionPoint;
    double tMin = DBL_MAX;
    double t = tMin;
    // we will check all meshes and all cylinders that are used to show the
    // abstract view...

    // meshes first...

    if (checkMeshes)
        for (uint i = 0; i < rbProps.meshes.size(); i++) {
            RigidTransformation meshTransform(state.orientation, state.pos);
            meshTransform *= rbProps.meshes[i].transform;

            rbProps.meshes[i].model->position = meshTransform.T;
            rbProps.meshes[i].model->orientation = meshTransform.R;

            if (rbProps.meshes[i].model->hitByRay(ray.origin, ray.dir, t)) {
                if (t < tMin) {
                    intersectionPoint = ray.origin + ray.dir * t;
                    tMin = t;
                }
            }
        }

    if (checkSkeleton) {
        // and now the cylinders...
        double cylRadius = rbProps.abstractViewCylRadius;

        if (pJoint != NULL)
            UPDATE_RAY_INTERSECTION(pJoint->getWorldPosition(), state.pos);

        for (uint i = 0; i < cJoints.size(); i++)
            UPDATE_RAY_INTERSECTION(state.pos, cJoints[i]->getWorldPosition());

        if (cJoints.size() == 0) {
            for (uint i = 0; i < rbProps.endEffectorPoints.size(); i++) {
                P3D startPos = state.getWorldCoordinates(P3D(0, 0, 0));
                P3D endPos = state.getWorldCoordinates(
                    rbProps.endEffectorPoints[i].endEffectorOffset);
                UPDATE_RAY_INTERSECTION(startPos, endPos);
            }
        }
    }

    return tMin < DBL_MAX / 2.0;
}

}  // namespace crl
