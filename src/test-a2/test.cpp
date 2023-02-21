#include <locomotion/LeggedRobot.h>
#include <robot/GeneralizedCoordinatesRobotRepresentation.h>

#include <fstream>
#include <iostream>

#include "TestResult.h"

using namespace tests;

TestResult test_4_AnalyticJacobian() {
    crl::LeggedRobot robot(CRL_DATA_FOLDER "/robots/cora/cora_v4.rbs", nullptr,
                           false);
    robot.addLimb("fl", robot.getRBByName("tibia_0"));
    robot.addLimb("hl", robot.getRBByName("tibia_1"));
    robot.addLimb("fr", robot.getRBByName("tibia_2"));
    robot.addLimb("hr", robot.getRBByName("tibia_3"));

    crl::GeneralizedCoordinatesRobotRepresentation gcrr(&robot);

    crl::Matrix dpdq_analytic;
    crl::Matrix dpdq_estimated;

    TestResult res;

    for (uint i = 0; i < robot.limbs.size(); i++) {
        gcrr.compute_dpdq(robot.limbs[i]->ee->endEffectorOffset,
                          robot.limbs[i]->eeRB, dpdq_analytic);
        gcrr.estimate_linear_jacobian(robot.limbs[i]->ee->endEffectorOffset,
                                      robot.limbs[i]->eeRB, dpdq_estimated);

        // this shouldn't be all zero
        if (dpdq_analytic.isZero(0)) {
            res.passed = false;
            res.error = "\nAnalytic Jacobian is not implemented.\n";
        }

        // compare analytic jacobian with estimated one
        for (uint r = 0; r < dpdq_analytic.rows(); r++) {
            for (uint c = 0; c < dpdq_analytic.cols(); c++) {
                res += SAME(dpdq_analytic(r, c), dpdq_estimated(r, c), 1e-4);
            }
        }
    }

    return res;
}

int main(int argc, char *argv[]) {
    // 4
    TEST(test_4_AnalyticJacobian);
    return (allTestsOk ? 0 : 1);
}
