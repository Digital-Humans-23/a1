#include <locomotion/LeggedRobot.h>
#include <robot/GeneralizedCoordinatesRobotRepresentation.h>

#include "TestResult.h"

#define CREATE_ROBOT()                                                 \
    crl::LeggedRobot robot(CRL_DATA_FOLDER "/robots/cora/cora_v4.rbs", \
                           nullptr, false);                            \
    robot.addLimb("fl", robot.getRBByName("tibia_0"));                 \
    robot.addLimb("hl", robot.getRBByName("tibia_1"));                 \
    robot.addLimb("fr", robot.getRBByName("tibia_2"));                 \
    robot.addLimb("hr", robot.getRBByName("tibia_3"));

#define TEST_ANALYTIC_JACOBIAN()                                              \
    crl::GeneralizedCoordinatesRobotRepresentation gcrr(&robot);              \
    crl::Matrix dpdq_analytic;                                                \
    crl::Matrix dpdq_estimated;                                               \
    for (uint i = 0; i < robot.limbs.size(); i++) {                           \
        gcrr.compute_dpdq(robot.limbs[i]->ee->endEffectorOffset,              \
                          robot.limbs[i]->eeRB, dpdq_analytic);               \
        gcrr.estimate_linear_jacobian(robot.limbs[i]->ee->endEffectorOffset,  \
                                      robot.limbs[i]->eeRB, dpdq_estimated);  \
        if (dpdq_analytic.isZero(0)) {                                        \
            res.passed = false;                                               \
            res.error = "\nAnalytic Jacobian is not implemented.\n";          \
        }                                                                     \
        for (uint r = 0; r < dpdq_analytic.rows(); r++) {                     \
            for (uint c = 0; c < dpdq_analytic.cols(); c++) {                 \
                res += SAME(dpdq_analytic(r, c), dpdq_estimated(r, c), 1e-4); \
            }                                                                 \
        }                                                                     \
    }

using namespace tests;

// non-zero base position
TestResult test4_case1() {
    TestResult res;
    CREATE_ROBOT()
    robot.setRootState(crl::P3D(0.1, 0.5, 0.6));
    TEST_ANALYTIC_JACOBIAN()
    return res;
}

// non-zero base orientation
TestResult test4_case2() {
    TestResult res;
    CREATE_ROBOT()
    robot.setRootState(crl::P3D(), crl::Quaternion(0.1, 0.2, 0.3, 0.4));
    TEST_ANALYTIC_JACOBIAN()
    return res;
}

int main(int argc, char *argv[]) {
    // 4
    TEST(test4_case1);
    TEST(test4_case2);
    return (allTestsOk ? 0 : 1);
}
