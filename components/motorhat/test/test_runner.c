#include "unity_fixture.h"

/**
 * TEST RUNNER
 */
TEST_GROUP_RUNNER(PCA9685) {
  RUN_TEST_CASE(PCA9685, PCA9685_Initialization);
  RUN_TEST_CASE(PCA9685, PCA9685_Wrong_Address);
  RUN_TEST_CASE(PCA9685, PCA9685_Set_Duty_Cycle);
  RUN_TEST_CASE(PCA9685, PCA9685_Digital_Write);
}

TEST_GROUP_RUNNER(MotorHAT) {
  RUN_TEST_CASE(MotorHAT, MotorHAT_Initialization);
}
