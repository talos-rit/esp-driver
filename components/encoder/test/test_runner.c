#include "unity_fixture.h"

/**
 * TEST RUNNER
 */
TEST_GROUP_RUNNER(ENCODER) {
  RUN_TEST_CASE(ENCODER, Encoder_True_Initialization);
}

TEST_GROUP_RUNNER(ENCODER_INFINITE) {
  RUN_TEST_CASE(ENCODER_INFINITE, Encoder_Run_Infinite);
  
}
