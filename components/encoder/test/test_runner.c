#include "unity_fixture.h"

/**
 * TEST RUNNER
 */
TEST_GROUP_RUNNER(ENCODER) {
  RUN_TEST_CASE(ENCODER, Encoder_Initialization);
  RUN_TEST_CASE(ENCODER, Encoder_Run_Short);
}
