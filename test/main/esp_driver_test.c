#include "unity_fixture.h"

static void print_banner(const char *text);

static void run_all_tests(void) {
  RUN_TEST_GROUP(I2C_Bus);
  RUN_TEST_GROUP(PCA9685);
  RUN_TEST_GROUP(MotorHAT);
}

void app_main(void) {

  print_banner("ESP Driver Test");
  char *unity_args[] = {"esp_driver_test"};
  const char **argv = (const char **)unity_args;
  UnityMain(0, argv, run_all_tests);
}

static void print_banner(const char *text) {
  printf("\n#### %s #####\n\n", text);
}
