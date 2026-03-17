#include "driver_wifi.h"
#include "sdkconfig.h"
#include "unity_fixture.h"

TEST_GROUP(Driver_WiFi);

TEST_SETUP(Driver_WiFi) {}

TEST_TEAR_DOWN(Driver_WiFi) {}

TEST(Driver_WiFi, Driver_WiFi_Init) {
  TEST_ASSERT_EQUAL(ESP_OK, ESP_FAIL);
}
