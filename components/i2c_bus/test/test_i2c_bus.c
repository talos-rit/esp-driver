#include "driver/i2c_master.h"
#include "i2c_bus.h"
#include "sdkconfig.h"
#include "unity_fixture.h"

static i2c_bus_t bus_handle;

TEST_GROUP(I2C_Bus);

TEST_SETUP(I2C_Bus) {}

TEST_TEAR_DOWN(I2C_Bus) {
  if (bus_handle.handle != NULL) {
    i2c_del_master_bus(bus_handle.handle);
  }
}

TEST(I2C_Bus, I2C_Bus_Initialization) {
  i2c_bus_config_t config = {
      .port = I2C_NUM_0, .sda_io_num = CONFIG_I2CBUS_TEST_SDA_PIN, .scl_io_num = CONFIG_I2CBUS_TEST_SCL_PIN};

  esp_err_t err = i2c_bus_init(&bus_handle, &config);
  TEST_ASSERT_EQUAL(ESP_OK, err);
  TEST_ASSERT_NOT_EQUAL(NULL, bus_handle.handle);
}
