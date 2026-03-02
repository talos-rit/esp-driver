#include "driver/i2c_master.h"
#include "i2c_bus.h"
#include "ads1015.h"
#include "unity_fixture.h"

static i2c_bus_t bus_handle;
static ads1015_handle_t handle;

TEST_GROUP(ADS1015);

TEST_SETUP(ADS1015) {
  i2c_bus_config_t bus_config = {
      .port = I2C_NUM_0,
      .sda_io_num = CONFIG_ADS1015_TEST_SDA_PIN,
      .scl_io_num = CONFIG_ADS1015_TEST_SCL_PIN,
  };
  ESP_ERROR_CHECK(i2c_bus_init(&bus_handle, &bus_config));
}

TEST_TEAR_DOWN(ADS1015) {
  if (handle.dev_handle != NULL) {
    i2c_master_bus_rm_device(handle.dev_handle);
  }
  if (bus_handle.handle != NULL) {
    i2c_del_master_bus(bus_handle.handle);
  }
}

TEST(ADS1015, ADS1015_Initialization) {
  ads1015_config_t config = {
      .i2c_addr = CONFIG_ADS1015_TEST_ADDR,
      .i2c_speed_hz = 400000,
      .bus_handle = bus_handle.handle,
  };

  esp_err_t err = ads1015_init(&handle, &config);
  TEST_ASSERT_EQUAL(ESP_OK, err);
  TEST_ASSERT_NOT_EQUAL(NULL, handle.dev_handle);
}

TEST(ADS1015, ADS1015_Wrong_Address) {

  ads1015_config_t config = {
      .i2c_addr = 0x00, // Invalid address
      .i2c_speed_hz = 400000,
      .bus_handle = bus_handle.handle,
  };

  esp_err_t err = ads1015_init(&handle, &config);
  TEST_ASSERT_NOT_EQUAL(ESP_OK, err);
}
