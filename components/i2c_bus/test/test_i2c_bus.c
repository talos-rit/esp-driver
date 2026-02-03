#include "i2c_bus.h"
#include "unity.h"

TEST_CASE("I2C Bus Initialization Test", "[i2c_bus]") {
  i2c_bus_t bus;
  i2c_bus_config_t config = {
      .port = I2C_NUM_0, .sda_io_num = 21, .scl_io_num = 22};

  esp_err_t err = i2c_bus_init(&bus, &config);
  TEST_ASSERT_EQUAL(ESP_OK, err);
  TEST_ASSERT_NOT_EQUAL(NULL, bus.handle);
}
