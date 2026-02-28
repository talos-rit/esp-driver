#include "driver/i2c_master.h"
#include "i2c_bus.h"
#include "unity.h"

static i2c_bus_t bus_handle;

// Runs BEFORE each test
void setUp(void) {
}

// Runs AFTER each test
void tearDown(void) {
  // Cleanup here
  i2c_del_master_bus(bus_handle.handle);
}

TEST_CASE("I2C Bus Initialization Test", "[i2c_bus]") {
  i2c_bus_config_t config = {
      .port = I2C_NUM_0, .sda_io_num = 21, .scl_io_num = 22};

  esp_err_t err = i2c_bus_init(&bus_handle, &config);
  TEST_ASSERT_EQUAL(ESP_OK, err);
  TEST_ASSERT_NOT_EQUAL(NULL, bus_handle.handle);

}
