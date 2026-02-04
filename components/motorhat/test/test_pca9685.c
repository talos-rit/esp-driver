#include "driver/i2c_master.h"
#include "i2c_bus.h"
#include "pca9685.h"
#include "unity.h"

static i2c_bus_t bus_handle;

// Runs BEFORE each test
void setUp(void) {
  i2c_bus_config_t bus_config = {
      .port = I2C_NUM_0,
      .sda_io_num = CONFIG_PCA9685_TEST_SDA_PIN,
      .scl_io_num = CONFIG_PCA9685_TEST_SCL_PIN,
  };
  ESP_ERROR_CHECK(i2c_bus_init(&bus_handle, &bus_config));
}

// Runs AFTER each test
void tearDown(void) {
  // Cleanup here
  i2c_del_master_bus(bus_handle.handle);
}

TEST_CASE("PCA9685 Initialization Test", "[pca9685]") {
  pca9685_handle_t handle;
  pca9685_config_t config = {
      .i2c_addr = CONFIG_PCA9685_TEST_ADDR,
      .i2c_speed_hz = 400000,
      .pwm_freq_hz = 1000,
      .bus_handle = bus_handle.handle,
  };

  esp_err_t err = pca9685_init(&handle, &config);
  TEST_ASSERT_EQUAL(ESP_OK, err);
  TEST_ASSERT_NOT_EQUAL(NULL, handle.dev_handle);
}

TEST_CASE("PCA9685 Wrong Address Test", "[pca9685]") {

  pca9685_handle_t handle;
  pca9685_config_t config = {
      .i2c_addr = 0x00, // Invalid address
      .i2c_speed_hz = 400000,
      .pwm_freq_hz = 1000,
      .bus_handle = bus_handle.handle,
  };

  esp_err_t err = pca9685_init(&handle, &config);
  TEST_ASSERT_NOT_EQUAL(ESP_OK, err);
}

TEST_CASE("PCA9685 Set Duty Cycle Test", "[pca9685]") {
  pca9685_handle_t handle;
  pca9685_config_t config = {
      .i2c_addr = CONFIG_PCA9685_TEST_ADDR,
      .i2c_speed_hz = 400000,
      .pwm_freq_hz = 1000,
      .bus_handle = bus_handle.handle,
  };

  ESP_ERROR_CHECK(pca9685_init(&handle, &config));

  esp_err_t err = pca9685_set_duty_cycle(&handle, PCA9685_CHANNEL0, 0.5f);

  // Read back the registers to verify
  uint8_t on_l, on_h, off_l, off_h;
  ESP_ERROR_CHECK(pca9685_read_register(&handle, PCA9685_LED0_ON_L, &on_l, 1));
  ESP_ERROR_CHECK(pca9685_read_register(&handle, PCA9685_LED0_ON_H, &on_h, 1));
  ESP_ERROR_CHECK(
      pca9685_read_register(&handle, PCA9685_LED0_OFF_L, &off_l, 1));
  ESP_ERROR_CHECK(
      pca9685_read_register(&handle, PCA9685_LED0_OFF_H, &off_h, 1));
  uint16_t on = (on_h << 8) | on_l;
  uint16_t off = (off_h << 8) | off_l;
  uint16_t expected_off = (uint16_t)(PCA9685_PWM_MAX * 0.5f);
  TEST_ASSERT_EQUAL(0, on);
  TEST_ASSERT_EQUAL(expected_off, off);
}

TEST_CASE("PCA9685 Digital Write Test", "[pca9685]") {
  pca9685_handle_t handle;
  pca9685_config_t config = {
      .i2c_addr = CONFIG_PCA9685_TEST_ADDR,
      .i2c_speed_hz = 400000,
      .pwm_freq_hz = 1000,
      .bus_handle = bus_handle.handle,
  };

  ESP_ERROR_CHECK(pca9685_init(&handle, &config));

  // Set channel high
  ESP_ERROR_CHECK(pca9685_digital_write(&handle, PCA9685_CHANNEL1, true));

  // Read back the registers to verify
  uint8_t on_l, on_h, off_l, off_h;
  ESP_ERROR_CHECK(pca9685_read_register(&handle, PCA9685_LED1_ON_L, &on_l, 1));
  ESP_ERROR_CHECK(pca9685_read_register(&handle, PCA9685_LED1_ON_H, &on_h, 1));
  ESP_ERROR_CHECK(
      pca9685_read_register(&handle, PCA9685_LED1_OFF_L, &off_l, 1));
  ESP_ERROR_CHECK(
      pca9685_read_register(&handle, PCA9685_LED1_OFF_H, &off_h, 1));
  uint16_t on = (on_h << 8) | on_l;
  uint16_t off = (off_h << 8) | off_l;
  TEST_ASSERT_EQUAL(PCA9685_PWM_MAX, on);
  TEST_ASSERT_EQUAL(0, off);

  // Set channel low
  ESP_ERROR_CHECK(pca9685_digital_write(&handle, PCA9685_CHANNEL1, false));

  // Read back the registers to verify
  ESP_ERROR_CHECK(pca9685_read_register(&handle, PCA9685_LED1_ON_L, &on_l, 1));
  ESP_ERROR_CHECK(pca9685_read_register(&handle, PCA9685_LED1_ON_H, &on_h, 1));
  ESP_ERROR_CHECK(
      pca9685_read_register(&handle, PCA9685_LED1_OFF_L, &off_l, 1));
  ESP_ERROR_CHECK(
      pca9685_read_register(&handle, PCA9685_LED1_OFF_H, &off_h, 1));
  on = (on_h << 8) | on_l;
  off = (off_h << 8) | off_l;
  TEST_ASSERT_EQUAL(0, on);
  TEST_ASSERT_EQUAL(PCA9685_PWM_MAX, off);
}
