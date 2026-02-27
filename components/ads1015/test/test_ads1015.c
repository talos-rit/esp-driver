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

  esp_err_t err = ads_init(&handle, &config);
  TEST_ASSERT_EQUAL(ESP_OK, err);
  TEST_ASSERT_NOT_EQUAL(NULL, handle.dev_handle);
}

// TEST(ADS1015, ADS1015_Wrong_Address) {

//   ads1015_config_t config = {
//       .i2c_addr = 0x00, // Invalid address
//       .i2c_speed_hz = 400000,
//       .bus_handle = bus_handle.handle,
//   };

//   esp_err_t err = ads1015_init(&handle, &config);
//   TEST_ASSERT_NOT_EQUAL(ESP_OK, err);
// }

// TEST(ADS1015, PCA9685_Set_Duty_Cycle) {
//   pca9685_config_t config = {
//       .i2c_addr = CONFIG_PCA9685_TEST_ADDR,
//       .i2c_speed_hz = 400000,
//       .pwm_freq_hz = 1000,
//       .bus_handle = bus_handle.handle,
//   };

//   TEST_ASSERT_EQUAL(pca9685_init(&handle, &config), ESP_OK);

//   esp_err_t err = pca9685_set_duty_cycle(&handle, PCA9685_CHANNEL0, 2048);
//   TEST_ASSERT_EQUAL(ESP_OK, err);

//   // Read back the registers to verify
//   uint8_t on_l, on_h, off_l, off_h;
//   TEST_ASSERT_EQUAL(pca9685_read_register(&handle, PCA9685_LED0_ON_L, &on_l), ESP_OK);
//   TEST_ASSERT_EQUAL(pca9685_read_register(&handle, PCA9685_LED0_ON_H, &on_h), ESP_OK);
//   TEST_ASSERT_EQUAL(pca9685_read_register(&handle, PCA9685_LED0_OFF_L, &off_l), ESP_OK);
//   TEST_ASSERT_EQUAL(pca9685_read_register(&handle, PCA9685_LED0_OFF_H, &off_h), ESP_OK);
//   TEST_ASSERT_EQUAL(0, on_l);
//   uint16_t on = (on_h << 8) | on_l;
//   uint16_t off = (off_h << 8) | off_l;
//   uint16_t expected_off = 2048;
//   TEST_ASSERT_EQUAL(0, on);
//   TEST_ASSERT_EQUAL(expected_off, off);
// }

// TEST(PCA9685, PCA9685_Digital_Write) {
//   pca9685_config_t config = {
//       .i2c_addr = CONFIG_PCA9685_TEST_ADDR,
//       .i2c_speed_hz = 400000,
//       .pwm_freq_hz = 1000,
//       .bus_handle = bus_handle.handle,
//   };

//   TEST_ASSERT_EQUAL(pca9685_init(&handle, &config), ESP_OK);

//   // Set channel high
//   TEST_ASSERT_EQUAL(pca9685_digital_write(&handle, PCA9685_CHANNEL1, true), ESP_OK);

//   // Read back the registers to verify
//   uint8_t on_l, on_h, off_l, off_h;
//   TEST_ASSERT_EQUAL(pca9685_read_register(&handle, PCA9685_LED1_ON_L, &on_l), ESP_OK);
//   TEST_ASSERT_EQUAL(pca9685_read_register(&handle, PCA9685_LED1_ON_H, &on_h), ESP_OK);
//   TEST_ASSERT_EQUAL(pca9685_read_register(&handle, PCA9685_LED1_OFF_L, &off_l), ESP_OK);
//   TEST_ASSERT_EQUAL(pca9685_read_register(&handle, PCA9685_LED1_OFF_H, &off_h), ESP_OK);
//   uint16_t on = (on_h << 8) | on_l;
//   uint16_t off = (off_h << 8) | off_l;
//   TEST_ASSERT_EQUAL(PCA9685_PWM_MAX, on);
//   TEST_ASSERT_EQUAL(0, off);

//   // Set channel low
//   TEST_ASSERT_EQUAL(pca9685_digital_write(&handle, PCA9685_CHANNEL1, false), ESP_OK);

//   // Read back the registers to verify
//   TEST_ASSERT_EQUAL(pca9685_read_register(&handle, PCA9685_LED1_ON_L, &on_l), ESP_OK);
//   TEST_ASSERT_EQUAL(pca9685_read_register(&handle, PCA9685_LED1_ON_H, &on_h), ESP_OK);
//   TEST_ASSERT_EQUAL(pca9685_read_register(&handle, PCA9685_LED1_OFF_L, &off_l), ESP_OK);
//   TEST_ASSERT_EQUAL(pca9685_read_register(&handle, PCA9685_LED1_OFF_H, &off_h), ESP_OK);
//   on = (on_h << 8) | on_l;
//   off = (off_h << 8) | off_l;
//   TEST_ASSERT_EQUAL(0, on);
//   TEST_ASSERT_EQUAL(PCA9685_PWM_MAX, off);
// }
