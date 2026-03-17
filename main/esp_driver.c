#include "driver_socket.h"
#include "driver_wifi.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "i2c_bus.h"

#include "ads1015.h"
#include "limit_switch.h"
#include "motorhat.h"
#include "nvs_flash.h"

#define TAG "MAIN"

void app_main(void) {

  // Install GPIO interrupt service
  ESP_ERROR_CHECK(gpio_install_isr_service(0));

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }

  ESP_ERROR_CHECK(ret);

  limit_switch_config_t limit_switch_config = {
      .limit_gpio = CONFIG_DRIVER_LIMIT_SWITCH_PIN,
  };
  ESP_ERROR_CHECK(limit_switch_init(&limit_switch_config));

  i2c_bus_t bus;
  i2c_bus_config_t bus_config = {
      .port = I2C_NUM_0,
      .sda_io_num = CONFIG_DRIVER_MOTORHAT_SDA_PIN,
      .scl_io_num = CONFIG_DRIVER_MOTORHAT_SCL_PIN,
  };
  ESP_ERROR_CHECK(i2c_bus_init(&bus, &bus_config));

  ads1015_handle_t ads;
  ads1015_config_t ads_config = {
      .i2c_addr = CONFIG_DRIVER_ADS1015_ADDRESS,
      .i2c_speed_hz = 400000,
      .rdy_gpio = CONFIG_DRIVER_ADS1015_RDY_PIN,
      .bus_handle = bus.handle,
      .adc_data_rate = CONFIG_DRIVER_ADS1015_DATA_RATE,
  };
  ESP_ERROR_CHECK(ads1015_init(&ads, &ads_config));

  motorhat_handle_t motorhat;

  motorhat_config_t motorhat_config = {
      .pca9685_config =
          {
              .i2c_addr = CONFIG_DRIVER_MOTORHAT_ADDRESS,
              .i2c_speed_hz = 400000,
              .pwm_freq_hz = DEFAULT_FREQUENCY_HZ,
              .bus_handle = bus.handle,
          },
  };

  ESP_ERROR_CHECK(motorhat_init(&motorhat, &motorhat_config));

  driver_wifi_config_t wifi_config = {
      .ssid = CONFIG_DRIVER_WIFI_SSID,
      .password = CONFIG_DRIVER_WIFI_PASSWORD,
  };

  ESP_ERROR_CHECK(wifi_init(&wifi_config));
  ESP_ERROR_CHECK(wait_for_wifi_connection());

  driver_socket_handle_t socket_handle;
  driver_socket_config_t socket_config = {
      .ip = CONFIG_DRIVER_SERVER_IP,
      .port = CONFIG_DRIVER_SERVER_PORT,
  };

  ESP_ERROR_CHECK(driver_socket_init(&socket_handle, &socket_config));

  int pulse_count;

  while (1) {
    esp_err_t err = encoder_get_raw_count(
  }
}
