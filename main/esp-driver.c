#include "driver_socket.h"
#include "driver_wifi.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "i2c_bus.h"
#include "motorhat.h"
#include "nvs_flash.h"
void app_main(void) {

  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }

  ESP_ERROR_CHECK(ret);

  // i2c_bus_t bus;
  // i2c_bus_config_t bus_config = {
  //     .port = I2C_NUM_0,
  //     .sda_io_num = CONFIG_MOTORHAT_SDA_PIN,
  //     .scl_io_num = CONFIG_MOTORHAT_SCL_PIN,
  // };
  // ESP_ERROR_CHECK(i2c_bus_init(&bus, &bus_config));
  //
  // motorhat_handle_t motorhat;
  //
  // motorhat_config_t motorhat_config = {
  //     .pca9685_config =
  //         {
  //             .i2c_addr = CONFIG_MOTORHAT_ADDRESS,
  //             .i2c_speed_hz = 400000,
  //             .pwm_freq_hz = DEFAULT_FREQUENCY_HZ,
  //             .bus_handle = bus.handle,
  //         },
  // };
  //
  // ESP_ERROR_CHECK(motorhat_init(&motorhat, &motorhat_config));

  driver_wifi_config_t wifi_config = {
      .ssid = CONFIG_WIFI_SSID,
      .password = CONFIG_WIFI_PASSWORD,
  };

  ESP_ERROR_CHECK(wifi_init(&wifi_config));
  ESP_ERROR_CHECK(wait_for_wifi_connection());

  driver_socket_handle_t socket_handle;
  driver_socket_config_t socket_config = {
      .ip = CONFIG_SERVER_IP,
      .port = CONFIG_SERVER_PORT,
  };

  ESP_ERROR_CHECK(driver_socket_init(&socket_handle, &socket_config));

  // motorhat_set_motor_direction(&motorhat, MOTORHAT_MOTOR1,
  //                              MOTORHAT_DIRECTION_FORWARD);
  // motorhat_set_motor_speed(&motorhat, MOTORHAT_MOTOR1, 200);
  //
  // vTaskDelay(1000 / portTICK_PERIOD_MS);
  // motorhat_set_motor_speed(&motorhat, MOTORHAT_MOTOR1,
  //                          MOTORHAT_DIRECTION_RELEASE);
}
