#include "ads1015.h"
#include "driver/gpio.h"
#include "driver_socket.h"
#include "driver_socket_api.h"
#include "driver_wifi.h"
#include "encoder.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "i2c_bus.h"
#include "limit_switch.h"
#include "motorhat.h"
#include "nvs_flash.h"
#include "pca9685.h"
#include "signal_bus.h"

#define TAG "MAIN"

#define ENCODER_GLITCH_FILTER 100  // CONFIG_ENCODER_GLITCH_FILTER
#define ENCODER_STANDARD_RESOLUTION 20
#define POLAR_PAN_SPEED

static const driver_socket_api_motor_interface_t motor_interface = {
    .polar_pan = motorhat_polar_pan,
    .polar_pan_start = motorhat_polar_pan_start,
    .polar_pan_stop = motorhat_polar_pan_stop,
    .home = motorhat_home,
};

typedef struct {
    encoder_handle_t* encoders;
    int count;
} encoder_array_t;

void clear_all_encoders(void* ctx) {
    encoder_array_t* arr = (encoder_array_t*)ctx;
    for (int i = 0; i < arr->count; i++) {
        encoder_clear_count(&arr->encoders[i]);
    }
}

void app_main(void) {
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  // Install GPIO interrupt service
  ESP_ERROR_CHECK(gpio_install_isr_service(0));

  // Initialize signal bus
  ESP_ERROR_CHECK(signal_bus_init());

  // Initialize I2C bus
  i2c_bus_t bus;
  i2c_bus_config_t bus_config = {
      .port = I2C_NUM_0,
      .sda_io_num = CONFIG_DRIVER_MOTORHAT_SDA_PIN,
      .scl_io_num = CONFIG_DRIVER_MOTORHAT_SCL_PIN,
  };
  ESP_ERROR_CHECK(i2c_bus_init(&bus, &bus_config));

  // Initialize limit switches
  limit_switch_config_t limit_switch_config = {
      .limit_gpio = CONFIG_DRIVER_LIMIT_SWITCH_PIN,
  };
  ESP_ERROR_CHECK(limit_switch_init(&limit_switch_config));

  // Initialize current sensing
  ads1015_handle_t ads;
  ads1015_config_t ads_config = {
      .i2c_addr = CONFIG_DRIVER_ADS1015_ADDRESS,
      .i2c_speed_hz = 400000,
      .rdy_gpio = CONFIG_DRIVER_ADS1015_RDY_PIN,
      .bus_handle = bus.handle,
      .adc_data_rate = CONFIG_DRIVER_ADS1015_DATA_RATE,
  };
  ESP_ERROR_CHECK(ads1015_init(&ads, &ads_config));

  // Initialize encoders
  encoder_handle_t encoders[2];
  encoder_config_t encoder_axis1_config = {
      .P0_pin = 36,
      .P1_pin = 39,
      .resolution = CONFIG_ENCODER_0_RESOLUTION,
      .glitch_filter_ns = CONFIG_ENCODER_GLITCH_FILTER,
      .invert_angle = CONFIG_ENCODER_0_ANGLE_INVERT,
  };
  ESP_ERROR_CHECK(encoder_init(&encoders[0], &encoder_axis1_config));
  ESP_ERROR_CHECK(encoder_start(&encoders[0]));

  encoder_config_t encoder_axis2_config = {
      .P0_pin = 34,
      .P1_pin = 35,
      .resolution = CONFIG_ENCODER_0_RESOLUTION,
      .glitch_filter_ns = CONFIG_ENCODER_GLITCH_FILTER,
      .invert_angle = CONFIG_ENCODER_0_ANGLE_INVERT,
  };
  ESP_ERROR_CHECK(encoder_init(&encoders[1], &encoder_axis2_config));
  ESP_ERROR_CHECK(encoder_start(&encoders[1]));

  encoder_array_t encoder_array = {
    .encoders = encoders,
    .count = 2,
};

  // Initialize motor controller
  motorhat_handle_t motorhat;
  motorhat_config_t motorhat_config = {
      .pca9685_config =
          {
              .i2c_addr = CONFIG_DRIVER_MOTORHAT_ADDRESS,
              .i2c_speed_hz = 400000,
              .pwm_freq_hz = DEFAULT_FREQUENCY_HZ,
              .bus_handle = bus.handle,
          },
      .polar_pan_speed =
          PCA9685_PWM_MAX * atof(CONFIG_DRIVER_MOTORHAT_PAN_SPEED),
      .encoder_cb = (motorhat_encoder_cb_t)clear_all_encoders,
      .encoder_ctx = &encoder_array,
      .limit_gpio = CONFIG_DRIVER_LIMIT_SWITCH_PIN,
  };
  ESP_ERROR_CHECK(motorhat_init(&motorhat, &motorhat_config));

  // Initialize Wi-Fi and socket connection
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

  ESP_ERROR_CHECK(
      driver_socket_init(&socket_handle, &socket_config, &motor_interface));


  // Track encoder values
  int axis1_count;
  int axis2_count;

  while (1) {
    esp_err_t err = encoder_get_raw_count(&encoders[0], &axis1_count);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to get encoder count: %s", esp_err_to_name(err));
    }
    err = encoder_get_raw_count(&encoders[1], &axis2_count);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Failed to get encoder count: %s", esp_err_to_name(err));
    }
    ESP_LOGI(TAG, "Axis 1 count: %i       Axis 2 count: %i", axis1_count, axis2_count);

    vTaskDelay(pdMS_TO_TICKS(500));  // Avoid busy loop

    // This loop can also be used to add periodic tasks like reading sensors, or
    // checking limit switches. However, it must not terminate or structs
    // initialized here will disappear from stack memory, breaking modules that
    // use them.
  }
}
