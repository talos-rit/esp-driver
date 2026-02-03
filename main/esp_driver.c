#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "i2c_bus.h"
#include "motorhat.h"
void app_main(void) {

  i2c_bus_t bus;
  i2c_bus_config_t bus_config = {
      .port = I2C_NUM_0,
      .sda_io_num = CONFIG_MOTORHAT_SDA_PIN,
      .scl_io_num = CONFIG_MOTORHAT_SCL_PIN,
  };
  ESP_ERROR_CHECK(i2c_bus_init(&bus, &bus_config));

  motorhat_handle_t motorhat;

  motorhat_config_t motorhat_config = {
      .pca9685_config =
          {
              .i2c_addr = CONFIG_MOTORHAT_ADDRESS,
              .i2c_speed_hz = 400000,
              .pwm_freq_hz = DEFAULT_FREQUENCY_HZ,
              .bus_handle = bus.handle,
          },
  };

  ESP_ERROR_CHECK(motorhat_init(&motorhat, &motorhat_config));

  motorhat_set_motor_direction(&motorhat, MOTORHAT_MOTOR1,
                               MOTORHAT_DIRECTION_FORWARD);
  motorhat_set_motor_speed(&motorhat, MOTORHAT_MOTOR1, 200);

  vTaskDelay(10000 / portTICK_PERIOD_MS);
  motorhat_set_motor_speed(&motorhat, MOTORHAT_MOTOR1,
                           MOTORHAT_DIRECTION_RELEASE);
}
