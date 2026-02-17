#include "motorhat.h"
#include "esp_err.h"

esp_err_t motorhat_init(motorhat_handle_t *handle,
                        const motorhat_config_t *config) {
  if (handle == NULL || config == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  return pca9685_init(handle->pca9685, &config->pca9685_config);
}

esp_err_t motorhat_set_motor_speed(motorhat_handle_t *handle,
                                   motorhat_motor_t motor, uint16_t speed) {
  if (handle == NULL || motor < MOTORHAT_MOTOR1 ||
      motor >= MOTORHAT_NUM_MOTORS) {
    return ESP_ERR_INVALID_ARG;
  }

  if (speed > PCA9685_PWM_MAX) {
    return ESP_ERR_INVALID_ARG;
  }

  const motorhat_motor_channels_t *channels = &motor_channels[motor];

  return pca9685_set_duty_cycle(handle->pca9685, channels->pwm_channel, speed);
}

esp_err_t motorhat_set_motor_direction(motorhat_handle_t *handle,
                                       motorhat_motor_t motor,
                                       motorhat_direction_t direction) {
  if (handle == NULL || motor < MOTORHAT_MOTOR1 ||
      motor >= MOTORHAT_NUM_MOTORS) {
    return ESP_ERR_INVALID_ARG;
  }

  const motorhat_motor_channels_t *channels = &motor_channels[motor];

  esp_err_t err;

  switch (direction) {
  case MOTORHAT_DIRECTION_FORWARD:
    err = pca9685_digital_write(handle->pca9685, channels->in1_channel, true);
    if (err != ESP_OK)
      return err;
    err = pca9685_digital_write(handle->pca9685, channels->in2_channel, false);
    break;
  case MOTORHAT_DIRECTION_BACKWARD:
    err = pca9685_digital_write(handle->pca9685, channels->in1_channel, false);
    if (err != ESP_OK)
      return err;
    err = pca9685_digital_write(handle->pca9685, channels->in2_channel, true);
    break;
  case MOTORHAT_DIRECTION_BRAKE:
    err = pca9685_digital_write(handle->pca9685, channels->in1_channel, true);
    if (err != ESP_OK)
      return err;
    err = pca9685_digital_write(handle->pca9685, channels->in2_channel, true);
    break;
  case MOTORHAT_DIRECTION_RELEASE:
    err = pca9685_digital_write(handle->pca9685, channels->in1_channel, false);
    if (err != ESP_OK)
      return err;
    err = pca9685_digital_write(handle->pca9685, channels->in2_channel, false);
    break;
  default:
    return ESP_ERR_INVALID_ARG;
  }

  return err;
}
