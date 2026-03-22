#include "motorhat.h"
#include "esp_err.h"
#include "esp_log.h"
#include "signal_bus.h"

#define TAG "motorhat"

void motor_stop_task(void *args) {
    motorhat_handle_t *handle = (motorhat_handle_t *)args;

    while (1) {
        // Block indefinitely until any stop bit is set
        xEventGroupWaitBits(handle->events,
                            handle->stop_bits,
                            pdFALSE,  // don't clear bits on exit
                            pdFALSE,  // any bit (OR)
                            portMAX_DELAY);

        motorhat_emergency_stop(handle);

        // Wait until all stop bits are cleared before watching again
        xEventGroupWaitBits(handle->events,
                            handle->stop_bits,
                            pdFALSE,
                            pdTRUE,    // all bits (AND)
                            portMAX_DELAY);
    }
}

esp_err_t motorhat_init(motorhat_handle_t *handle, const motorhat_config_t *config, EventGroupHandle_t events) {
  if (handle == NULL || config == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  handle->events = events;
  handle->stop_bits = FAULT_ANY;

  xTaskCreate(motor_stop_task, "motor_stop_task", 4096, handle, 8, NULL);

  // return pca9685_init(&handle->pca9685, &config->pca9685_config);
  return ESP_OK;
}

esp_err_t motorhat_polar_pan(int16_t delta_azimuth, int16_t delta_altitude,
                                  uint16_t delay_ms, uint16_t time_ms) {
    // motorhat_set_motor_direction();
    ESP_LOGI(TAG, "Received polar pan command: delta_azimuth=%d, delta_altitude=%d, delay_ms=%d, time_ms=%d",
             delta_azimuth, delta_altitude, delay_ms, time_ms);
    return ESP_OK;
}

esp_err_t motorhat_polar_pan_start(int8_t delta_azimuth, int8_t delta_altitude) {
    // motorhat_set_motor_direction();
    ESP_LOGI(TAG, "Received polar pan start command: delta_azimuth=%d, delta_altitude=%d",
             delta_azimuth, delta_altitude);
    return ESP_OK;
}

esp_err_t motorhat_polar_pan_stop(void) {
    // motorhat_set_motor_direction();
    ESP_LOGI(TAG, "Received polar pan stop command");
    return ESP_OK;
}

esp_err_t motorhat_home(uint16_t delay_ms) {
    // motorhat_set_motor_direction();
    ESP_LOGI(TAG, "Received home command: delay_ms=%d", delay_ms);
    return ESP_OK;
}

esp_err_t motorhat_set_motor_speed(motorhat_handle_t *handle,
                                   motorhat_motor_t motor, uint16_t speed) {
  if (handle == NULL || motor < MOTORHAT_MOTOR1 ||
      motor >= MOTORHAT_NUM_MOTORS) {
    return ESP_ERR_INVALID_ARG;
  }

  if (xEventGroupGetBits(handle->events) & handle->stop_bits) {
    ESP_LOGW(TAG, "Motor %d in fault state, cannot set speed", motor);
    return ESP_ERR_INVALID_STATE;
  }

  if (speed > PCA9685_PWM_MAX) {
    return ESP_ERR_INVALID_ARG;
  }

  const motorhat_motor_channels_t *channels = &motor_channels[motor];

  // return pca9685_set_duty_cycle(&handle->pca9685, channels->pwm_channel, speed);
  return ESP_OK;
}

esp_err_t motorhat_set_motor_direction(motorhat_handle_t *handle,
                                       motorhat_motor_t motor,
                                       motorhat_direction_t direction) {
  if (handle == NULL || motor < MOTORHAT_MOTOR1 ||
      motor >= MOTORHAT_NUM_MOTORS) {
    return ESP_ERR_INVALID_ARG;
  }

  if (xEventGroupGetBits(handle->events) & handle->stop_bits) {
    ESP_LOGW(TAG, "Motor %d in fault state, cannot set direction", motor);
    return ESP_ERR_INVALID_STATE;
  }

  const motorhat_motor_channels_t *channels = &motor_channels[motor];

  esp_err_t err;

  switch (direction) {
  case MOTORHAT_DIRECTION_FORWARD:
    // err = pca9685_digital_write(&handle->pca9685, channels->in1_channel, true);
    // if (err != ESP_OK)
    //   return err;
    // err = pca9685_digital_write(&handle->pca9685, channels->in2_channel, false);
    ESP_LOGI(TAG, "Set motor %d direction to FORWARD", motor);
    err = ESP_OK;
    break;
  case MOTORHAT_DIRECTION_BACKWARD:
    // err = pca9685_digital_write(&handle->pca9685, channels->in1_channel, false);
    // if (err != ESP_OK)
    //   return err;
    // err = pca9685_digital_write(&handle->pca9685, channels->in2_channel, true);
    ESP_LOGI(TAG, "Set motor %d direction to BACKWARD", motor);
    err = ESP_OK;
    break;
  case MOTORHAT_DIRECTION_BRAKE:
    // err = pca9685_digital_write(&handle->pca9685, channels->in1_channel, true);
    // if (err != ESP_OK)
    //   return err;
    // err = pca9685_digital_write(&handle->pca9685, channels->in2_channel, true);
    ESP_LOGI(TAG, "Set motor %d direction to BRAKE", motor);
    err = ESP_OK;
    break;
  case MOTORHAT_DIRECTION_RELEASE:
    // err = pca9685_digital_write(&handle->pca9685, channels->in1_channel, false);
    // if (err != ESP_OK)
    //   return err;
    // err = pca9685_digital_write(&handle->pca9685, channels->in2_channel, false);
    ESP_LOGI(TAG, "Set motor %d direction to RELEASE", motor);
    err = ESP_OK;
    break;
  default:
    return ESP_ERR_INVALID_ARG;
  }

  return err;
}

esp_err_t motorhat_emergency_stop(motorhat_handle_t *handle) {
    if (handle == NULL) {
      return ESP_ERR_INVALID_ARG;
    }

    esp_err_t first_err = ESP_OK;
    for (int m = MOTORHAT_MOTOR1; m < MOTORHAT_NUM_MOTORS; m++) {
        const motorhat_motor_channels_t *ch = &motor_channels[m];

        // Stop PWM cycle and release both control lines
        ESP_LOGI(TAG, "Emergency stopping motor %d", m);
        // esp_err_t err = pca9685_set_duty_cycle(&handle->pca9685, ch->pwm_channel, 0);
        // if (err != ESP_OK && first_err == ESP_OK) first_err = err;

        // err = pca9685_digital_write(&handle->pca9685, ch->in1_channel, false);
        // if (err != ESP_OK && first_err == ESP_OK) first_err = err;

        // err = pca9685_digital_write(&handle->pca9685, ch->in2_channel, false);
        // if (err != ESP_OK && first_err == ESP_OK) first_err = err;
    }
    return first_err;
}
