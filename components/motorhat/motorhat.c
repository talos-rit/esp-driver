#include "motorhat.h"

#include "esp_err.h"
#include "esp_log.h"
#include "signal_bus.h"
#include "driver/gpio.h"

#define TAG "motorhat"

static motorhat_handle_t* s_handle = NULL;

static motorhat_direction_t delta_to_direction(int8_t delta) {
  if (delta > 0) return MOTORHAT_DIRECTION_FORWARD;
  if (delta < 0) return MOTORHAT_DIRECTION_BACKWARD;

  return MOTORHAT_DIRECTION_RELEASE;
}

void motor_stop_task(void* args) {
  while (1) {
    // Block indefinitely until any stop bit is set
    xEventGroupWaitBits(g_motor_events, CURRENT_ANY,
                        pdFALSE,  // don't clear bits on exit
                        pdFALSE,  // any bit (OR)
                        portMAX_DELAY);

    // Check for homing sequence before reacting
    if (!(xEventGroupGetBits(g_motor_events) & HOMING_FLAG)){
      motorhat_emergency_stop(s_handle);
    } else {
    // Homing sequence will handle this, yield and check again shortly
    vTaskDelay(pdMS_TO_TICKS(10));
}
  }
}

esp_err_t motorhat_init(motorhat_handle_t* handle,
                        const motorhat_config_t* config) {
  if (handle == NULL || config == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  s_handle = handle;
  handle->polar_pan_speed = config->polar_pan_speed;
  handle->encoder_cb = config->encoder_cb;
  handle->encoder_ctx = config->encoder_ctx;
  handle->limit_gpio = config->limit_gpio;

  xTaskCreate(motor_stop_task, "motor_stop_task", 4096, handle, 8, NULL);

  return pca9685_init(&handle->pca9685, &config->pca9685_config);
  return ESP_OK;
}

// Socket commands
esp_err_t motorhat_home(uint16_t delay_ms) {
  if (xEventGroupGetBits(g_motor_events) & HOMING_FLAG) {
    ESP_LOGW(TAG, "Homing in progress, cannot send commands");
    return ESP_ERR_INVALID_STATE;
  }
  
  if (s_handle == NULL) return ESP_ERR_INVALID_STATE;

  ESP_LOGI(TAG, "Received home command: delay_ms=%d", delay_ms);

  // Set homing flag
  xEventGroupSetBits(g_motor_events, HOMING_FLAG);

  // Add delay
  vTaskDelay(pdMS_TO_TICKS(delay_ms));

  // Stop any movement before homing
  for (int m = MOTORHAT_MOTOR1; m < MOTORHAT_NUM_MOTORS; m++) {
    motorhat_set_motor_speed(s_handle, m, 0);
    motorhat_set_motor_direction(s_handle, m, MOTORHAT_DIRECTION_RELEASE);
  }

  // Home each axis individually until we candetermine which motor triggers limit switch and current events
  for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < MOTORHAT_NUM_AXES; axis++) {
    int m = axis_motor[axis];
    ESP_LOGI(TAG, "Homing motor %d", m);

    motorhat_set_motor_direction(s_handle, m, MOTORHAT_DIRECTION_FORWARD);
    motorhat_set_motor_speed(s_handle, m, s_handle->polar_pan_speed);

    EventBits_t bits = xEventGroupWaitBits(
      g_motor_events, 
      HOME_EVENT, 
      pdTRUE, 
      pdFALSE, 
      portMAX_DELAY
    );

    // Stop when a homing event is detected
    motorhat_set_motor_speed(s_handle, m, 0);

    if (bits & CURRENT_ANY) {
      ESP_LOGI(TAG, "Motor %d hit the forward end of its range", m);
      
      // Small pause before going opposite direction
      vTaskDelay(pdMS_TO_TICKS(200));

      // Switch directions and move towards limit switch
      motorhat_set_motor_direction(s_handle, m, MOTORHAT_DIRECTION_BACKWARD);
      motorhat_set_motor_speed(s_handle, m, s_handle->polar_pan_speed);

      bits = xEventGroupWaitBits(
        g_motor_events, 
        HOME_EVENT,
        pdTRUE, 
        pdFALSE, 
        portMAX_DELAY
      );
      
      // Stop when a homing event is detected
      motorhat_set_motor_speed(s_handle, m, 0);

      if (bits & LIMIT_ANY) {
        ESP_LOGI(TAG, "Motor %d hit a limit switch moving backward", m);

        // Continue forward slowly until limit switch releases
        motorhat_set_motor_speed(s_handle, m, s_handle->polar_pan_speed / 3);

        // Detect limit switch release by polling
        while (!gpio_get_level(s_handle->limit_gpio)) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        ESP_LOGI(TAG, "Motor %d finished homing", m);
        
        // Homing finished, stop the motor and continue sequence
        motorhat_set_motor_speed(s_handle, m, 0);
        motorhat_set_motor_direction(s_handle, m, MOTORHAT_DIRECTION_RELEASE);

      } else {
        ESP_LOGW(TAG, "Motor %d unexpected homing event detected", m);
        motorhat_emergency_stop(s_handle);
      }

    } else if (bits & LIMIT_ANY) {
      ESP_LOGI(TAG, "Motor %d hit a limit switch moving forward", m);

      // Small pause before going the opposite direction
      vTaskDelay(pdMS_TO_TICKS(200));

      // Turn around and move slowly until limit switch releases
      motorhat_set_motor_direction(s_handle, m, MOTORHAT_DIRECTION_BACKWARD);
      motorhat_set_motor_speed(s_handle, m, s_handle->polar_pan_speed / 3);

      // Detect limit switch release by polling
      while (!gpio_get_level(s_handle->limit_gpio)) {
          vTaskDelay(pdMS_TO_TICKS(10));
      }
      ESP_LOGI(TAG, "Motor %d finished homing", m);

      // Homing finished, stop the motor and continue sequence
      motorhat_set_motor_speed(s_handle, m, 0);
      motorhat_set_motor_direction(s_handle, m, MOTORHAT_DIRECTION_RELEASE);

    } else {
      ESP_LOGW(TAG, "Motor %d unexpected homing event detected", m);
      motorhat_emergency_stop(s_handle);
    }
  }
  
  // Reset encoder values
  if (s_handle->encoder_cb) {
      s_handle->encoder_cb(s_handle->encoder_ctx);
      ESP_LOGI(TAG, "Homing successful, encoders reset");
  } else {
      ESP_LOGW(TAG, "Encoder reset callback not found, homing unsuccessful");
  }

  // Clear homing flag
  xEventGroupClearBits(g_motor_events, HOMING_FLAG);

  return ESP_OK;
}

esp_err_t motorhat_polar_pan(int16_t delta_azimuth, int16_t delta_altitude,
                             uint16_t delay_ms, uint16_t time_ms) {
  if (xEventGroupGetBits(g_motor_events) & HOMING_FLAG) {
    ESP_LOGW(TAG, "Homing in progress, cannot send commands");
    return ESP_ERR_INVALID_STATE;
  }

  if (s_handle == NULL) return ESP_ERR_INVALID_STATE;

  ESP_LOGI(TAG, "Received polar pan command: delta_azimuth=%d, delta_altitude=%d, delay_ms=%d, time_ms=%d", 
    delta_azimuth, delta_altitude, delay_ms, time_ms);

  // TODO: Implement polar pan logic using motorhat_set_motor_direction and
  // motorhat_set_motor_speed
  return ESP_OK;
}

esp_err_t motorhat_polar_pan_start(int8_t delta_azimuth,
                                   int8_t delta_altitude) {
  if (xEventGroupGetBits(g_motor_events) & HOMING_FLAG) {
    ESP_LOGW(TAG, "Homing in progress, cannot send commands");
    return ESP_ERR_INVALID_STATE;
  }

  if (s_handle == NULL) return ESP_ERR_INVALID_STATE;

  ESP_LOGI(TAG, "Received polar pan start command: delta_azimuth=%d, delta_altitude=%d", delta_azimuth, delta_altitude);

  // Set both axes speed to 0 to prevent direction change while moving
  esp_err_t err =
      motorhat_set_motor_speed(s_handle, axis_motor[MOTORHAT_AXIS_AZIMUTH], 0);
  if (err != ESP_OK) {
    return err;
  }
  err =
      motorhat_set_motor_speed(s_handle, axis_motor[MOTORHAT_AXIS_ALTITUDE], 0);
  if (err != ESP_OK) {
    return err;
  }

  // Set directions based on deltas
  err =
      motorhat_set_motor_direction(s_handle, axis_motor[MOTORHAT_AXIS_AZIMUTH],
                                   delta_to_direction(delta_azimuth));
  if (err != ESP_OK) {
    return err;
  }
  err =
      motorhat_set_motor_direction(s_handle, axis_motor[MOTORHAT_AXIS_ALTITUDE],
                                   delta_to_direction(delta_altitude));
  if (err != ESP_OK) {
    return err;
  }

  // Set speeds to a default value
  err = motorhat_set_motor_speed(s_handle, axis_motor[MOTORHAT_AXIS_AZIMUTH],
                                 s_handle->polar_pan_speed);
  if (err != ESP_OK) {
    return err;
  }
  return motorhat_set_motor_speed(s_handle, axis_motor[MOTORHAT_AXIS_ALTITUDE],
                                  s_handle->polar_pan_speed);
}

esp_err_t motorhat_polar_pan_stop(void) {
  if (xEventGroupGetBits(g_motor_events) & HOMING_FLAG) {
    ESP_LOGW(TAG, "Homing in progress, cannot send commands");
    return ESP_ERR_INVALID_STATE;
  }

  if (s_handle == NULL) return ESP_ERR_INVALID_STATE;

  ESP_LOGI(TAG, "Received polar pan stop command");

  for (int m = MOTORHAT_MOTOR1; m < MOTORHAT_NUM_MOTORS; m++) {
    motorhat_set_motor_speed(s_handle, m, 0);
    motorhat_set_motor_direction(s_handle, m, MOTORHAT_DIRECTION_BRAKE);
  }
  return ESP_OK;
}

// Motor control to be used by socket command functions
esp_err_t motorhat_set_motor_speed(motorhat_handle_t* handle,
                                   motorhat_motor_t motor, uint16_t speed) {
  if (handle == NULL || motor < MOTORHAT_MOTOR1 ||
      motor >= MOTORHAT_NUM_MOTORS) {
    return ESP_ERR_INVALID_ARG;
  }

  if (xEventGroupGetBits(g_motor_events) & CURRENT_ANY) {
    ESP_LOGW(TAG, "Motor %d in fault state, cannot set speed", motor);
    return ESP_ERR_INVALID_STATE;
  }

  if (speed > PCA9685_PWM_MAX) {
    return ESP_ERR_INVALID_ARG;
  }

  const motorhat_motor_channels_t* channels = &motor_channels[motor];

  return pca9685_set_duty_cycle(&handle->pca9685, channels->pwm_channel, speed);
}

esp_err_t motorhat_set_motor_direction(motorhat_handle_t* handle,
                                       motorhat_motor_t motor,
                                       motorhat_direction_t direction) {
  if (handle == NULL || motor < MOTORHAT_MOTOR1 ||
      motor >= MOTORHAT_NUM_MOTORS) {
    return ESP_ERR_INVALID_ARG;
  }

  if (xEventGroupGetBits(g_motor_events) & CURRENT_ANY) {
    ESP_LOGW(TAG, "Motor %d in fault state, cannot set direction", motor);
    return ESP_ERR_INVALID_STATE;
  }

  const motorhat_motor_channels_t* channels = &motor_channels[motor];

  esp_err_t err;

  switch (direction) {
    case MOTORHAT_DIRECTION_FORWARD:
      err =
          pca9685_digital_write(&handle->pca9685, channels->in1_channel, true);
      if (err != ESP_OK) return err;
      err =
          pca9685_digital_write(&handle->pca9685, channels->in2_channel, false);
      err = ESP_OK;
      break;
    case MOTORHAT_DIRECTION_BACKWARD:
      err =
          pca9685_digital_write(&handle->pca9685, channels->in1_channel, false);
      if (err != ESP_OK) return err;
      err =
          pca9685_digital_write(&handle->pca9685, channels->in2_channel, true);
      err = ESP_OK;
      break;
    case MOTORHAT_DIRECTION_BRAKE:
      err =
          pca9685_digital_write(&handle->pca9685, channels->in1_channel, true);
      if (err != ESP_OK) return err;
      err =
          pca9685_digital_write(&handle->pca9685, channels->in2_channel, true);
      err = ESP_OK;
      break;
    case MOTORHAT_DIRECTION_RELEASE:
      err =
          pca9685_digital_write(&handle->pca9685, channels->in1_channel, false);
      if (err != ESP_OK) return err;
      err =
          pca9685_digital_write(&handle->pca9685, channels->in2_channel, false);
      err = ESP_OK;
      break;
    default:
      return ESP_ERR_INVALID_ARG;
  }

  return err;
}

// Emergency stop on all motors that bypasses normal logic flow
esp_err_t motorhat_emergency_stop(motorhat_handle_t* handle) {
  if (handle == NULL) {
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t first_err = ESP_OK;
  for (int m = MOTORHAT_MOTOR1; m < MOTORHAT_NUM_MOTORS; m++) {
    const motorhat_motor_channels_t* ch = &motor_channels[m];

    // Stop PWM cycle and release both control lines
    esp_err_t err =
        pca9685_set_duty_cycle(&handle->pca9685, ch->pwm_channel, 0);
    if (err != ESP_OK && first_err == ESP_OK) first_err = err;

    err = pca9685_digital_write(&handle->pca9685, ch->in1_channel, false);
    if (err != ESP_OK && first_err == ESP_OK) first_err = err;

    err = pca9685_digital_write(&handle->pca9685, ch->in2_channel, false);
    if (err != ESP_OK && first_err == ESP_OK) first_err = err;
  }
  return first_err;
}
