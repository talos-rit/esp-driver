#include "motorhat.h"
#include "motorhat_servo.h"

#include <inttypes.h>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/semphr.h"
#include "signal_bus.h"

#define TAG "motorhat"

static motorhat_handle_t* s_handle = NULL;

// Last direction commanded on each axis
static motorhat_direction_t s_axis_direction[MOTORHAT_NUM_AXES];

// Tick of the last start command that drove each axis, for the max run check
static TickType_t s_axis_started[MOTORHAT_NUM_AXES];

// Held while checking soft limits and writing axis motor state. Check and motor
// writes that follow can't be intertwined with another task
static SemaphoreHandle_t s_axis_lock = NULL;

// Set by motorhat_home_cancel, read by every wait in motorhat_home. Only motorhat_home clears it
static volatile bool s_home_cancel = false;

static bool soft_limit_allows(motorhat_axis_t axis, motorhat_direction_t direction);

// Defined below with the other soft limit functions, started by motorat_init
static void soft_limit_monitor_task(void* args);

// Motor N lives on HAT N / 4, port N % 4
static pca9685_handle_t* motor_chip(motorhat_handle_t* handle,
                                    motorhat_motor_t motor) {
  return &handle->pca9685[motor / MOTORHAT_MOTORS_PER_HAT];
}

static const motorhat_motor_channels_t* motor_port(motorhat_motor_t motor) {
  return &motor_channels[motor % MOTORHAT_MOTORS_PER_HAT];
}

static motorhat_direction_t delta_to_direction(int8_t delta) {
  if (delta > 0) return MOTORHAT_DIRECTION_FORWARD;
  if (delta < 0) return MOTORHAT_DIRECTION_BACKWARD;

  return MOTORHAT_DIRECTION_RELEASE;
}

static motorhat_direction_t reverse_direction(motorhat_direction_t direction) {
  if (direction == MOTORHAT_DIRECTION_FORWARD)
    return MOTORHAT_DIRECTION_BACKWARD;
  if (direction == MOTORHAT_DIRECTION_BACKWARD)
    return MOTORHAT_DIRECTION_FORWARD;

  return direction;
}

void motor_stop_task(void* args) {
  while (1) {
    // Block indefinitely until any stop bit is set
    xEventGroupWaitBits(g_motor_events, CURRENT_ANY,
                        pdFALSE,  // don't clear bits on exit
                        pdFALSE,  // any bit (OR)
                        portMAX_DELAY);

    // Check for homing sequence before reacting
    if (!(xEventGroupGetBits(g_motor_events) & HOMING_FLAG)) {
      // Stop once, then clear the bits so the next start command can run. The
      // current sensor sets them again while the overcurrent lasts.
      xSemaphoreTake(s_axis_lock, portMAX_DELAY);
      motorhat_emergency_stop(s_handle);
      motorhat_servo_halt();
      for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < MOTORHAT_NUM_AXES;
           axis++) {
        s_axis_direction[axis] = MOTORHAT_DIRECTION_RELEASE;
      }
      xEventGroupClearBits(g_motor_events, CURRENT_ANY);
      xSemaphoreGive(s_axis_lock);
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

  if (config->num_hats < 1 || config->num_hats > MOTORHAT_MAX_HATS) {
    ESP_LOGE(TAG, "num_hats %d, must be 1 to %d", config->num_hats,
             MOTORHAT_MAX_HATS);
    return ESP_ERR_INVALID_ARG;
  }
  // Two HATs at one address would both act on every write
  for (int hat = 1; hat < config->num_hats; hat++) {
    for (int other = 0; other < hat; other++) {
      if (config->pca9685_config[hat].i2c_addr ==
          config->pca9685_config[other].i2c_addr) {
        ESP_LOGE(TAG, "HATs %d and %d both at address 0x%02X", other + 1,
                 hat + 1, config->pca9685_config[hat].i2c_addr);
        return ESP_ERR_INVALID_ARG;
      }
    }
  }
  int num_motors = config->num_hats * MOTORHAT_MOTORS_PER_HAT;

  // Polar pan always needs azimuth and altitude
  if (!config->axis_enabled[MOTORHAT_AXIS_AZIMUTH] ||
      !config->axis_enabled[MOTORHAT_AXIS_ALTITUDE]) {
    ESP_LOGE(TAG, "Azimuth and altitude axes must be enabled");
    return ESP_ERR_INVALID_ARG;
  }

  // Each axis needs its own valid port, or two axes would fight over one motor
  for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < MOTORHAT_NUM_AXES; axis++) {
    if (!config->axis_enabled[axis]) {
      continue;
    }
    motorhat_motor_t motor = config->axis_motor[axis];
    if (motor < MOTORHAT_MOTOR1 || motor >= num_motors) {
      ESP_LOGE(TAG, "Axis %d mapped to invalid motor %d", axis, motor + 1);
      return ESP_ERR_INVALID_ARG;
    }
    if (config->axis_speed[axis] > PCA9685_PWM_MAX) {
      ESP_LOGE(TAG, "Axis %d speed %d above the maximum %d", axis,
               config->axis_speed[axis], PCA9685_PWM_MAX);
      return ESP_ERR_INVALID_ARG;
    }
    for (int other = MOTORHAT_AXIS_AZIMUTH; other < axis; other++) {
      if (config->axis_enabled[other] && config->axis_motor[other] == motor) {
        ESP_LOGE(TAG, "Axes %d and %d both mapped to motor %d", other, axis,
                 motor + 1);
        return ESP_ERR_INVALID_ARG;
      }
    }
  }

  if (config->soft_limits_enabled) {
    if (config->axis_count_cb == NULL) {
      ESP_LOGE(TAG, "Soft limits enabled but no axis count callback given");
      return ESP_ERR_INVALID_ARG;
    }

    for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < MOTORHAT_NUM_AXES; axis++) {
      if (!config->axis_enabled[axis]) {
        continue;
      }
      int count = 0;
      esp_err_t err =
          config->axis_count_cb(config->axis_count_ctx, axis, &count);
      if (err != ESP_OK) {
        ESP_LOGE(TAG, "Axis %d count read failed: %s", axis,
                 esp_err_to_name(err));
        return err;
      }

      if (config->soft_limits[axis].min_count > 0 ||
          config->soft_limits[axis].max_count < 0) {
        ESP_LOGE(TAG, "Axis %d soft limits do not contain the boot position",
                 axis);
        return ESP_ERR_INVALID_ARG;
      }
    }
  }

  s_axis_lock = xSemaphoreCreateMutex();
  if (s_axis_lock == NULL) {
    return ESP_ERR_NO_MEM;
  }

  // FORWARD is 0 so the zero initialized array must be set explicitly
  for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < MOTORHAT_NUM_AXES; axis++) {
    s_axis_direction[axis] = MOTORHAT_DIRECTION_RELEASE;
  }

  s_handle = handle;
  handle->polar_pan_speed = config->polar_pan_speed;
  handle->encoder_cb = config->encoder_cb;
  handle->home_step_cb = config->home_step_cb;
  handle->encoder_ctx = config->encoder_ctx;
  handle->limit_gpio = config->limit_gpio;

  handle->soft_limits_enabled = config->soft_limits_enabled;
  handle->homing_enabled = config->homing_enabled;
  handle->axis_count_cb = config->axis_count_cb;
  handle->axis_count_ctx = config->axis_count_ctx;
  for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < MOTORHAT_NUM_AXES; axis++) {
    handle->soft_limits[axis] = config->soft_limits[axis];
    handle->forward_increases_count[axis] =
        config->forward_increases_count[axis];
    handle->axis_motor[axis] = config->axis_motor[axis];
    handle->axis_enabled[axis] = config->axis_enabled[axis];
    handle->axis_speed[axis] = config->axis_speed[axis];
    handle->axis_speed_max[axis] = config->axis_speed[axis];
  }
  handle->soft_limit_poll_ms = config->soft_limit_poll_ms;
  handle->max_run_ms = config->max_run_ms;
  handle->altitude_b_reversed = config->altitude_b_reversed;
  handle->num_hats = config->num_hats;
  handle->num_motors = num_motors;
  handle->servo_enabled = config->servo.enabled;

  // Each init forces that HAT's outputs off first, so a motor left running
  // through an ESP32 reset stops on every HAT
  for (int hat = 0; hat < config->num_hats; hat++) {
    esp_err_t err =
        pca9685_init(&handle->pca9685[hat], &config->pca9685_config[hat]);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "HAT %d at 0x%02X init failed: %s", hat + 1,
               config->pca9685_config[hat].i2c_addr, esp_err_to_name(err));
      return err;
    }
    ESP_LOGI(TAG, "HAT %d at 0x%02X ready, motors M%d-M%d", hat + 1,
             config->pca9685_config[hat].i2c_addr,
             hat * MOTORHAT_MOTORS_PER_HAT + 1,
             (hat + 1) * MOTORHAT_MOTORS_PER_HAT);
  }

  // Started after every PCA9685 is ready; it may emergency stop at once.
  xTaskCreate(motor_stop_task, "motor_stop_task", 4096, handle, 8, NULL);

  // The closed loop does soft limits and the max run time itself
  if (handle->servo_enabled) {
    esp_err_t err = motorhat_servo_init(handle, &config->servo);
    if (err != ESP_OK) {
      return err;
    }
  } else if (handle->soft_limits_enabled || handle->max_run_ms > 0) {
    // Watches moving axes for soft limits and the max run time
    if (xTaskCreate(soft_limit_monitor_task, "soft_limit_task", 4096, NULL, 9,
                    NULL) != pdPASS) {
      return ESP_ERR_NO_MEM;
    }
  }

  return ESP_OK;
}

// Ends a homing run. It stops every motor and lets other commands run again
static esp_err_t home_cancelled(void) {
  ESP_LOGW(TAG, "Homing cancelled");
  motorhat_emergency_stop(s_handle);
  s_home_cancel = false;
  xEventGroupClearBits(g_motor_events, HOMING_FLAG);
  return ESP_ERR_NOT_FINISHED;
}

// Returns 0 on a soft limit or a cancel. check s_home_cancel to tell them apart
static EventBits_t home_wait(motorhat_axis_t axis, motorhat_direction_t direction) {
  while (1) {
    EventBits_t bits = xEventGroupWaitBits(g_motor_events, HOME_EVENT, pdTRUE, pdFALSE, pdMS_TO_TICKS(10));
    if (bits & HOME_EVENT) {
      return bits;
    }
    if (s_home_cancel) {
      return 0;
    }
    if (s_handle->soft_limits_enabled && !soft_limit_allows(axis, direction)) {
      return 0;
    }
  }
}

// How long each nudge runs when getting off a pressed limit switch
#define HOME_NUDGE_MS 500

// Homing drives each axis at its own Speed from the Motors menu, since some
// axes (the elbow lifting the forearm) need more than others to move at all
static uint16_t home_speed(motorhat_axis_t axis) {
  return s_handle->axis_speed_max[axis];
}

// All limit switches share one pin, which is low while any of them is pressed
static bool limit_pressed(void) {
  return gpio_get_level(s_handle->limit_gpio) == 0;
}

// Tells whoever is listening (the Pi link) about a homing step
static void home_step(const char* step, int axis) {
  if (s_handle->home_step_cb) {
    s_handle->home_step_cb(step, axis);
  }
}

// Drives one axis for up to ms. Stops early once no switch is pressed, on a
// cancel or on an overcurrent
static void nudge(motorhat_axis_t axis, motorhat_direction_t direction, int ms) {
  int m = s_handle->axis_motor[axis];
  motorhat_set_motor_direction(s_handle, m, direction);
  motorhat_set_motor_speed(s_handle, m, home_speed(axis));

  for (int t = 0; t < ms && limit_pressed(); t += 10) {
    if (s_home_cancel || (xEventGroupGetBits(g_motor_events) & CURRENT_ANY)) {
      break;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  motorhat_set_motor_speed(s_handle, m, 0);
  motorhat_set_motor_direction(s_handle, m, MOTORHAT_DIRECTION_BRAKE);
}

// A pressed switch hides every other switch since they share one pin, so
// homing would never see the switch it is looking for. Before homing, nudge
// each axis forward and then backward until no switch is pressed. The nudges
// are short so no axis goes far
static esp_err_t get_off_limit_switch(void) {
  bool was_pressed = limit_pressed();
  for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < MOTORHAT_NUM_AXES; axis++) {
    if (!limit_pressed()) {
      break;
    }
    if (!s_handle->axis_enabled[axis]) {
      continue;
    }
    ESP_LOGI(TAG, "A limit switch is pressed, nudging axis %d", axis);
    home_step("NUDGE", axis);

    // Forward, then back past where it started, then forward to the start
    nudge(axis, MOTORHAT_DIRECTION_FORWARD, HOME_NUDGE_MS);
    if (limit_pressed()) {
      nudge(axis, MOTORHAT_DIRECTION_BACKWARD, 2 * HOME_NUDGE_MS);
    }
    if (limit_pressed()) {
      nudge(axis, MOTORHAT_DIRECTION_FORWARD, HOME_NUDGE_MS);
    }

    if (s_home_cancel) {
      return ESP_ERR_NOT_FINISHED;
    }
    if (xEventGroupGetBits(g_motor_events) & CURRENT_ANY) {
      return MOTORHAT_ERR_HOME_OVERCURRENT;
    }
  }

  if (limit_pressed()) {
    ESP_LOGW(TAG, "Still on a limit switch after nudging every axis");
    home_step("STILL_ON_SWITCH", -1);
    return MOTORHAT_ERR_HOME_LIMIT;
  }
  if (was_pressed) {
    home_step("OFF_SWITCH", -1);
  }

  // Let the switch settle, then forget it was pressed so homing starts clean
  vTaskDelay(pdMS_TO_TICKS(50));
  xEventGroupClearBits(g_motor_events, LIMIT_ANY);
  return ESP_OK;
}

// Give up backing off a switch after this long so a stalled motor stops
#define HOME_BACKOFF_MS 3000
// After each axis we wait this long for it to settle, then check the switches
#define HOME_RECHECK_MS 300
// How long one axis may drive when backing off a switch again
#define HOME_RECLEAR_MS 1000
// The switch must read released this long in a row before the motor stops.
// This also drives the axis a little past the switch edge, so it does not
// sag back onto the switch once it is braked (the wrist did)
#define HOME_CLEAR_MS 300
// After stopping we wait this long and check the axis did not fall back on
#define HOME_SETTLE_MS 100

// Slowly drives an axis off the limit switch it just hit. Half speed is slow
// enough to stop near the switch edge but still strong enough for the wrist.
// It keeps going until the switch stays released, then brakes and checks
// again so the next axis never starts with a switch pressed
static esp_err_t back_off_switch(motorhat_axis_t axis, motorhat_direction_t direction,
                                 int timeout_ms) {
  int m = s_handle->axis_motor[axis];
  TickType_t start = xTaskGetTickCount();
  bool driving = false;
  int clear_ms = 0;

  while (1) {
    if (s_home_cancel) {
      return ESP_ERR_NOT_FINISHED;
    }
    if (xTaskGetTickCount() - start > pdMS_TO_TICKS(timeout_ms)) {
      motorhat_set_motor_speed(s_handle, m, 0);
      motorhat_set_motor_direction(s_handle, m, MOTORHAT_DIRECTION_BRAKE);
      return MOTORHAT_ERR_HOME_LIMIT;
    }

    if (!driving) {
      motorhat_set_motor_direction(s_handle, m, direction);
      motorhat_set_motor_speed(s_handle, m, home_speed(axis) / 2);
      driving = true;
      clear_ms = 0;
    }

    if (limit_pressed()) {
      clear_ms = 0;  // A bounce or still on it, so start counting again
    } else {
      clear_ms += 10;
    }

    if (clear_ms >= HOME_CLEAR_MS) {
      // Brake so gravity does not pull it back onto the switch
      motorhat_set_motor_speed(s_handle, m, 0);
      motorhat_set_motor_direction(s_handle, m, MOTORHAT_DIRECTION_BRAKE);
      driving = false;
      vTaskDelay(pdMS_TO_TICKS(HOME_SETTLE_MS));
      if (!limit_pressed()) {
        break;
      }
      // It settled back onto the switch, so drive off it again
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  // Forget the presses from this axis so the next one starts clean
  xEventGroupClearBits(g_motor_events, LIMIT_ANY);
  return ESP_OK;
}

// Which way each homed axis drove to get off its switch
static motorhat_direction_t s_off_switch_dir[MOTORHAT_NUM_AXES];

// Ends a homing run that failed. It stops every motor and returns err
// Unused while the homing faults are off, so mark it to keep the build happy
__attribute__((unused)) static esp_err_t home_failed(esp_err_t err) {
  motorhat_emergency_stop(s_handle);
  s_home_cancel = false;
  xEventGroupClearBits(g_motor_events, HOMING_FLAG);
  return err;
}

// After an axis is homed, wait for it to settle and make sure no switch is
// pressed. If one is, back it off again. The wrist is the usual one since its
// two motors move one joint, so homing motor 5 can push the wrist back onto
// the switch motor 4 just left. We only try the axis just homed and its wrist
// partner. Moving other axes would shift their home
static esp_err_t stay_off_switches(motorhat_axis_t homed) {
  vTaskDelay(pdMS_TO_TICKS(HOME_RECHECK_MS));
  if (!limit_pressed()) {
    return ESP_OK;
  }

  int partner = -1;
  if (homed == MOTORHAT_AXIS_ALTITUDE_B) {
    partner = MOTORHAT_AXIS_ALTITUDE;
  }

  int order[MOTORHAT_NUM_AXES];
  int n = 0;
  order[n++] = homed;
  if (partner >= 0 && s_handle->axis_enabled[partner]) {
    order[n++] = partner;
  }

  for (int i = 0; i < n && limit_pressed(); i++) {
    ESP_LOGW(TAG, "A limit switch is pressed, backing axis %d off again", order[i]);
    home_step("BACK_OFF_AGAIN", order[i]);
    esp_err_t err = back_off_switch(order[i], s_off_switch_dir[order[i]], HOME_RECLEAR_MS);
    if (err == ESP_ERR_NOT_FINISHED) {
      return err;
    }
    // A timeout means it was not this axis, so try the partner
  }

  if (limit_pressed()) {
    ESP_LOGW(TAG, "Still on a limit switch after homing axis %d", homed);
    home_step("STILL_ON_SWITCH", -1);
    return MOTORHAT_ERR_HOME_LIMIT;
  }
  xEventGroupClearBits(g_motor_events, LIMIT_ANY);
  return ESP_OK;
}

// Socket commands
esp_err_t motorhat_home(uint16_t delay_ms) {
  if (xEventGroupGetBits(g_motor_events) & HOMING_FLAG) {
    ESP_LOGW(TAG, "Homing in progress, cannot send commands");
    return ESP_ERR_INVALID_STATE;
  }

  if (s_handle == NULL) return ESP_ERR_INVALID_STATE;

  if (!s_handle->homing_enabled) {
    ESP_LOGW(TAG, "Homing disabled in menuconfig, home command rejected");
    return ESP_ERR_NOT_SUPPORTED;
  }

  ESP_LOGI(TAG, "Received home command: delay_ms=%d", delay_ms);

  // Set homing flag
  xEventGroupSetBits(g_motor_events, HOMING_FLAG);

  // Add delay
  vTaskDelay(pdMS_TO_TICKS(delay_ms));
  if (s_home_cancel) {
    return home_cancelled();
  }

  // Stop any movement before homing
  for (int m = MOTORHAT_MOTOR1; m < s_handle->num_motors; m++) {
    motorhat_set_motor_speed(s_handle, m, 0);
    motorhat_set_motor_direction(s_handle, m, MOTORHAT_DIRECTION_RELEASE);
  }

  esp_err_t err = get_off_limit_switch();
  if (err == ESP_ERR_NOT_FINISHED) {
    return home_cancelled();
  }
  // HOMING FAULTS OFF for the Maker Faire. Not safe, put the return back after
  // if (err != ESP_OK) {
  //   motorhat_emergency_stop(s_handle);
  //   s_home_cancel = false;
  //   xEventGroupClearBits(g_motor_events, HOMING_FLAG);
  //   return err;
  // }

  // Home each axis individually until we candetermine which motor triggers
  // limit switch and current events
  for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < MOTORHAT_NUM_AXES; axis++) {
    if (!s_handle->axis_enabled[axis]) {
      continue;
    }
    if (s_home_cancel) {
      return home_cancelled();
    }
    int m = s_handle->axis_motor[axis];
    // A pressed switch would hide this axis's switch, so never start on one
    if (limit_pressed()) {
      ESP_LOGW(TAG, "A limit switch is pressed before homing motor %d", m);
      home_step("STILL_ON_SWITCH", -1);
      // HOMING FAULTS OFF for the Maker Faire. Not safe, put the return back after
      // motorhat_emergency_stop(s_handle);
      // s_home_cancel = false;
      // xEventGroupClearBits(g_motor_events, HOMING_FLAG);
      // return MOTORHAT_ERR_HOME_LIMIT;
    }
    ESP_LOGI(TAG, "Homing motor %d", m);
    home_step("AXIS_START", axis);

    motorhat_set_motor_direction(s_handle, m, MOTORHAT_DIRECTION_FORWARD);
    motorhat_set_motor_speed(s_handle, m, home_speed(axis));

    EventBits_t bits = home_wait(axis, MOTORHAT_DIRECTION_FORWARD);
    if (s_home_cancel) {
      return home_cancelled();
    }

    // Stop when a homing event is detected
    motorhat_set_motor_speed(s_handle, m, 0);

    if (bits == 0 || (bits & CURRENT_ANY)) {
      ESP_LOGI(TAG, "Motor %d hit the forward end of its range", m);
      home_step("RANGE_END", axis);

      // Small pause before going opposite direction
      vTaskDelay(pdMS_TO_TICKS(200));
      if (s_home_cancel) {
        return home_cancelled();
      }

      // Switch directions and move towards limit switch
      motorhat_set_motor_direction(s_handle, m, MOTORHAT_DIRECTION_BACKWARD);
      motorhat_set_motor_speed(s_handle, m, home_speed(axis));

      bits = home_wait(axis, MOTORHAT_DIRECTION_BACKWARD);
      if (s_home_cancel) {
        return home_cancelled();
      }

      // Stop when a homing event is detected
      motorhat_set_motor_speed(s_handle, m, 0);

      if (bits & LIMIT_ANY) {
        ESP_LOGI(TAG, "Motor %d hit a limit switch moving backward", m);
        home_step("SWITCH_HIT", axis);

        // Go forward slowly until the limit switch releases
        s_off_switch_dir[axis] = MOTORHAT_DIRECTION_FORWARD;
        err = back_off_switch(axis, MOTORHAT_DIRECTION_FORWARD, HOME_BACKOFF_MS);
        if (err == ESP_ERR_NOT_FINISHED) {
          return home_cancelled();
        }
        if (err != ESP_OK) {
          ESP_LOGW(TAG, "Motor %d could not back off the limit switch", m);
          home_step("STILL_ON_SWITCH", axis);
          // HOMING FAULTS OFF for the Maker Faire. Not safe, put the return back after
          // return home_failed(err);
        }
        // back_off_switch left the motor braked so it stays off the switch
        ESP_LOGI(TAG, "Motor %d finished homing", m);
        home_step("AXIS_DONE", axis);

      } else {
        // Overcurrent, or a soft limit (bits 0), before reaching the switch
        ESP_LOGW(TAG, "Motor %d did not find the limit switch moving backward", m);
        home_step("NO_SWITCH", axis);
        // Stop just this motor and go on to the next axis
        motorhat_set_motor_direction(s_handle, m, MOTORHAT_DIRECTION_BRAKE);
        // HOMING FAULTS OFF for the Maker Faire. Not safe, put the return back after
        // motorhat_emergency_stop(s_handle);
        // s_home_cancel = false;
        // xEventGroupClearBits(g_motor_events, HOMING_FLAG);
        // if (bits & CURRENT_ANY) {
        //   return MOTORHAT_ERR_HOME_OVERCURRENT;
        // }
        // return MOTORHAT_ERR_HOME_LIMIT;
      }

    } else if (bits & LIMIT_ANY) {
      ESP_LOGI(TAG, "Motor %d hit a limit switch moving forward", m);
      home_step("SWITCH_HIT", axis);

      // Small pause before going the opposite direction
      vTaskDelay(pdMS_TO_TICKS(200));
      if (s_home_cancel) {
        return home_cancelled();
      }

      // Turn around and move slowly until the limit switch releases
      s_off_switch_dir[axis] = MOTORHAT_DIRECTION_BACKWARD;
      err = back_off_switch(axis, MOTORHAT_DIRECTION_BACKWARD, HOME_BACKOFF_MS);
      if (err == ESP_ERR_NOT_FINISHED) {
        return home_cancelled();
      }
      if (err != ESP_OK) {
        ESP_LOGW(TAG, "Motor %d could not back off the limit switch", m);
        home_step("STILL_ON_SWITCH", axis);
        // HOMING FAULTS OFF for the Maker Faire. Not safe, put the return back after
        // return home_failed(err);
      }
      // back_off_switch left the motor braked so it stays off the switch
      ESP_LOGI(TAG, "Motor %d finished homing", m);
      home_step("AXIS_DONE", axis);

    } else {
      // Can't happen today: home_wait only returns 0, current bits or limit
      // bits, and the branches above take all three. If it ever does, fail
      // rather than fall through and zero the encoders away from home
      ESP_LOGW(TAG, "Motor %d unexpected homing event detected", m);
      motorhat_emergency_stop(s_handle);
      s_home_cancel = false;
      xEventGroupClearBits(g_motor_events, HOMING_FLAG);
      return ESP_FAIL;
    }

    // Never leave an axis, or end homing, with a switch pressed
    err = stay_off_switches(axis);
    if (err == ESP_ERR_NOT_FINISHED) {
      return home_cancelled();
    }
    // HOMING FAULTS OFF for the Maker Faire. Not safe, put the return back after
    // if (err != ESP_OK) {
    //   return home_failed(err);
    // }
  }

  // Reset encoder values
  if (s_handle->encoder_cb) {
    s_handle->encoder_cb(s_handle->encoder_ctx);
    ESP_LOGI(TAG, "Homing successful, encoders reset");
    home_step("ENCODERS_ZEROED", -1);
  } else {
    ESP_LOGW(TAG, "Encoder reset callback not found, homing unsuccessful");
  }

  // Clear homing flag. A cancel that came after the last check is dropped, the motors are already stopped
  s_home_cancel = false;
  xEventGroupClearBits(g_motor_events, HOMING_FLAG);

  return ESP_OK;
}

esp_err_t motorhat_home_cancel(void) {
  s_home_cancel = true;
  return ESP_OK;
}

esp_err_t motorhat_polar_pan(int16_t delta_azimuth, int16_t delta_altitude,
                             uint16_t delay_ms, uint16_t time_ms) {
  if (xEventGroupGetBits(g_motor_events) & HOMING_FLAG) {
    ESP_LOGW(TAG, "Homing in progress, cannot send commands");
    return ESP_ERR_INVALID_STATE;
  }

  if (s_handle == NULL) return ESP_ERR_INVALID_STATE;

  ESP_LOGI(TAG,
           "Received polar pan command: delta_azimuth=%d, delta_altitude=%d, "
           "delay_ms=%d, time_ms=%d",
           delta_azimuth, delta_altitude, delay_ms, time_ms);

  // TODO: Implement polar pan logic using motorhat_set_motor_direction and
  // motorhat_set_motor_speed
  return ESP_OK;
}

// Returns true if axis may be driven in this direction.
// BRAKE and RELEASE are always allowed.
// FORWARD and BACKWARD are refused if they would move the count further past
// the soft limit the axis has already reached. Caller must hold s_axis_lock
static bool soft_limit_allows(motorhat_axis_t axis,
                              motorhat_direction_t direction) {
  if (direction != MOTORHAT_DIRECTION_FORWARD &&
      direction != MOTORHAT_DIRECTION_BACKWARD) {
    return true;
  }

  int count = 0;
  esp_err_t err =
      s_handle->axis_count_cb(s_handle->axis_count_ctx, axis, &count);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Axis %d count read failed (%s), refusing to move it", axis,
             esp_err_to_name(err));
    return false;
  }

  // if direction is forward and going forward makes encoder count go up, true
  // if direction is forward and going forward makes encoder go down, false
  // if direction is backward and going forward makes count go up, false
  // if direction is backward and going forward makes count go down, true
  // works out whether this command pushes the count up or down
  bool count_increases = (direction == MOTORHAT_DIRECTION_FORWARD) ==
                         s_handle->forward_increases_count[axis];
  const motorhat_soft_limit_t* limit = &s_handle->soft_limits[axis];

  if (count_increases && count >= limit->max_count) {
    ESP_LOGW(TAG, "Axis %d at max soft limit (count %d, max %" PRId32 ")", axis,
             count, limit->max_count);
    return false;
  }

  if (!count_increases && count <= limit->min_count) {
    ESP_LOGW(TAG, "Axis %d at min soft limit (count %d, min %" PRId32 ")", axis,
             count, limit->min_count);
    return false;
  }

  return true;
}

// Brakes one axis. If that fails, stop every motor instead so a failed write
// can never leave a motor running.
// Caller must hold s_axis_lock.
static void stop_axis(motorhat_axis_t axis) {
  esp_err_t err =
      motorhat_set_motor_speed(s_handle, s_handle->axis_motor[axis], 0);
  if (err == ESP_OK) {
    err = motorhat_set_motor_direction(s_handle, s_handle->axis_motor[axis],
                                       MOTORHAT_DIRECTION_BRAKE);
  }

  if (err == ESP_OK) {
    s_axis_direction[axis] = MOTORHAT_DIRECTION_BRAKE;
    return;
  }

  ESP_LOGE(TAG, "Axis %d stop failed (%s), stopping all motors", axis,
           esp_err_to_name(err));
  motorhat_emergency_stop(s_handle);
  for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < MOTORHAT_NUM_AXES; axis++) {
    s_axis_direction[axis] = MOTORHAT_DIRECTION_RELEASE;
  }
}

// With two altitude motors the wrist only pitches while both turn, and one
// turning alone rolls it, so stopping either stops both.
// Caller must hold s_axis_lock.
static void stop_axis_and_partner(motorhat_axis_t axis) {
  stop_axis(axis);
  if (!s_handle->axis_enabled[MOTORHAT_AXIS_ALTITUDE_B]) {
    return;
  }
  if (axis == MOTORHAT_AXIS_ALTITUDE) {
    stop_axis(MOTORHAT_AXIS_ALTITUDE_B);
  } else if (axis == MOTORHAT_AXIS_ALTITUDE_B) {
    stop_axis(MOTORHAT_AXIS_ALTITUDE);
  }
}

// Every soft_limit_poll_ms, stops any axis being driven past its soft limit
// (the same rule as polar pan start, soft_limit_allows, so a moving axis stops
// where a new start in that direction would be refused) or driven longer than
// max_run_ms without a new start command.
static void soft_limit_monitor_task(void* args) {
  // At CONFIG_FREERTOS_HZ=100 a tick is 10ms so shorter periods round down to 0
  // ticks which vTaskDelayUntil does not allow
  TickType_t period = pdMS_TO_TICKS(s_handle->soft_limit_poll_ms);
  if (period == 0) {
    period = 1;
  }
  TickType_t max_run_ticks = pdMS_TO_TICKS(s_handle->max_run_ms);

  TickType_t last_wake = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&last_wake, period);

    xSemaphoreTake(s_axis_lock, portMAX_DELAY);
    TickType_t now = xTaskGetTickCount();
    for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < MOTORHAT_NUM_AXES; axis++) {
      if (!s_handle->axis_enabled[axis]) {
        continue;
      }
      motorhat_direction_t direction = s_axis_direction[axis];
      if (direction != MOTORHAT_DIRECTION_FORWARD &&
          direction != MOTORHAT_DIRECTION_BACKWARD) {
        continue;
      }

      if (s_handle->soft_limits_enabled &&
          !soft_limit_allows(axis, direction)) {
        ESP_LOGW(TAG, "Axis %d stopped at soft limit", axis);
        stop_axis_and_partner(axis);
      } else if (max_run_ticks > 0 &&
                 now - s_axis_started[axis] > max_run_ticks) {
        ESP_LOGW(TAG, "Axis %d ran %" PRIu32 " ms without a new start, stopped",
                 axis, s_handle->max_run_ms);
        stop_axis_and_partner(axis);
      }
    }
    xSemaphoreGive(s_axis_lock);
  }
}

// Applies a polar pan start to botha axes. Axes that soft limits refuse are
// brakes and left at speed 0. Caller must hold s_axis_lock.
static esp_err_t start_axes(
    motorhat_direction_t directions[MOTORHAT_NUM_AXES]) {
  bool blocked[MOTORHAT_NUM_AXES] = {false};
  // Disabled axes are skipped by every loop below
  for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < MOTORHAT_NUM_AXES; axis++) {
    if (!s_handle->axis_enabled[axis]) {
      blocked[axis] = true;
    }
  }
  if (s_handle->soft_limits_enabled) {
    for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < MOTORHAT_NUM_AXES; axis++) {
      if (blocked[axis]) {
        continue;
      }
      if (!soft_limit_allows(axis, directions[axis])) {
        directions[axis] = MOTORHAT_DIRECTION_BRAKE;
        blocked[axis] = true;
      }
    }
  }

  // Never drive one of two altitude motors alone, it would roll the wrist
  if (s_handle->axis_enabled[MOTORHAT_AXIS_ALTITUDE_B] &&
      (blocked[MOTORHAT_AXIS_ALTITUDE] || blocked[MOTORHAT_AXIS_ALTITUDE_B])) {
    directions[MOTORHAT_AXIS_ALTITUDE] = MOTORHAT_DIRECTION_BRAKE;
    directions[MOTORHAT_AXIS_ALTITUDE_B] = MOTORHAT_DIRECTION_BRAKE;
    blocked[MOTORHAT_AXIS_ALTITUDE] = true;
    blocked[MOTORHAT_AXIS_ALTITUDE_B] = true;
  }

  // set every axis speed to 0 to prevent direction change while moving
  for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < MOTORHAT_NUM_AXES; axis++) {
    if (!s_handle->axis_enabled[axis]) {
      continue;
    }
    esp_err_t err =
        motorhat_set_motor_speed(s_handle, s_handle->axis_motor[axis], 0);
    if (err != ESP_OK) {
      return err;
    }
  }

  // Set directions
  for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < MOTORHAT_NUM_AXES; axis++) {
    if (!s_handle->axis_enabled[axis]) {
      continue;
    }
    esp_err_t err = motorhat_set_motor_direction(
        s_handle, s_handle->axis_motor[axis], directions[axis]);
    if (err != ESP_OK) {
      return err;
    }
    s_axis_direction[axis] = directions[axis];
  }

  // Set speeds to a default value
  for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < MOTORHAT_NUM_AXES; axis++) {
    if (blocked[axis]) {
      continue;
    }
    esp_err_t err = motorhat_set_motor_speed(
        s_handle, s_handle->axis_motor[axis], s_handle->axis_speed[axis]);
    if (err != ESP_OK) {
      return err;
    }
    s_axis_started[axis] = xTaskGetTickCount();
  }

  return ESP_OK;
}

// Starts azimuth, altitude and elbow together. A 0 stops that axis
static esp_err_t jog_start(int8_t delta_azimuth, int8_t delta_altitude,
                           int8_t delta_elbow) {
  if (xEventGroupGetBits(g_motor_events) & HOMING_FLAG) {
    ESP_LOGW(TAG, "Homing in progress, cannot send commands");
    return ESP_ERR_INVALID_STATE;
  }

  if (s_handle == NULL) return ESP_ERR_INVALID_STATE;

  ESP_LOGI(TAG,
           "Received jog start command: delta_azimuth=%d, delta_altitude=%d, "
           "delta_elbow=%d",
           delta_azimuth, delta_altitude, delta_elbow);

  if (s_handle->servo_enabled) {
    motorhat_servo_start(delta_azimuth, delta_altitude, delta_elbow);
    return ESP_OK;
  }

  motorhat_direction_t directions[MOTORHAT_NUM_AXES] = {
      [MOTORHAT_AXIS_AZIMUTH] = delta_to_direction(delta_azimuth),
      [MOTORHAT_AXIS_ALTITUDE] = delta_to_direction(delta_altitude),
      [MOTORHAT_AXIS_ELBOW] = delta_to_direction(delta_elbow),
      [MOTORHAT_AXIS_ALTITUDE_B] = delta_to_direction(delta_altitude),
  };
  if (s_handle->altitude_b_reversed) {
    directions[MOTORHAT_AXIS_ALTITUDE_B] =
        reverse_direction(directions[MOTORHAT_AXIS_ALTITUDE_B]);
  }

  xSemaphoreTake(s_axis_lock, portMAX_DELAY);
  esp_err_t err = start_axes(directions);
  xSemaphoreGive(s_axis_lock);

  return err;
}

esp_err_t motorhat_polar_pan_start(int8_t delta_azimuth,
                                   int8_t delta_altitude) {
  return jog_start(delta_azimuth, delta_altitude, 0);
}

esp_err_t motorhat_elbow_start(int8_t delta_elbow) {
  if (s_handle == NULL) return ESP_ERR_INVALID_STATE;
  if (!s_handle->axis_enabled[MOTORHAT_AXIS_ELBOW]) {
    return ESP_ERR_NOT_SUPPORTED;
  }
  return jog_start(0, 0, delta_elbow);
}

esp_err_t motorhat_polar_pan_stop(void) {
  if (xEventGroupGetBits(g_motor_events) & HOMING_FLAG) {
    ESP_LOGW(TAG, "Homing in progress, cannot send commands");
    return ESP_ERR_INVALID_STATE;
  }

  if (s_handle == NULL) return ESP_ERR_INVALID_STATE;

  ESP_LOGI(TAG, "Received polar pan stop command");

  if (s_handle->servo_enabled) {
    motorhat_servo_stop();
    return ESP_OK;
  }

  xSemaphoreTake(s_axis_lock, portMAX_DELAY);
  for (int m = MOTORHAT_MOTOR1; m < s_handle->num_motors; m++) {
    motorhat_set_motor_speed(s_handle, m, 0);
    motorhat_set_motor_direction(s_handle, m, MOTORHAT_DIRECTION_BRAKE);
  }
  for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < MOTORHAT_NUM_AXES; axis++) {
    s_axis_direction[axis] = MOTORHAT_DIRECTION_BRAKE;
  }
  xSemaphoreGive(s_axis_lock);
  return ESP_OK;
}

esp_err_t motorhat_set_speed_percent(uint8_t percent) {
  if (s_handle == NULL) {
    return ESP_ERR_INVALID_STATE;
  }
  if (percent < 1 || percent > 100) {
    return ESP_ERR_INVALID_ARG;
  }

  // Closed loop ignores axis_speed for jogging so scale the jog speed instead
  if (s_handle->servo_enabled) {
    motorhat_servo_set_speed_percent(percent);
    ESP_LOGI(TAG, "Jog speed %d%%", percent);
    return ESP_OK;
  }

  // start_axes reads axis_speed under this lock
  xSemaphoreTake(s_axis_lock, portMAX_DELAY);
  for (int axis = MOTORHAT_AXIS_AZIMUTH; axis < MOTORHAT_NUM_AXES; axis++) {
    // A percent of the configured speed, not of the last speed, so  result is at most axis_speed_max
    s_handle->axis_speed[axis] = (uint32_t)s_handle->axis_speed_max[axis] * percent / 100;
  }
  xSemaphoreGive(s_axis_lock);

  ESP_LOGI(TAG, "Pan speed %d%%", percent);
  for (int axis = 0; axis < MOTORHAT_NUM_AXES; axis++) {
    if (s_handle->axis_enabled[axis]) {
      ESP_LOGI(TAG, "    axis %d: duty %d", axis, s_handle->axis_speed[axis]);
    }
  }

  return ESP_OK;
}

// Motor control to be used by socket command functions
esp_err_t motorhat_set_motor_speed(motorhat_handle_t* handle,
                                   motorhat_motor_t motor, uint16_t speed) {
  if (handle == NULL || motor < MOTORHAT_MOTOR1 ||
      motor >= handle->num_motors) {
    return ESP_ERR_INVALID_ARG;
  }

  if (xEventGroupGetBits(g_motor_events) & CURRENT_ANY) {
    ESP_LOGW(TAG, "Motor %d in fault state, cannot set speed", motor);
    return ESP_ERR_INVALID_STATE;
  }

  if (speed > PCA9685_PWM_MAX) {
    return ESP_ERR_INVALID_ARG;
  }

  const motorhat_motor_channels_t* channels = motor_port(motor);
  pca9685_handle_t* chip = motor_chip(handle, motor);

  return pca9685_set_duty_cycle(chip, channels->pwm_channel, speed);
}

esp_err_t motorhat_set_motor_direction(motorhat_handle_t* handle,
                                       motorhat_motor_t motor,
                                       motorhat_direction_t direction) {
  if (handle == NULL || motor < MOTORHAT_MOTOR1 ||
      motor >= handle->num_motors) {
    return ESP_ERR_INVALID_ARG;
  }

  if (xEventGroupGetBits(g_motor_events) & CURRENT_ANY) {
    ESP_LOGW(TAG, "Motor %d in fault state, cannot set direction", motor);
    return ESP_ERR_INVALID_STATE;
  }

  const motorhat_motor_channels_t* channels = motor_port(motor);
  pca9685_handle_t* chip = motor_chip(handle, motor);

  esp_err_t err;

  switch (direction) {
    case MOTORHAT_DIRECTION_FORWARD:
      err =
          pca9685_digital_write(chip, channels->in1_channel, true);
      if (err != ESP_OK) return err;
      err =
          pca9685_digital_write(chip, channels->in2_channel, false);
      err = ESP_OK;
      break;
    case MOTORHAT_DIRECTION_BACKWARD:
      err =
          pca9685_digital_write(chip, channels->in1_channel, false);
      if (err != ESP_OK) return err;
      err =
          pca9685_digital_write(chip, channels->in2_channel, true);
      err = ESP_OK;
      break;
    case MOTORHAT_DIRECTION_BRAKE:
      err =
          pca9685_digital_write(chip, channels->in1_channel, true);
      if (err != ESP_OK) return err;
      err =
          pca9685_digital_write(chip, channels->in2_channel, true);
      err = ESP_OK;
      break;
    case MOTORHAT_DIRECTION_RELEASE:
      err =
          pca9685_digital_write(chip, channels->in1_channel, false);
      if (err != ESP_OK) return err;
      err =
          pca9685_digital_write(chip, channels->in2_channel, false);
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

  // Every port on every HAT. A write that fails on one HAT does not stop the
  // loop, so the other HAT still gets stopped
  esp_err_t first_err = ESP_OK;
  for (int m = MOTORHAT_MOTOR1; m < handle->num_motors; m++) {
    const motorhat_motor_channels_t* ch = motor_port(m);
    pca9685_handle_t* chip = motor_chip(handle, m);

    // Stop PWM cycle and release both control lines
    esp_err_t err = pca9685_set_duty_cycle(chip, ch->pwm_channel, 0);
    if (err != ESP_OK && first_err == ESP_OK) first_err = err;

    err = pca9685_digital_write(chip, ch->in1_channel, false);
    if (err != ESP_OK && first_err == ESP_OK) first_err = err;

    err = pca9685_digital_write(chip, ch->in2_channel, false);
    if (err != ESP_OK && first_err == ESP_OK) first_err = err;
  }
  return first_err;
}
