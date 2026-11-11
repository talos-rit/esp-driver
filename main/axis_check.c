// Axis check: robot connected, both motor HATs, motor supply on.
// Runs instead of the normal firmware (AXIS_CHECK in esp_driver.c) and never
// returns.
//
// 1. I2C scan, then starts HAT 1 (CONFIG_DRIVER_MOTORHAT_ADDRESS) and HAT 2
//    (CONFIG_DRIVER_MOTORHAT2_ADDRESS) and releases every port on both.
// 2. Starts all six encoders and prints their counts.
// 3. For each axis, waits for a key, then drives its motor for a fixed short
//    time (not to an encoder target, since an encoder may not count),
//    forward and then backward. After each pulse it prints the PCA9685
//    readback of that port (proves the right HAT and port were commanded)
//    and how far every encoder moved.
// 4. Prints a summary per axis.
//
// No current sensing: the ADS1015s are removed. The only protection is the
// short pulse, the low speed, and you on the motor supply switch.
#include <stdio.h>
#include <string.h>

#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "encoder.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "motorhat.h"
#include "pca9685.h"
#include "signal_bus.h"

// The check always uses both HATs. Without Second MotorHAT in menuconfig it
// assumes HAT 2 is at 0x61 (A0 bridged)
#ifndef CONFIG_DRIVER_MOTORHAT2_ADDRESS
#define CONFIG_DRIVER_MOTORHAT2_ADDRESS 0x61
#endif

#define TAG "AXIS_CHECK"

#define PULSE_SPEED (PCA9685_PWM_MAX * 50 / 100) 
#define PULSE_MS 800 // short enough that a wrong mapping can't reach a hard stop
#define SETTLE_MS 300 // let the axis coast to a stop before reading encoders
#define MIN_COUNTS 2 // an encoder must move at least this much to count as moving
#define CONSOLE_UART UART_NUM_0

// Which HAT is on the bus for this run:
// 0 = both stacked (HAT 2 must have A0 bridged, so it answers at 0x61)
// 1 = only HAT 1 connected, tests axes 1-4
// 2 = only HAT 2 connected (A0 not bridged, so it answers at 0x60), tests axis 5 and gripper
// With 1 or 2 the single board is driven at CONFIG_DRIVER_MOTORHAT_ADDRESS.
// Never connect both HATs at once without A0 bridged: both would answer 0x60.
#define TEST_HAT 0

typedef struct {
  const char* name;
  motorhat_motor_t port;  // MOTOR1-4 = HAT 1 M1-M4, MOTOR5-8 = HAT 2 M1-M4
  gpio_num_t p0_gpio;
  gpio_num_t p1_gpio;
} axis_t;

// Motors: axes 1-4 on HAT 1 M1-M4, axis 5 on HAT 2 M3, gripper on HAT 2 M4.
// Encoders (P0/P1): A1 36/39, A2 34/35, A3 32/33, A4 25/26, A5 27/14, gripper 13/4.
// Each encoder uses one PCNT unit; the classic ESP32 has 8.
static const axis_t axes[] = {
    {"Axis 1", MOTORHAT_MOTOR1, GPIO_NUM_36, GPIO_NUM_39},
    {"Axis 2", MOTORHAT_MOTOR2, GPIO_NUM_34, GPIO_NUM_35},
    {"Axis 3", MOTORHAT_MOTOR3, GPIO_NUM_32, GPIO_NUM_33},
    {"Axis 4", MOTORHAT_MOTOR4, GPIO_NUM_25, GPIO_NUM_26},
    {"Axis 5", MOTORHAT_MOTOR7, GPIO_NUM_27, GPIO_NUM_14},
    {"Gripper", MOTORHAT_MOTOR8, GPIO_NUM_13, GPIO_NUM_4},
};
#define NUM_AXES (sizeof(axes) / sizeof(axes[0]))

static encoder_handle_t encoders[NUM_AXES];  // same order as axes[]
static bool encoder_ok[NUM_AXES];

static void halt(motorhat_handle_t* hat, const char* why) {
  motorhat_emergency_stop(hat);
  ESP_LOGE(TAG, "STOPPED: %s. All motors released. Reset the board to run again.", why);
  while (1) vTaskDelay(portMAX_DELAY);
}

// Waits for one key typed into idf.py monitor, returned lower case
static char wait_key(void) {
  uart_flush_input(CONSOLE_UART);
  uint8_t c = 0;
  while (uart_read_bytes(CONSOLE_UART, &c, 1, portMAX_DELAY) != 1) {
  }
  return (c >= 'A' && c <= 'Z') ? (char)(c + 32) : (char)c;
}

static void i2c_scan(i2c_master_bus_handle_t bus) {
  ESP_LOGI(TAG, "I2C scan (expect 0x%02X HAT 1 and 0x%02X HAT 2; 0x70 is the PCA9685 all-call, also OK):",
           CONFIG_DRIVER_MOTORHAT_ADDRESS, CONFIG_DRIVER_MOTORHAT2_ADDRESS);
  int found = 0;
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    if (i2c_master_probe(bus, addr, 50) == ESP_OK) {
      ESP_LOGI(TAG, "  0x%02X", addr);
      found++;
    }
  }
  if (found == 0) ESP_LOGE(TAG, "  nothing answered. Check SDA GPIO%d, SCL GPIO%d, 3V3 and GND to the HATs.",
                           CONFIG_DRIVER_MOTORHAT_SDA_PIN, CONFIG_DRIVER_MOTORHAT_SCL_PIN);
}

// Reads one PCA9685 channel back. Each channel has 4 registers starting at
// LED0_ON_L + 4*ch. Bit 4 of ON_H = full on, bit 4 of OFF_H = full off.
static const char* channel_state(pca9685_handle_t* pca, pca9685_channel_t ch, char* buf, size_t len) {
  uint8_t r[4];
  for (int i = 0; i < 4; i++) {
    if (pca9685_read_register(pca, (pca9685_register_t)(PCA9685_LED0_ON_L + 4 * ch + i), &r[i]) != ESP_OK) {
      return "ERR";
    }
  }
  if (r[1] & 0x10) return "HIGH";
  if (r[3] & 0x10) return "low";
  snprintf(buf, len, "PWM %d", ((r[3] & 0x0F) << 8) | r[2]);
  return buf;
}

// True if this axis's motor is on a HAT connected for this run
static bool axis_on_bus(const axis_t* a) {
  return TEST_HAT == 0 || a->port / MOTORHAT_MOTORS_PER_HAT == TEST_HAT - 1;
}

// Motor number to command. With one HAT on the bus it is the only one the
// handle holds, so HAT 2 M3 is driven as port M3 of that single board
static motorhat_motor_t drive_port(const axis_t* a) {
  return TEST_HAT == 0 ? a->port : (motorhat_motor_t)(a->port % MOTORHAT_MOTORS_PER_HAT);
}

static void print_port(motorhat_handle_t* hat, const axis_t* a) {
  motorhat_motor_t port = drive_port(a);
  pca9685_handle_t* pca = &hat->pca9685[port / MOTORHAT_MOTORS_PER_HAT];
  const motorhat_motor_channels_t* ch = &motor_channels[port % MOTORHAT_MOTORS_PER_HAT];
  char b1[16], b2[16], b3[16];
  ESP_LOGI(TAG, "  HAT %d M%d readback: IN1 %s, IN2 %s, PWM %s", a->port / MOTORHAT_MOTORS_PER_HAT + 1,
           a->port % MOTORHAT_MOTORS_PER_HAT + 1, channel_state(pca, ch->in1_channel, b1, sizeof(b1)),
           channel_state(pca, ch->in2_channel, b2, sizeof(b2)), channel_state(pca, ch->pwm_channel, b3, sizeof(b3)));
}

static void read_counts(int counts[NUM_AXES]) {
  for (int i = 0; i < NUM_AXES; i++) {
    counts[i] = 0;
    if (encoder_ok[i]) encoder_get_raw_count(&encoders[i], &counts[i]);
  }
}

static void print_counts(const char* label) {
  int now[NUM_AXES];
  read_counts(now);
  char line[192];
  int len = 0;
  for (int i = 0; i < NUM_AXES; i++) {
    len += snprintf(line + len, sizeof(line) - len, "%s %d  ", axes[i].name, now[i]);
  }
  ESP_LOGI(TAG, "%s: %s", label, line);
}

// Drives one axis for PULSE_MS, releases it, and returns how far its own
// encoder moved. Logs the change on every encoder so a crossed wire shows up.
static int run_pulse(motorhat_handle_t* hat, int idx, motorhat_direction_t dir) {
  const axis_t* a = &axes[idx];
  bool fwd = (dir == MOTORHAT_DIRECTION_FORWARD);
  int before[NUM_AXES], after[NUM_AXES];
  read_counts(before);

  motorhat_motor_t port = drive_port(a);
  esp_err_t err = motorhat_set_motor_direction(hat, port, dir);
  if (err == ESP_OK) err = motorhat_set_motor_speed(hat, port, PULSE_SPEED);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "  could not command HAT %d M%d: %s", a->port / MOTORHAT_MOTORS_PER_HAT + 1,
             a->port % MOTORHAT_MOTORS_PER_HAT + 1, esp_err_to_name(err));
    halt(hat, "motor write failed");
  }
  vTaskDelay(pdMS_TO_TICKS(PULSE_MS / 2));
  ESP_LOGI(TAG, "  expect IN1 %s, IN2 %s, PWM %d", fwd ? "HIGH" : "low", fwd ? "low" : "HIGH", PULSE_SPEED);
  print_port(hat, a);  // also takes some I2C time, the pulse stays near PULSE_MS
  vTaskDelay(pdMS_TO_TICKS(PULSE_MS / 2));

  // Brake then release, same as the normal stop, so the axis does not coast far
  motorhat_set_motor_speed(hat, port, 0);
  motorhat_set_motor_direction(hat, port, MOTORHAT_DIRECTION_BRAKE);
  vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));
  motorhat_set_motor_direction(hat, port, MOTORHAT_DIRECTION_RELEASE);
  read_counts(after);

  for (int i = 0; i < NUM_AXES; i++) {
    int d = after[i] - before[i];
    if (i == idx) {
      ESP_LOGI(TAG, "  %s encoder %+d %s", axes[i].name, d,
               !encoder_ok[i] ? "(encoder not started)" : (d >= MIN_COUNTS || d <= -MIN_COUNTS) ? "" : "<- barely moved");
    } else if (d >= MIN_COUNTS || d <= -MIN_COUNTS) {
      ESP_LOGW(TAG, "  %s encoder also moved %+d (crossed encoder wires? small amounts OK on the wrist)",
               axes[i].name, d);
    }
  }
  return after[idx] - before[idx];
}

void axis_check(void) {
  ESP_LOGW(TAG, "AXIS CHECK BUILD - normal firmware is not running. Keep a hand on the motor supply switch.");
  ESP_ERROR_CHECK(signal_bus_init());  // motorhat speed/direction read its fault bits
  ESP_ERROR_CHECK(uart_driver_install(CONSOLE_UART, 256, 0, 0, NULL, 0));

  i2c_bus_t bus;
  i2c_bus_config_t bus_cfg = {
      .port = I2C_NUM_0,
      .sda_io_num = CONFIG_DRIVER_MOTORHAT_SDA_PIN,
      .scl_io_num = CONFIG_DRIVER_MOTORHAT_SCL_PIN,
  };
  ESP_ERROR_CHECK(i2c_bus_init(&bus, &bus_cfg));
  i2c_scan(bus.handle);

  // Built by hand rather than motorhat_init so no soft limit or stop tasks run.
  // With one HAT on the bus it is the only board in the handle, at the first address
  const int num_hats = TEST_HAT == 0 ? MOTORHAT_MAX_HATS : 1;
  const uint8_t addr[MOTORHAT_MAX_HATS] = {CONFIG_DRIVER_MOTORHAT_ADDRESS, CONFIG_DRIVER_MOTORHAT2_ADDRESS};
  if (TEST_HAT == 0) {
    ESP_LOGI(TAG, "Testing both HATs (TEST_HAT 0)");
  } else {
    ESP_LOGW(TAG, "Testing ONLY HAT %d at 0x%02X (TEST_HAT %d). The other HAT must be off the bus.", TEST_HAT,
             addr[0], TEST_HAT);
  }
  motorhat_handle_t hat = {0};
  for (int h = 0; h < num_hats; h++) {
    pca9685_config_t cfg = {
        .i2c_addr = addr[h],
        .i2c_speed_hz = 400000,
        .pwm_freq_hz = DEFAULT_FREQUENCY_HZ,
        .bus_handle = bus.handle,
    };
    int label = TEST_HAT == 0 ? h + 1 : TEST_HAT;
    esp_err_t err = pca9685_init(&hat.pca9685[h], &cfg);
    if (err != ESP_OK) {
      // Stop what already started so a half-working setup never drives
      hat.num_hats = h;
      hat.num_motors = h * MOTORHAT_MOTORS_PER_HAT;
      motorhat_emergency_stop(&hat);
      ESP_LOGE(TAG, "HAT %d at 0x%02X did not start (%s). Check its A0 jumper, stacking header, 3V3/GND.",
               label, addr[h], esp_err_to_name(err));
      ESP_LOGE(TAG, "Cut motor power: a HAT that did not start may still hold its last outputs.");
      while (1) vTaskDelay(portMAX_DELAY);
    }
    ESP_LOGI(TAG, "HAT %d at 0x%02X ready", label, addr[h]);
  }
  hat.num_hats = num_hats;
  hat.num_motors = num_hats * MOTORHAT_MOTORS_PER_HAT;
  motorhat_emergency_stop(&hat);

  for (int i = 0; i < NUM_AXES; i++) {
    encoder_config_t cfg = {
        .P0_pin = axes[i].p0_gpio,
        .P1_pin = axes[i].p1_gpio,
        .glitch_filter_ns = CONFIG_ENCODER_GLITCH_FILTER,
        .resolution = CONFIG_ENCODER_0_RESOLUTION,
    };
    esp_err_t err = encoder_init(&encoders[i], &cfg);
    if (err == ESP_OK) err = encoder_start(&encoders[i]);
    encoder_ok[i] = (err == ESP_OK);
    if (!encoder_ok[i]) {
      // Keep going: the motor of this axis can still be tested
      ESP_LOGE(TAG, "%s encoder GPIO%d/%d did not start: %s", axes[i].name, axes[i].p0_gpio, axes[i].p1_gpio,
               esp_err_to_name(err));
    }
  }
  print_counts("Encoders at rest");

  int fwd[NUM_AXES] = {0}, bwd[NUM_AXES] = {0};
  bool tested[NUM_AXES] = {false};

  for (int i = 0; i < NUM_AXES; i++) {
    const axis_t* a = &axes[i];
    if (!axis_on_bus(a)) continue;  // its HAT is not connected this run
    ESP_LOGI(TAG, "---- %s: HAT %d M%d, encoder GPIO%d/%d ----", a->name, a->port / MOTORHAT_MOTORS_PER_HAT + 1,
             a->port % MOTORHAT_MOTORS_PER_HAT + 1, a->p0_gpio, a->p1_gpio);
    ESP_LOGI(TAG, "Watch the arm. Enter = %d ms FORWARD pulse, s = skip axis, q = quit", PULSE_MS);
    char k = wait_key();
    if (k == 'q') halt(&hat, "quit");
    if (k == 's') continue;
    fwd[i] = run_pulse(&hat, i, MOTORHAT_DIRECTION_FORWARD);

    ESP_LOGI(TAG, "Enter = %d ms BACKWARD pulse (moves it back), q = quit", PULSE_MS);
    k = wait_key();
    if (k == 'q') halt(&hat, "quit");
    bwd[i] = run_pulse(&hat, i, MOTORHAT_DIRECTION_BACKWARD);
    tested[i] = true;
  }

  motorhat_emergency_stop(&hat);
  ESP_LOGI(TAG, "==== Summary (did the arm move? only you can see that) ====");
  for (int i = 0; i < NUM_AXES; i++) {
    const axis_t* a = &axes[i];
    int hat_n = a->port / MOTORHAT_MOTORS_PER_HAT + 1, m = a->port % MOTORHAT_MOTORS_PER_HAT + 1;
    if (!axis_on_bus(a)) {
      ESP_LOGI(TAG, "  %-7s HAT %d M%d: not tested, HAT %d not connected this run", a->name, hat_n, m, hat_n);
      continue;
    }
    if (!tested[i]) {
      ESP_LOGI(TAG, "  %-7s HAT %d M%d: skipped", a->name, hat_n, m);
      continue;
    }
    const char* verdict = "encoder did not follow (motor moved? then check the encoder wiring)";
    if (fwd[i] >= MIN_COUNTS && bwd[i] <= -MIN_COUNTS) verdict = "OK, FORWARD increases count";
    if (fwd[i] <= -MIN_COUNTS && bwd[i] >= MIN_COUNTS) verdict = "OK, FORWARD decreases count";
    ESP_LOGI(TAG, "  %-7s HAT %d M%d: forward %+d, backward %+d -> %s", a->name, hat_n, m, fwd[i], bwd[i], verdict);
  }
  print_counts("Encoders now");
  ESP_LOGI(TAG, "Done, all motors released. Reset to run again, or set AXIS_CHECK 0 for the normal firmware.");
  while (1) vTaskDelay(portMAX_DELAY);
}
