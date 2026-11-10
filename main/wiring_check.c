// Wiring checks. Both run instead of the normal firmware (see WIRING_CHECK in
// esp_driver.c) and never return.
//
// wiring_check(): ESP32 alone, USB power only, no D50, no motor supply.
// 1. I2C scan: motor HAT and both ADS1015s must answer.
// 2. Motor ports: drives M1, M2, M3 one at a time and reads the PCA9685 back,
//    proving the firmware port numbering. Outputs stay at 0 V without motor power.
// 3. Encoder pins: logs every level change on the six encoder GPIOs.
//
// robot_check(): robot connected, motor supply on. Waits for a key before every
// move, gives each axis a short slow pulse forward and back, and reports encoder
// movement on all three encoders plus the peak current on every ADC pair. Stops
// everything on overcurrent or if the wrong encoder moves.
//
// encoder_watch(): robot connected, motors never driven. Prints all six
// encoder counts (axes 1-5 and gripper) whenever one changes while you move axes by hand.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ads1015.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/uart.h"
#include "encoder.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "i2c_bus.h"
#include "motorhat.h"
#include "pca9685.h"
#include "signal_bus.h"

#define TAG "WIRING"
#define CHECK_SPEED (PCA9685_PWM_MAX / 2)  // 50% duty, shows up as "PWM 2048" in the readback
#define MAX_ADCS 4

typedef struct {
  const char* name;
  motorhat_motor_t port;
  int d50_a;
  int d50_b;
} motor_check_t;

// Wiring: axis 1 -> M1, axis 4 -> M2, axis 5 -> M3, gripper -> M4. D50 pins from the ER-4u manual ch. 8.
static const motor_check_t motors[] = {
    {"Axis 1", MOTORHAT_MOTOR1, 50, 17},
    {"Axis 4", MOTORHAT_MOTOR2, 47, 14},
    {"Axis 5", MOTORHAT_MOTOR3, 46, 13},
    {"Gripper", MOTORHAT_MOTOR4, 45, 12},
};
#define NUM_MOTORS (sizeof(motors) / sizeof(motors[0]))

typedef struct {
  const char* name;
  gpio_num_t gpio;
  int d50;
} enc_pin_t;

static const enc_pin_t enc_pins[] = {
    {"A1 P0", GPIO_NUM_36, 2},  {"A1 P1", GPIO_NUM_39, 5},
    {"A4 P0", GPIO_NUM_25, 35}, {"A4 P1", GPIO_NUM_26, 20},
    {"A5 P0", GPIO_NUM_27, 18}, {"A5 P1", GPIO_NUM_14, 3},
    {"GR P0", GPIO_NUM_13, 34}, {"GR P1", GPIO_NUM_4, 19},
};  // two entries per motors[] row, same order
#define NUM_ENC_PINS (sizeof(enc_pins) / sizeof(enc_pins[0]))

static ads1015_handle_t adcs[MAX_ADCS];
static uint8_t adc_addr[MAX_ADCS];
static int adc_count = 0;

static void i2c_scan(i2c_master_bus_handle_t bus) {
  ESP_LOGI(TAG, "I2C scan (SDA %d, SCL %d)", CONFIG_DRIVER_MOTORHAT_SDA_PIN, CONFIG_DRIVER_MOTORHAT_SCL_PIN);
  for (uint8_t addr = 0x08; addr < 0x78; addr++) {
    if (i2c_master_probe(bus, addr, 50) != ESP_OK) continue;

    const char* what = "unknown";
    if (addr == CONFIG_DRIVER_MOTORHAT_ADDRESS) what = "motor HAT";
    else if (addr == 0x70) what = "PCA9685 all-call (normal)";
    else if (addr >= 0x48 && addr <= 0x4B) what = "ADS1015";
    ESP_LOGI(TAG, "  0x%02X  %s", addr, what);

    if (addr >= 0x48 && addr <= 0x4B && adc_count < MAX_ADCS) {
      i2c_device_config_t dev = {
          .dev_addr_length = I2C_ADDR_BIT_LEN_7,
          .device_address = addr,
          .scl_speed_hz = 400000,
      };
      if (i2c_master_bus_add_device(bus, &dev, &adcs[adc_count].dev_handle) == ESP_OK) {
        adc_addr[adc_count++] = addr;
      }
    }
  }
  if (adc_count != 2) ESP_LOGW(TAG, "Expected 2 ADS1015, found %d", adc_count);
}

// Single-shot differential read, same gain as the firmware (4.096 V, 2 mV per count).
// Comparator disabled so the ALERT/RDY line stays quiet.
static esp_err_t adc_read(ads1015_handle_t* adc, ads1015_mux_t mux, int* counts) {
  uint16_t cfg = ads1015_build_config(mux, ADS1015_PGA_4_096V, ADS1015_MODE_SINGLESHOT,
                                      ADS1015_DR_1600SPS, ADS1015_COMP_TRADITIONAL,
                                      ADS1015_COMP_ACTIVE_LOW, ADS1015_COMP_NON_LATCHING,
                                      ADS1015_COMP_DISABLE, true);
  esp_err_t err = ads1015_write_register(adc, ADS1015_CONFIG, cfg);
  if (err != ESP_OK) return err;

  // Wait for the conversion-done bit (bit 15 reads 1 when idle). One conversion at
  // 1600 SPS takes under 1 ms, so busy-wait instead of a 10 ms FreeRTOS tick.
  uint16_t reg = 0;
  esp_rom_delay_us(700);
  for (int i = 0; i < 20; i++) {
    err = ads1015_read_register(adc, ADS1015_CONFIG, &reg);
    if (err != ESP_OK) return err;
    if (reg & (1 << ADS1015_OS_BIT)) break;
    esp_rom_delay_us(100);
  }

  uint16_t raw;
  err = ads1015_read_register(adc, ADS1015_CONVERSION, &raw);
  if (err != ESP_OK) return err;
  *counts = ((int16_t)raw) >> 4;
  return ESP_OK;
}

static void print_adcs(void) {
  for (int i = 0; i < adc_count; i++) {
    int a = 0, b = 0;
    esp_err_t ea = adc_read(&adcs[i], ADS1015_MUX_AIN0_AIN1, &a);
    esp_err_t eb = adc_read(&adcs[i], ADS1015_MUX_AIN2_AIN3, &b);
    if (ea != ESP_OK || eb != ESP_OK) {
      ESP_LOGE(TAG, "    ADC 0x%02X read failed", adc_addr[i]);
      continue;
    }
    ESP_LOGI(TAG, "    ADC 0x%02X  A0-A1: %5d (%5d mV)   A2-A3: %5d (%5d mV)",
             adc_addr[i], a, a * 2, b, b * 2);
  }
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

static void print_ports(motorhat_handle_t* hat) {
  for (int m = MOTORHAT_MOTOR1; m < MOTORHAT_NUM_MOTORS; m++) {
    char b1[16], b2[16], b3[16];
    ESP_LOGI(TAG, "    M%d  IN1 %-8s IN2 %-8s PWM %-8s", m + 1,
             channel_state(&hat->pca9685, motor_channels[m].in1_channel, b1, sizeof(b1)),
             channel_state(&hat->pca9685, motor_channels[m].in2_channel, b2, sizeof(b2)),
             channel_state(&hat->pca9685, motor_channels[m].pwm_channel, b3, sizeof(b3)));
  }
}

static void drive(motorhat_handle_t* hat, motorhat_motor_t port, motorhat_direction_t dir, uint16_t speed) {
  ESP_ERROR_CHECK_WITHOUT_ABORT(motorhat_set_motor_direction(hat, port, dir));
  ESP_ERROR_CHECK_WITHOUT_ABORT(motorhat_set_motor_speed(hat, port, speed));
}

static void run_step(motorhat_handle_t* hat, const motor_check_t* m, motorhat_direction_t dir) {
  bool fwd = (dir == MOTORHAT_DIRECTION_FORWARD);
  ESP_LOGI(TAG, "%s -> HAT M%d (D50 %d / %d)  %s: expect M%d IN1 %s, IN2 %s, PWM %d; other ports low",
           m->name, m->port + 1, m->d50_a, m->d50_b, fwd ? "FORWARD" : "BACKWARD",
           m->port + 1, fwd ? "HIGH" : "low", fwd ? "low" : "HIGH", CHECK_SPEED);
  drive(hat, m->port, dir, CHECK_SPEED);
  vTaskDelay(pdMS_TO_TICKS(200));
  print_ports(hat);
  print_adcs();
  vTaskDelay(pdMS_TO_TICKS(1000));
  drive(hat, m->port, MOTORHAT_DIRECTION_RELEASE, 0);
  vTaskDelay(pdMS_TO_TICKS(500));
}

static void motor_test(motorhat_handle_t* hat) {
  motorhat_emergency_stop(hat);
  ESP_LOGI(TAG, "All ports released (expect every channel low):");
  print_ports(hat);
  ESP_LOGI(TAG, "ADCs at rest:");
  print_adcs();

  for (int i = 0; i < NUM_MOTORS; i++) {
    run_step(hat, &motors[i], MOTORHAT_DIRECTION_FORWARD);
    run_step(hat, &motors[i], MOTORHAT_DIRECTION_BACKWARD);
  }
  motorhat_emergency_stop(hat);
  ESP_LOGI(TAG, "Motor port test done, all ports released");
}

static void encoder_monitor(void) {
  uint64_t mask = 0;
  for (int i = 0; i < NUM_ENC_PINS; i++) mask |= 1ULL << enc_pins[i].gpio;
  gpio_config_t io = {
      .pin_bit_mask = mask,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,  // 34-39 have none anyway; read what is actually driven
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  ESP_ERROR_CHECK(gpio_config(&io));

  int last[NUM_ENC_PINS];
  ESP_LOGI(TAG, "Encoder pin levels now:");
  for (int i = 0; i < NUM_ENC_PINS; i++) {
    last[i] = gpio_get_level(enc_pins[i].gpio);
    ESP_LOGI(TAG, "  %s  D50 %2d -> GPIO%-2d = %d", enc_pins[i].name, enc_pins[i].d50, enc_pins[i].gpio, last[i]);
  }
  ESP_LOGI(TAG, "Touch each encoder GPIO terminal to GND, then 3V3 (never 5 V); only that pin should log");

  while (1) {
    for (int i = 0; i < NUM_ENC_PINS; i++) {
      int now = gpio_get_level(enc_pins[i].gpio);
      if (now != last[i]) {
        ESP_LOGI(TAG, "  %s  D50 %2d -> GPIO%-2d  %d -> %d", enc_pins[i].name, enc_pins[i].d50, enc_pins[i].gpio, last[i], now);
        last[i] = now;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

void wiring_check(void) {
  ESP_LOGW(TAG, "WIRING CHECK BUILD - normal firmware is not running");
  ESP_ERROR_CHECK(signal_bus_init());  // motorhat speed/direction read its fault bits

  i2c_bus_t bus;
  i2c_bus_config_t bus_cfg = {
      .port = I2C_NUM_0,
      .sda_io_num = CONFIG_DRIVER_MOTORHAT_SDA_PIN,
      .scl_io_num = CONFIG_DRIVER_MOTORHAT_SCL_PIN,
  };
  ESP_ERROR_CHECK(i2c_bus_init(&bus, &bus_cfg));
  i2c_scan(bus.handle);

  // Only the PCA9685 is set up, so no soft limit / homing / encoder callbacks run.
  motorhat_handle_t hat = {0};
  pca9685_config_t pca_cfg = {
      .i2c_addr = CONFIG_DRIVER_MOTORHAT_ADDRESS,
      .i2c_speed_hz = 400000,
      .pwm_freq_hz = DEFAULT_FREQUENCY_HZ,
      .bus_handle = bus.handle,
  };
  esp_err_t err = pca9685_init(&hat.pca9685, &pca_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Motor HAT init failed (%s), skipping motor port test", esp_err_to_name(err));
  } else {
    motor_test(&hat);
  }

  encoder_monitor();
}

// ---------------------------------------------------------------------------
// Robot-connected check

#define ROBOT_SPEED (PCA9685_PWM_MAX / 4)  // 25%, inside the TB6612 stall rule from the Kconfig help
#define ROBOT_PULSE_MS 200                 // short enough that a wrong mapping can't reach a hard stop
#define ROBOT_SETTLE_MS 300                // let the axis coast to a stop before reading encoders
#define ROBOT_MIN_COUNTS 2                 // an encoder must move at least this much to count as moving
#define CONSOLE_UART UART_NUM_0

static encoder_handle_t robot_encoders[NUM_MOTORS];  // same order as motors[]

typedef struct {
  int delta[NUM_MOTORS];  // encoder change on every axis during the pulse
  int peak[MAX_ADCS][2];  // largest |counts| on each ADC pair (A0-A1, A2-A3)
  bool overcurrent;
  bool adc_error;
} pulse_result_t;

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

static void pulse(motorhat_handle_t* hat, const motor_check_t* m, motorhat_direction_t dir, pulse_result_t* r) {
  int before[NUM_MOTORS] = {0};
  memset(r, 0, sizeof(*r));
  for (int i = 0; i < NUM_MOTORS; i++) encoder_get_raw_count(&robot_encoders[i], &before[i]);

  drive(hat, m->port, dir, ROBOT_SPEED);
  TickType_t start = xTaskGetTickCount();
  while (!r->overcurrent && !r->adc_error && (xTaskGetTickCount() - start) < pdMS_TO_TICKS(ROBOT_PULSE_MS)) {
    for (int a = 0; a < adc_count && !r->adc_error; a++) {
      for (int p = 0; p < 2; p++) {
        int v;
        // No current reading means no protection: stop instead of carrying on blind
        if (adc_read(&adcs[a], p == 0 ? ADS1015_MUX_AIN0_AIN1 : ADS1015_MUX_AIN2_AIN3, &v) != ESP_OK) {
          r->adc_error = true;
          break;
        }
        if (abs(v) > r->peak[a][p]) r->peak[a][p] = abs(v);
        // Same thresholds the normal firmware uses for overcurrent
        if (v >= CONFIG_ADS1015_HIGH_THRESH || v <= CONFIG_ADS1015_LOW_THRESH) r->overcurrent = true;
      }
    }
  }
  drive(hat, m->port, MOTORHAT_DIRECTION_RELEASE, 0);
  if (r->overcurrent || r->adc_error) return;

  vTaskDelay(pdMS_TO_TICKS(ROBOT_SETTLE_MS));
  for (int i = 0; i < NUM_MOTORS; i++) {
    int after = before[i];
    encoder_get_raw_count(&robot_encoders[i], &after);
    r->delta[i] = after - before[i];
  }
}

// Pulses one axis, prints what happened, halts on anything unsafe. Returns its own encoder change.
static int run_pulse(motorhat_handle_t* hat, int idx, motorhat_direction_t dir) {
  const motor_check_t* m = &motors[idx];
  const char* dir_name = (dir == MOTORHAT_DIRECTION_FORWARD) ? "FORWARD" : "BACKWARD";
  pulse_result_t r;
  pulse(hat, m, dir, &r);

  char enc[96];
  int len = 0;
  for (int i = 0; i < NUM_MOTORS; i++) {
    len += snprintf(enc + len, sizeof(enc) - len, "%s %+d   ", motors[i].name, r.delta[i]);
  }
  ESP_LOGI(TAG, "  %s %s: encoders  %s", m->name, dir_name, enc);
  for (int a = 0; a < adc_count; a++) {
    ESP_LOGI(TAG, "    ADC 0x%02X peak  A0-A1 %4d   A2-A3 %4d  (counts, 2 mV each, limit %d)",
             adc_addr[a], r.peak[a][0], r.peak[a][1], CONFIG_ADS1015_HIGH_THRESH);
  }

  if (r.overcurrent) halt(hat, "overcurrent during the pulse");
  if (r.adc_error) halt(hat, "ADC read failed during the pulse");

  int own = abs(r.delta[idx]);
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (i == idx || abs(r.delta[i]) < ROBOT_MIN_COUNTS) continue;
    if (abs(r.delta[i]) > own) {
      ESP_LOGE(TAG, "  HAT M%d moved the %s encoder more than the %s encoder", m->port + 1, motors[i].name, m->name);
      halt(hat, "motor or encoder mapping is wrong");
    }
    // The wrist is a differential, so the other wrist motor can get backdriven a little
    ESP_LOGW(TAG, "  %s encoder also moved (%+d). Small amounts are OK on the wrist.", motors[i].name, r.delta[i]);
  }
  if (own < ROBOT_MIN_COUNTS) {
    ESP_LOGW(TAG, "  %s encoder barely moved (%+d). Did the arm move? Yes = encoder not reaching its GPIO. "
                  "No = no motor power on M%d or %d%% speed too low for this axis.",
             m->name, r.delta[idx], m->port + 1, ROBOT_SPEED * 100 / PCA9685_PWM_MAX);
  }
  return r.delta[idx];
}

void robot_check(void) {
  ESP_LOGW(TAG, "ROBOT CHECK BUILD - keep a hand on the motor supply switch");
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

  motorhat_handle_t hat = {0};
  pca9685_config_t pca_cfg = {
      .i2c_addr = CONFIG_DRIVER_MOTORHAT_ADDRESS,
      .i2c_speed_hz = 400000,
      .pwm_freq_hz = DEFAULT_FREQUENCY_HZ,
      .bus_handle = bus.handle,
  };
  esp_err_t err = pca9685_init(&hat.pca9685, &pca_cfg);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Motor HAT init failed (%s). Cut motor power: the HAT may still hold its last outputs.",
             esp_err_to_name(err));
    while (1) vTaskDelay(portMAX_DELAY);
  }
  // The PCA9685 keeps its outputs through an ESP reset, so release everything first
  motorhat_emergency_stop(&hat);

  if (adc_count != 2) halt(&hat, "need both ADS1015 for overcurrent protection");

  for (int i = 0; i < NUM_MOTORS; i++) {
    encoder_config_t cfg = {
        .P0_pin = enc_pins[2 * i].gpio,
        .P1_pin = enc_pins[2 * i + 1].gpio,
        .glitch_filter_ns = CONFIG_ENCODER_GLITCH_FILTER,
        .resolution = CONFIG_ENCODER_0_RESOLUTION,
    };
    ESP_ERROR_CHECK(encoder_init(&robot_encoders[i], &cfg));
    ESP_ERROR_CHECK(encoder_start(&robot_encoders[i]));
  }

  ESP_LOGI(TAG, "ADCs at rest (should be near 0):");
  print_adcs();

  int fwd[NUM_MOTORS] = {0}, bwd[NUM_MOTORS] = {0};
  bool tested[NUM_MOTORS] = {false};

  for (int i = 0; i < NUM_MOTORS; i++) {
    const motor_check_t* m = &motors[i];
    ESP_LOGI(TAG, "---- %s on HAT M%d, encoder GPIO%d/%d ----", m->name, m->port + 1,
             enc_pins[2 * i].gpio, enc_pins[2 * i + 1].gpio);
    ESP_LOGI(TAG, "Watch the arm. Enter = %d ms FORWARD pulse, s = skip axis, q = quit", ROBOT_PULSE_MS);
    char k = wait_key();
    if (k == 'q') halt(&hat, "quit");
    if (k == 's') continue;
    fwd[i] = run_pulse(&hat, i, MOTORHAT_DIRECTION_FORWARD);

    ESP_LOGI(TAG, "Enter = %d ms BACKWARD pulse (moves it back), q = quit", ROBOT_PULSE_MS);
    k = wait_key();
    if (k == 'q') halt(&hat, "quit");
    bwd[i] = run_pulse(&hat, i, MOTORHAT_DIRECTION_BACKWARD);
    tested[i] = true;
  }

  motorhat_emergency_stop(&hat);
  ESP_LOGI(TAG, "==== Summary ====");
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (!tested[i]) {
      ESP_LOGI(TAG, "  %s M%d: skipped", motors[i].name, motors[i].port + 1);
      continue;
    }
    const char* verdict = "CHECK: encoder did not follow the motor both ways";
    if (fwd[i] >= ROBOT_MIN_COUNTS && bwd[i] <= -ROBOT_MIN_COUNTS) verdict = "OK, FORWARD increases count";
    if (fwd[i] <= -ROBOT_MIN_COUNTS && bwd[i] >= ROBOT_MIN_COUNTS) verdict = "OK, FORWARD decreases count";
    ESP_LOGI(TAG, "  %s M%d: forward %+d, backward %+d -> %s", motors[i].name, motors[i].port + 1, fwd[i], bwd[i], verdict);
  }
  ESP_LOGI(TAG, "Done, all motors released. Reset to run again.");
  while (1) vTaskDelay(portMAX_DELAY);
}

// ---------------------------------------------------------------------------
// Encoder watch: robot connected, motors never driven. Move axes by hand and
// watch the counts. Prints a line whenever any count changes.

#define WATCH_POLL_MS 50

typedef struct {
  const char* name;
  gpio_num_t p0_gpio;
  int p0_d50;
  gpio_num_t p1_gpio;
  int p1_d50;
} watch_encoder_t;

// All encoders wired to the ESP32. D50 pins from the ER-4u manual ch. 8.
// Each one uses a PCNT unit; the classic ESP32 has 8.
static const watch_encoder_t watch_encoders[] = {
    {"A1", GPIO_NUM_36, 2, GPIO_NUM_39, 5},
    {"A2", GPIO_NUM_34, 1, GPIO_NUM_35, 21},
    {"A3", GPIO_NUM_32, 36, GPIO_NUM_33, 4},
    {"A4", GPIO_NUM_25, 35, GPIO_NUM_26, 20},
    {"A5", GPIO_NUM_27, 18, GPIO_NUM_14, 3},
    {"Grip", GPIO_NUM_13, 34, GPIO_NUM_4, 19},
};
#define NUM_WATCH (sizeof(watch_encoders) / sizeof(watch_encoders[0]))

void encoder_watch(void) {
  ESP_LOGW(TAG, "ENCODER WATCH BUILD - motors are never driven");

  // Release all HAT ports in case the PCA9685 kept outputs from an earlier run.
  // Needs I2C; if the HAT is not there the watch still runs.
  i2c_bus_t bus;
  i2c_bus_config_t bus_cfg = {
      .port = I2C_NUM_0,
      .sda_io_num = CONFIG_DRIVER_MOTORHAT_SDA_PIN,
      .scl_io_num = CONFIG_DRIVER_MOTORHAT_SCL_PIN,
  };
  if (i2c_bus_init(&bus, &bus_cfg) == ESP_OK) {
    motorhat_handle_t hat = {0};
    pca9685_config_t pca_cfg = {
        .i2c_addr = CONFIG_DRIVER_MOTORHAT_ADDRESS,
        .i2c_speed_hz = 400000,
        .pwm_freq_hz = DEFAULT_FREQUENCY_HZ,
        .bus_handle = bus.handle,
    };
    if (pca9685_init(&hat.pca9685, &pca_cfg) == ESP_OK && motorhat_emergency_stop(&hat) == ESP_OK) {
      ESP_LOGI(TAG, "All HAT ports released");
    } else {
      ESP_LOGW(TAG, "Could not reach the motor HAT to release its ports; keep the motor supply off");
    }
  }

  encoder_handle_t enc[NUM_WATCH];
  for (int i = 0; i < NUM_WATCH; i++) {
    const watch_encoder_t* w = &watch_encoders[i];
    encoder_config_t cfg = {
        .P0_pin = w->p0_gpio,
        .P1_pin = w->p1_gpio,
        .glitch_filter_ns = CONFIG_ENCODER_GLITCH_FILTER,
        .resolution = CONFIG_ENCODER_0_RESOLUTION,
    };
    ESP_ERROR_CHECK(encoder_init(&enc[i], &cfg));
    ESP_ERROR_CHECK(encoder_start(&enc[i]));
    ESP_LOGI(TAG, "  %-4s  P0 D50 %2d -> GPIO%-2d   P1 D50 %2d -> GPIO%-2d", w->name,
             w->p0_d50, w->p0_gpio, w->p1_d50, w->p1_gpio);
  }
  ESP_LOGI(TAG, "Move one axis at a time by hand. Only its count should change; moving back reverses the sign.");

  int last[NUM_WATCH] = {0};
  while (1) {
    int now[NUM_WATCH];
    bool changed = false;
    for (int i = 0; i < NUM_WATCH; i++) {
      now[i] = last[i];
      encoder_get_raw_count(&enc[i], &now[i]);
      if (now[i] != last[i]) changed = true;
    }
    if (changed) {
      char line[192];
      int len = 0;
      for (int i = 0; i < NUM_WATCH; i++) {
        len += snprintf(line + len, sizeof(line) - len, "%s %5d%s ", watch_encoders[i].name, now[i],
                        now[i] != last[i] ? "*" : " ");
        last[i] = now[i];
      }
      ESP_LOGI(TAG, "%s", line);
    }
    vTaskDelay(pdMS_TO_TICKS(WATCH_POLL_MS));
  }
}

// ---------------------------------------------------------------------------
// Shunt check: finds the ADS1015s, prints their resting readings every second,
// and on a keypress pulses one HAT port to show which ADC pair follows which motor.

#define SHUNT_PRINT_MS 1000
#define SHUNT_PULSE_MS 300
#define SHUNT_MIN_RESPONSE 5     // counts (10 mV); smaller than this is treated as noise
#define ADS1015_FULL_SCALE 2047  // largest positive code

static const ads1015_mux_t pair_mux[2] = {ADS1015_MUX_AIN0_AIN1, ADS1015_MUX_AIN2_AIN3};
static const char* pair_name[2] = {"A0-A1", "A2-A3"};
static const ads1015_mux_t single_mux[4] = {ADS1015_MUX_AIN0_GND, ADS1015_MUX_AIN1_GND,
                                            ADS1015_MUX_AIN2_GND, ADS1015_MUX_AIN3_GND};

static void shunt_print_rest(void) {
  for (int a = 0; a < adc_count; a++) {
    int pair[2] = {0}, single[4] = {0};
    bool ok = true;
    for (int p = 0; p < 2 && ok; p++) ok = adc_read(&adcs[a], pair_mux[p], &pair[p]) == ESP_OK;
    for (int s = 0; s < 4 && ok; s++) ok = adc_read(&adcs[a], single_mux[s], &single[s]) == ESP_OK;
    if (!ok) {
      ESP_LOGE(TAG, "ADC 0x%02X read failed", adc_addr[a]);
      continue;
    }
    ESP_LOGI(TAG, "ADC 0x%02X  A0-A1 %5d  A2-A3 %5d  |  A0 %4d mV  A1 %4d mV  A2 %4d mV  A3 %4d mV",
             adc_addr[a], pair[0], pair[1], single[0] * 2, single[1] * 2, single[2] * 2, single[3] * 2);
    for (int s = 0; s < 4; s++) {
      if (single[s] >= ADS1015_FULL_SCALE - 5) {
        ESP_LOGW(TAG, "  ADC 0x%02X A%d is at full scale (about 4 V or more). The ADS1015 inputs must stay "
                      "below its supply voltage; if this pin sits on the motor side of a shunt it can be damaged.",
                 adc_addr[a], s);
      }
    }
  }
}

// One pulse on one port. Tracks the signed min and max of every pair.
static void shunt_pulse(motorhat_handle_t* hat, motorhat_motor_t port, motorhat_direction_t dir) {
  int lo[MAX_ADCS][2], hi[MAX_ADCS][2];
  for (int a = 0; a < MAX_ADCS; a++) {
    for (int p = 0; p < 2; p++) {
      lo[a][p] = 100000;
      hi[a][p] = -100000;
    }
  }
  bool hit_limit = false, adc_error = false;

  drive(hat, port, dir, ROBOT_SPEED);
  TickType_t start = xTaskGetTickCount();
  while (!hit_limit && !adc_error && (xTaskGetTickCount() - start) < pdMS_TO_TICKS(SHUNT_PULSE_MS)) {
    for (int a = 0; a < adc_count && !adc_error; a++) {
      for (int p = 0; p < 2; p++) {
        int v;
        if (adc_read(&adcs[a], pair_mux[p], &v) != ESP_OK) {
          adc_error = true;
          break;
        }
        if (v < lo[a][p]) lo[a][p] = v;
        if (v > hi[a][p]) hi[a][p] = v;
        if (v >= CONFIG_ADS1015_HIGH_THRESH || v <= CONFIG_ADS1015_LOW_THRESH) hit_limit = true;
      }
    }
  }
  drive(hat, port, MOTORHAT_DIRECTION_RELEASE, 0);

  const char* dir_name = (dir == MOTORHAT_DIRECTION_FORWARD) ? "FORWARD" : "BACKWARD";
  if (adc_error) {
    ESP_LOGE(TAG, "M%d %s: ADC read failed, motor released", port + 1, dir_name);
    return;
  }

  int best_a = -1, best_p = 0, best_v = 0;
  for (int a = 0; a < adc_count; a++) {
    for (int p = 0; p < 2; p++) {
      int peak = (abs(hi[a][p]) > abs(lo[a][p])) ? hi[a][p] : lo[a][p];
      ESP_LOGI(TAG, "  M%d %-8s ADC 0x%02X %s  min %5d  max %5d", port + 1, dir_name, adc_addr[a],
               pair_name[p], lo[a][p], hi[a][p]);
      if (abs(peak) > abs(best_v)) {
        best_a = a;
        best_p = p;
        best_v = peak;
      }
    }
  }
  if (hit_limit) {
    ESP_LOGW(TAG, "  Reading reached the overcurrent limit (+/-%d counts), pulse cut short", CONFIG_ADS1015_HIGH_THRESH);
  }
  if (best_a < 0 || abs(best_v) < SHUNT_MIN_RESPONSE) {
    ESP_LOGW(TAG, "  M%d %s: no pair responded. Motor not connected or powered, or its shunt is not wired to an ADC.",
             port + 1, dir_name);
  } else {
    ESP_LOGI(TAG, "  M%d (%s) %s -> ADC 0x%02X %s (peak %+d counts, %+d mV)", port + 1, motors[port].name, dir_name,
             adc_addr[best_a], pair_name[best_p], best_v, best_v * 2);
  }
}

void shunt_check(void) {
  ESP_LOGW(TAG, "SHUNT CHECK BUILD - motors only move when you press 1-4");
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

  // Release the HAT first: the PCA9685 keeps its outputs through an ESP reset
  motorhat_handle_t hat = {0};
  pca9685_config_t pca_cfg = {
      .i2c_addr = CONFIG_DRIVER_MOTORHAT_ADDRESS,
      .i2c_speed_hz = 400000,
      .pwm_freq_hz = DEFAULT_FREQUENCY_HZ,
      .bus_handle = bus.handle,
  };
  bool hat_ok = pca9685_init(&hat.pca9685, &pca_cfg) == ESP_OK && motorhat_emergency_stop(&hat) == ESP_OK;
  if (!hat_ok) ESP_LOGW(TAG, "Motor HAT not reachable: resting readings only, keys 1-4 disabled");

  if (adc_count == 0) {
    ESP_LOGE(TAG, "No ADS1015 answered. Check each one's VDD, GND, SDA (GPIO%d) and SCL (GPIO%d).",
             CONFIG_DRIVER_MOTORHAT_SDA_PIN, CONFIG_DRIVER_MOTORHAT_SCL_PIN);
    ESP_LOGE(TAG, "ADDR pin sets the address: GND=0x48, VDD=0x49, SDA=0x4A, SCL=0x4B. Firmware expects 0x%02X.",
             CONFIG_DRIVER_ADS1015_ADDRESS);
    while (1) vTaskDelay(portMAX_DELAY);
  }

  // Before anything is written, the config register should read the power-on default 0x8583
  for (int a = 0; a < adc_count; a++) {
    uint16_t reg = 0;
    if (ads1015_read_register(&adcs[a], ADS1015_CONFIG, &reg) == ESP_OK) {
      ESP_LOGI(TAG, "ADC 0x%02X config 0x%04X%s%s", adc_addr[a], reg, reg == 0x8583 ? " (power-on default)" : "",
               adc_addr[a] == CONFIG_DRIVER_ADS1015_ADDRESS ? "  <- address the firmware uses" : "");
    } else {
      ESP_LOGE(TAG, "ADC 0x%02X answered the scan but its config register could not be read", adc_addr[a]);
    }
  }
  bool fw_addr_found = false;
  for (int a = 0; a < adc_count; a++) fw_addr_found |= adc_addr[a] == CONFIG_DRIVER_ADS1015_ADDRESS;
  if (!fw_addr_found) {
    ESP_LOGW(TAG, "Nothing at 0x%02X (DRIVER_ADS1015_ADDRESS): the normal firmware will crash in ads1015_init",
             CONFIG_DRIVER_ADS1015_ADDRESS);
  }

  ESP_LOGI(TAG, "Readings at rest print every second; pairs should sit near 0 with motors off.");
  ESP_LOGI(TAG, "Keys 1-4 = %d ms pulse on HAT M1-M4, forward then backward (motor must be connected and powered)",
           SHUNT_PULSE_MS);
  for (int i = 0; i < NUM_MOTORS; i++) ESP_LOGI(TAG, "  %d = M%d %s", motors[i].port + 1, motors[i].port + 1, motors[i].name);

  while (1) {
    shunt_print_rest();
    uint8_t c = 0;
    if (uart_read_bytes(CONSOLE_UART, &c, 1, pdMS_TO_TICKS(SHUNT_PRINT_MS)) != 1) continue;
    if (c < '1' || c > '4') continue;
    if (!hat_ok) {
      ESP_LOGW(TAG, "Motor HAT not reachable, ignoring key");
      continue;
    }
    motorhat_motor_t port = (motorhat_motor_t)(c - '1');
    ESP_LOGI(TAG, "---- Pulsing HAT M%d (%s) ----", port + 1, motors[port].name);
    shunt_pulse(&hat, port, MOTORHAT_DIRECTION_FORWARD);
    vTaskDelay(pdMS_TO_TICKS(500));
    shunt_pulse(&hat, port, MOTORHAT_DIRECTION_BACKWARD);
    uart_flush_input(CONSOLE_UART);
  }
}
