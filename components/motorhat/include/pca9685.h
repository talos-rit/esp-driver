#ifndef _PCA9685_H_
#define _PCA9685_H_

#include "driver/i2c_types.h"
#include "esp_err.h"
#include <stdint.h>

#define PCA9685_PWM_MAX 4096
#define PCA9685_I2C_MASTER_TIMEOUT_MS 1000

typedef enum {
  PCA9685_MODE1 = 0x00,
  PCA9685_MODE2 = 0x01,

  PCA9685_SUBADR1 = 0x02,
  PCA9685_SUBADR2 = 0x03,
  PCA9685_SUBADR3 = 0x04,
  PCA9685_ALLCALLADR = 0x05,

  PCA9685_LED0_ON_L = 0x06,
  PCA9685_LED0_ON_H = 0x07,
  PCA9685_LED0_OFF_L = 0x08,
  PCA9685_LED0_OFF_H = 0x09,

  PCA9685_LED1_ON_L = 0x0A,
  PCA9685_LED1_ON_H = 0x0B,
  PCA9685_LED1_OFF_L = 0x0C,
  PCA9685_LED1_OFF_H = 0x0D,

  PCA9685_LED2_ON_L = 0x0E,
  PCA9685_LED2_ON_H = 0x0F,
  PCA9685_LED2_OFF_L = 0x10,
  PCA9685_LED2_OFF_H = 0x11,

  PCA9685_LED3_ON_L = 0x12,
  PCA9685_LED3_ON_H = 0x13,
  PCA9685_LED3_OFF_L = 0x14,
  PCA9685_LED3_OFF_H = 0x15,

  PCA9685_LED4_ON_L = 0x16,
  PCA9685_LED4_ON_H = 0x17,
  PCA9685_LED4_OFF_L = 0x18,
  PCA9685_LED4_OFF_H = 0x19,

  PCA9685_LED5_ON_L = 0x1A,
  PCA9685_LED5_ON_H = 0x1B,
  PCA9685_LED5_OFF_L = 0x1C,
  PCA9685_LED5_OFF_H = 0x1D,

  PCA9685_LED6_ON_L = 0x1E,
  PCA9685_LED6_ON_H = 0x1F,
  PCA9685_LED6_OFF_L = 0x20,
  PCA9685_LED6_OFF_H = 0x21,

  PCA9685_LED7_ON_L = 0x22,
  PCA9685_LED7_ON_H = 0x23,
  PCA9685_LED7_OFF_L = 0x24,
  PCA9685_LED7_OFF_H = 0x25,

  PCA9685_LED8_ON_L = 0x26,
  PCA9685_LED8_ON_H = 0x27,
  PCA9685_LED8_OFF_L = 0x28,
  PCA9685_LED8_OFF_H = 0x29,

  PCA9685_LED9_ON_L = 0x2A,
  PCA9685_LED9_ON_H = 0x2B,
  PCA9685_LED9_OFF_L = 0x2C,
  PCA9685_LED9_OFF_H = 0x2D,

  PCA9685_LED10_ON_L = 0x2E,
  PCA9685_LED10_ON_H = 0x2F,
  PCA9685_LED10_OFF_L = 0x30,
  PCA9685_LED10_OFF_H = 0x31,

  PCA9685_LED11_ON_L = 0x32,
  PCA9685_LED11_ON_H = 0x33,
  PCA9685_LED11_OFF_L = 0x34,
  PCA9685_LED11_OFF_H = 0x35,

  PCA9685_LED12_ON_L = 0x36,
  PCA9685_LED12_ON_H = 0x37,
  PCA9685_LED12_OFF_L = 0x38,
  PCA9685_LED12_OFF_H = 0x39,

  PCA9685_LED13_ON_L = 0x3A,
  PCA9685_LED13_ON_H = 0x3B,
  PCA9685_LED13_OFF_L = 0x3C,
  PCA9685_LED13_OFF_H = 0x3D,

  PCA9685_LED14_ON_L = 0x3E,
  PCA9685_LED14_ON_H = 0x3F,
  PCA9685_LED14_OFF_L = 0x40,
  PCA9685_LED14_OFF_H = 0x41,

  PCA9685_LED15_ON_L = 0x42,
  PCA9685_LED15_ON_H = 0x43,
  PCA9685_LED15_OFF_L = 0x44,
  PCA9685_LED15_OFF_H = 0x45,

  PCA9685_ALL_LED_ON_L = 0xFA,
  PCA9685_ALL_LED_ON_H = 0xFB,
  PCA9685_ALL_LED_OFF_L = 0xFC,
  PCA9685_ALL_LED_OFF_H = 0xFD,
  PCA9685_PRE_SCALE = 0xFE,
  PCA9685_TESTMODE = 0xFF
} pca9685_register_t;

typedef enum {
  PCA9685_CHANNEL0 = 0,
  PCA9685_CHANNEL1 = 1,
  PCA9685_CHANNEL2 = 2,
  PCA9685_CHANNEL3 = 3,
  PCA9685_CHANNEL4 = 4,
  PCA9685_CHANNEL5 = 5,
  PCA9685_CHANNEL6 = 6,
  PCA9685_CHANNEL7 = 7,
  PCA9685_CHANNEL8 = 8,
  PCA9685_CHANNEL9 = 9,
  PCA9685_CHANNEL10 = 10,
  PCA9685_CHANNEL11 = 11,
  PCA9685_CHANNEL12 = 12,
  PCA9685_CHANNEL13 = 13,
  PCA9685_CHANNEL14 = 14,
  PCA9685_CHANNEL15 = 15
} pca9685_channel_t;

typedef struct {
  pca9685_register_t on_low;
  pca9685_register_t on_high;
  pca9685_register_t off_low;
  pca9685_register_t off_high;
} pca9685_channel_registers_t;

typedef struct {
  uint8_t i2c_addr;
  uint32_t i2c_speed_hz;
  float pwm_freq_hz;
  i2c_master_bus_handle_t bus_handle;
} pca9685_config_t;

typedef struct {
  i2c_master_dev_handle_t dev_handle;
} pca9685_handle_t;



esp_err_t pca9685_init(pca9685_handle_t *handle,
                       const pca9685_config_t *config);

esp_err_t pca9685_set_duty_cycle(pca9685_handle_t *handle,
                                 pca9685_channel_t channel, float duty_cycle);

esp_err_t pca9685_digital_write(pca9685_handle_t *handle,
                                pca9685_channel_t channel, bool level);

esp_err_t pca9685_write_channel_registers(pca9685_handle_t *handle,
                                          pca9685_channel_t channel,
                                          uint16_t on, uint16_t off);

esp_err_t pca9685_read_register(pca9685_handle_t *handle,
                                pca9685_register_t reg, uint8_t *data,
                                size_t len);

esp_err_t pca9685_write_register(pca9685_handle_t *handle,
                                 pca9685_register_t reg, uint8_t data);

#endif // _PCA9685_H_
