#ifndef _PCA9685_H_
#define _PCA9685_H_

#include "driver/i2c_types.h"
#include "esp_err.h"
#include <stdint.h>

#define PCA9685_PWM_MAX 4096
#define PCA9685_I2C_MASTER_TIMEOUT_MS 1000

// 25 MHz internal oscillator frequency
#define PCA9685_OSCILLATOR_FREQ_HZ 25000000
#define PCA9685_MIN_PWM_FREQ_HZ 24.0f
#define PCA9685_MAX_PWM_FREQ_HZ 1526.0f

// Calculate pre-scale value for a desired PWM frequency
// See PCA9685 datasheet section 7.3.5
// The + 0.5f is for rounding to nearest integer without truncation
#define PCA9685_CALC_PRE_SCALE(freq_hz)                                        \
  ((uint8_t)((PCA9685_OSCILLATOR_FREQ_HZ / (4096.0f * (freq_hz))) - 1.0f +     \
             0.5f))

/**
 * @brief PCA9685 register addresses
 */
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

/**
 * @brief PCA9685 mode register bit definitions
 */
typedef enum {
  PCA9685_MODE1_RESTART = 0x80,
  PCA9685_MODE1_EXTCLK = 0x40,
  PCA9685_MODE1_AI = 0x20,
  PCA9685_MODE1_SLEEP = 0x10,
  PCA9685_MODE1_SUB1 = 0x08,
  PCA9685_MODE1_SUB2 = 0x04,
  PCA9685_MODE1_SUB3 = 0x02,
  PCA9685_MODE1_ALLCALL = 0x01,

  PCA9685_MODE2_INVRT = 0x10,
  PCA9685_MODE2_OCH = 0x08,
  PCA9685_MODE2_OUTDRV = 0x04,
  PCA9685_MODE2_OUTNE1 = 0x02,
  PCA9685_MODE2_OUTNE0 = 0x01
} pca9685_mode_bits_t;

/**
 * @brief PCA9685 channel numbers (0-15)
 */
typedef enum {
  PCA9685_CHANNEL0 = 0,
  PCA9685_CHANNEL1,
  PCA9685_CHANNEL2,
  PCA9685_CHANNEL3,
  PCA9685_CHANNEL4,
  PCA9685_CHANNEL5,
  PCA9685_CHANNEL6,
  PCA9685_CHANNEL7,
  PCA9685_CHANNEL8,
  PCA9685_CHANNEL9,
  PCA9685_CHANNEL10,
  PCA9685_CHANNEL11,
  PCA9685_CHANNEL12,
  PCA9685_CHANNEL13,
  PCA9685_CHANNEL14,
  PCA9685_CHANNEL15,
} pca9685_channel_t;

/**
 * @brief Register addresses for a single PWM channel
 */
typedef struct {
  pca9685_register_t on_low;
  pca9685_register_t on_high;
  pca9685_register_t off_low;
  pca9685_register_t off_high;
} pca9685_channel_registers_t;

/**
 * @brief PCA9685 configuration structure
 */
typedef struct {
  uint8_t i2c_addr;      /**< I2C device address (typically 0x40-0x7F) */
  uint32_t i2c_speed_hz; /**< I2C bus speed in Hz */
  float pwm_freq_hz;     /**< PWM frequency in Hz (24-1526 Hz) */
  i2c_master_bus_handle_t bus_handle; /**< I2C master bus handle */
} pca9685_config_t;

/**
 * @brief PCA9685 device handle
 */
typedef struct {
  i2c_master_dev_handle_t dev_handle; /**< I2C device handle */
} pca9685_handle_t;

/**
 * @brief Initialize the PCA9685 device
 *
 * This function initializes the PCA9685 PWM controller, configures the I2C
 * communication, sets the PWM frequency, and enables auto-increment mode.
 *
 * @param[out] handle Pointer to PCA9685 handle structure
 * @param[in] config Pointer to configuration structure
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer or invalid
 * frequency)
 *    - ESP_ERR_*: Other ESP-IDF error codes from I2C operations
 */
esp_err_t pca9685_init(pca9685_handle_t *handle,
                       const pca9685_config_t *config);

/**
 * @brief Deinitialize the PCA9685 device
 *
 * This function releases the I2C device handle and performs any necessary cleanup.
 *
 * @param[in] handle Pointer to PCA9685 handle structure
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer)
 *    - ESP_ERR_*: Other ESP-IDF error codes from I2C operations
 */
 esp_err_t pca9685_deinit(pca9685_handle_t *handle);

/**
 * @brief Set PWM duty cycle for a specific channel
 *
 * Sets the duty cycle for a channel using a 12-bit value (0-4096).
 * - 0 = fully off
 * - 4096 = fully on
 * - Values in between set proportional duty cycle
 *
 * @param[in] handle Pointer to PCA9685 handle
 * @param[in] channel Channel number (0-15)
 * @param[in] duty_cycle Duty cycle value (0-4096)
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL handle, invalid channel, or
 * duty cycle > 4096)
 *    - ESP_ERR_*: Other ESP-IDF error codes from I2C operations
 */
esp_err_t pca9685_set_duty_cycle(pca9685_handle_t *handle,
                                 pca9685_channel_t channel,
                                 uint16_t duty_cycle);

/**
 * @brief Set digital output state for a channel
 *
 * Sets a channel to fully on (high) or fully off (low), useful for
 * controlling digital outputs like LEDs or relays.
 *
 * @param[in] handle Pointer to PCA9685 handle
 * @param[in] channel Channel number (0-15)
 * @param[in] level Output level (true = HIGH/ON, false = LOW/OFF)
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL handle or invalid channel)
 *    - ESP_ERR_*: Other ESP-IDF error codes from I2C operations
 */
esp_err_t pca9685_digital_write(pca9685_handle_t *handle,
                                pca9685_channel_t channel, bool level);

/**
 * @brief Write ON and OFF register values for a channel
 *
 * Low-level function to directly set the ON and OFF counter values for a
 * channel. This allows precise control of when the output turns on and off
 * within the PWM cycle (0-4095). For most applications, use
 * pca9685_set_duty_cycle() instead.
 *
 * @param[in] handle Pointer to PCA9685 handle
 * @param[in] channel Channel number (0-15)
 * @param[in] on Counter value when output turns ON (0-4096)
 * @param[in] off Counter value when output turns OFF (0-4096)
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL handle, invalid channel, or
 * values > 4096)
 *    - ESP_ERR_*: Other ESP-IDF error codes from I2C operations
 */
esp_err_t pca9685_write_channel_registers(pca9685_handle_t *handle,
                                          pca9685_channel_t channel,
                                          uint16_t on, uint16_t off);

/**
 * @brief Read one or more registers from the PCA9685
 *
 * @param[in] handle Pointer to PCA9685 handle
 * @param[in] reg Starting register address to read from
 * @param[out] data Pointer to buffer to store read data
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer)
 *    - ESP_ERR_*: Other ESP-IDF error codes from I2C operations
 */
esp_err_t pca9685_read_register(pca9685_handle_t *handle,
                                pca9685_register_t reg, uint8_t *data);

/**
 * @brief Write a single byte to a PCA9685 register
 *
 * @param[in] handle Pointer to PCA9685 handle
 * @param[in] reg Register address to write to
 * @param[in] data Byte value to write
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL handle)
 *    - ESP_ERR_*: Other ESP-IDF error codes from I2C operations
 */
esp_err_t pca9685_write_register(pca9685_handle_t *handle,
                                 pca9685_register_t reg, uint8_t data);

#endif // _PCA9685_H_
