#ifndef _ADS1015_H_
#define _ADS1015_H_

#include "driver/i2c_types.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define ADS1015_I2C_MASTER_TIMEOUT_MS 1000
#define ADS1015_MUX_MASK          (0x7 << 12)
// Config Register Bit Positions
#define ADS1015_OS_BIT            15
#define ADS1015_MUX_SHIFT         12
#define ADS1015_PGA_SHIFT         9
#define ADS1015_MODE_BIT          8
#define ADS1015_DR_SHIFT          5
#define ADS1015_COMP_MODE_BIT     4
#define ADS1015_COMP_POL_BIT      3
#define ADS1015_COMP_LAT_BIT      2
#define ADS1015_COMP_QUE_SHIFT    0

/**
 * @brief ADS1015 register addresses
 */
typedef enum {
    ADS1015_CONVERSION = 0x00,
    ADS1015_CONFIG = 0x01,
    ADS1015_LOW_THRESH_REG = 0x02,
    ADS1015_HIGH_THRESH_REG = 0x03,
} ads1015_register_t;

/**
 * @brief ADS1015 MUX register bit definitions
 */
typedef enum {
    ADS1015_MUX_AIN0_AIN1 = 0b000,
    ADS1015_MUX_AIN0_AIN3 = 0b001,
    ADS1015_MUX_AIN1_AIN3 = 0b010,
    ADS1015_MUX_AIN2_AIN3 = 0b011,
    ADS1015_MUX_AIN0_GND  = 0b100,
    ADS1015_MUX_AIN1_GND  = 0b101,
    ADS1015_MUX_AIN2_GND  = 0b110,
    ADS1015_MUX_AIN3_GND  = 0b111
} ads1015_mux_t;

/**
 * @brief ADS1015 PGA register bit definitions
 */
typedef enum {
    ADS1015_PGA_6_144V = 0b000,
    ADS1015_PGA_4_096V = 0b001,
    ADS1015_PGA_2_048V = 0b010,
    ADS1015_PGA_1_024V = 0b011,
    ADS1015_PGA_0_512V = 0b100,
    ADS1015_PGA_0_256V = 0b101
} ads1015_pga_t;

/**
 * @brief ADS1015 mode register bit definitions
 */
typedef enum {
    ADS1015_MODE_CONTINUOUS = 0,
    ADS1015_MODE_SINGLESHOT = 1
} ads1015_mode_t;

/**
 * @brief ADS1015 data rate register bit definitions
 */
typedef enum {
    ADS1015_DR_128SPS  = 0b000,
    ADS1015_DR_250SPS  = 0b001,
    ADS1015_DR_490SPS  = 0b010,
    ADS1015_DR_920SPS  = 0b011,
    ADS1015_DR_1600SPS = 0b100,
    ADS1015_DR_2400SPS = 0b101,
    ADS1015_DR_3300SPS = 0b110
} ads1015_dr_t;

/**
 * @brief ADS1015 comparator settings register bit definitions
 */
typedef enum {
    ADS1015_COMP_TRADITIONAL = 0,
    ADS1015_COMP_WINDOW      = 1
} ads1015_comp_mode_t;

typedef enum {
    ADS1015_COMP_ACTIVE_LOW  = 0,
    ADS1015_COMP_ACTIVE_HIGH = 1
} ads1015_comp_pol_t;

typedef enum {
    ADS1015_COMP_NON_LATCHING = 0,
    ADS1015_COMP_LATCHING     = 1
} ads1015_comp_lat_t;

typedef enum {
    ADS1015_COMP_ASSERT_1 = 0b00,
    ADS1015_COMP_ASSERT_2 = 0b01,
    ADS1015_COMP_ASSERT_4 = 0b10,
    ADS1015_COMP_DISABLE  = 0b11
} ads1015_comp_que_t;

static inline uint16_t ads1015_build_config(
    ads1015_mux_t mux,
    ads1015_pga_t pga,
    ads1015_mode_t mode,
    ads1015_dr_t dr,
    ads1015_comp_mode_t comp_mode,
    ads1015_comp_pol_t comp_pol,
    ads1015_comp_lat_t comp_lat,
    ads1015_comp_que_t comp_que,
    bool start_conversion
) {
    return
        ((start_conversion ? 1 : 0) << ADS1015_OS_BIT) |
        (mux        << ADS1015_MUX_SHIFT) |
        (pga        << ADS1015_PGA_SHIFT) |
        (mode       << ADS1015_MODE_BIT) |
        (dr         << ADS1015_DR_SHIFT) |
        (comp_mode  << ADS1015_COMP_MODE_BIT) |
        (comp_pol   << ADS1015_COMP_POL_BIT) |
        (comp_lat   << ADS1015_COMP_LAT_BIT) |
        (comp_que   << ADS1015_COMP_QUE_SHIFT);
}

/**
 * @brief ADS1015 configuration structure
 */
typedef struct {
    uint8_t i2c_addr;                   /**< I2C device address */
    uint32_t i2c_speed_hz;              /**< I2C bus speed in Hz */
    i2c_master_bus_handle_t bus_handle; /**< I2C master bus handle */
    gpio_num_t rdy_gpio;                /**< GPIO number for ALERT pin */
} ads1015_config_t;

/**
 * @brief ADS1015 device handle
 */
typedef struct {
    i2c_master_dev_handle_t dev_handle; /**< I2C device handle */
    uint16_t config_reg;                /**< ADS1015 config register values */
} ads1015_handle_t;

/**
 * @brief Initialize the ADS1015 device
 *
 * This function initializes the ADS1015 device, configures the I2C
 * communication, sets the device address, and sets up the alert GPIO.
 *
 * @param[out] handle Pointer to ADS1015 handle structure
 * @param[in] config Pointer to configuration structure
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer or invalid
 * frequency)
 *    - ESP_ERR_*: Other ESP-IDF error codes from I2C operations
 */
esp_err_t ads_init(ads1015_handle_t *handle, const ads1015_config_t *config);

/**
 * @brief Check the ADC value against the configured threshholds and trigger E-stop if it exceeds them
 *
 * @param[in] value ADC value to check
 *
 * @return
 *    - ESP_OK: Success
 */
esp_err_t ads1015_check_current(int16_t value, bool mux_state);

/**
 * @brief Read one or more registers from the ADS1015
 *
 * @param[in] handle Pointer to ADS1015 handle
 * @param[in] reg Starting register address to read from
 * @param[out] data Pointer to buffer to store read data
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer)
 *    - ESP_ERR_*: Other ESP-IDF error codes from I2C operations
 */
esp_err_t ads1015_read_register(ads1015_handle_t *handle, ads1015_register_t reg, uint16_t *data);

/**
 * @brief Write a single byte to a ADS1015 register
 *
 * @param[in] handle Pointer to ADS1015 handle
 * @param[in] reg Register address to write to
 * @param[in] data Byte value to write
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL handle)
 *    - ESP_ERR_*: Other ESP-IDF error codes from I2C operations
 */
esp_err_t ads1015_write_register(ads1015_handle_t *handle, ads1015_register_t reg, uint16_t data);

#endif // _ADS1015_H_
