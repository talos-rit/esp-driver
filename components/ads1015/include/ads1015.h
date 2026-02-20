#ifndef _ADS1015_H_
#define _ADS1015_H_

#include "driver/i2c_types.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define ADS1015_I2C_MASTER_TIMEOUT_MS 1000

/**
 * @brief ADS1015 register addresses
 */
typedef enum {
    ADS1015_CONVERSION = 0x00,
    ADS1015_CONFIG = 0x01,
    ADS1015_LOW_THRESH = 0x02,
    ADS1015_HIGH_THRESH = 0x03,
} ads1015_register_t;

/**
 * @brief ADS1015 configuration structure
 */
typedef struct {
    uint8_t i2c_addr;                   /**< I2C device address (should be 0x48 if ADDR is connected to GND) */
    uint32_t i2c_speed_hz;              /**< I2C bus speed in Hz */
    i2c_master_bus_handle_t bus_handle; /**< I2C master bus handle */
    gpio_num_t alert_gpio;              /**< GPIO number for RDY pin (gets passed to ads1015_handle_t) */
} ads1015_config_t;

/**
 * @brief ADS1015 device handle
 */
typedef struct {
    i2c_master_dev_handle_t dev_handle; /**< I2C device handle */
    gpio_num_t alert_gpio;              /**< GPIO number for RDY pin */
    SemaphoreHandle_t rdy_sem;          /**< Binary semaphore to signal when a new conversion is ready */
    uint16_t config_reg;                /**< Cached config register value for easy updates */
} ads1015_handle_t;

/**
 * @brief Initialize the ADS1015 device
 *
 * This function initializes the ADS1015 device, configures the I2C
 * communication, sets the device address, and initializes the alert GPIO.
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
 * @brief Start a single conversion on the ADS1015 in single shot mode
 *
 * @param[in] handle Pointer to ADS1015 handle
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer)
 *    - ESP_ERR_*: Other ESP-IDF error codes from I2C operations
 */
esp_err_t ads1015_start_conversion(ads1015_handle_t *handle);

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
