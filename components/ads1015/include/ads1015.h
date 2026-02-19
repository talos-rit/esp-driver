#ifndef _ADS1015_H_
#define _ADS1015_H_

#include "driver/i2c_types.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

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
    uint8_t i2c_addr;                   /**< I2C device address (typically 0x48) */
    uint32_t i2c_speed_hz;              /**< I2C bus speed in Hz */
    i2c_master_bus_handle_t bus_handle; /**< I2C master bus handle */
    gpio_num_t alert_gpio;              /**< GPIO number for RDY pin */
    SemaphoreHandle_t rdy_sem;          /**< Binary semaphore to signal when a new conversion is ready */
} ads1015_config_t;

/**
 * @brief ADS1015 device handle
 */
typedef struct {
    i2c_master_dev_handle_t dev_handle; /**< I2C device handle */
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

#endif // _ADS1015_H_
