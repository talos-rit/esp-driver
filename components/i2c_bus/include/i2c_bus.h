#ifndef _I2C_BUS_H_
#define _I2C_BUS_H_

#include "driver/i2c_types.h"
#include "soc/gpio_num.h"
#include "esp_err.h"

/**
 * @brief I2C bus configuration structure
 *
 * Contains the essential parameters needed to configure an I2C master bus,
 * including the I2C port number and GPIO pin assignments.
 */
typedef struct {
  i2c_port_t port;         /**< I2C port number (I2C_NUM_0, I2C_NUM_1, etc.) */
  gpio_num_t sda_io_num;   /**< GPIO number for SDA (data line) */
  gpio_num_t scl_io_num;   /**< GPIO number for SCL (clock line) */
} i2c_bus_config_t;

/**
 * @brief I2C bus handle structure
 *
 * Encapsulates the ESP-IDF I2C master bus handle. This handle is used
 * for all subsequent operations on the I2C bus, including adding devices.
 */
typedef struct {
  i2c_master_bus_handle_t handle;  /**< ESP-IDF I2C master bus handle */
} i2c_bus_t;

/**
 * @brief Initialize an I2C master bus
 *
 * Initializes an I2C master bus with the specified configuration. The bus
 * is configured with:
 * - Default clock source (I2C_CLK_SRC_DEFAULT)
 * - Glitch filtering (7 cycles)
 * - Internal pull-up resistors enabled
 *
 * After successful initialization, the bus handle can be used to add I2C
 * devices using the ESP-IDF i2c_master_bus_add_device() function.
 *
 * @param[out] bus Pointer to I2C bus structure to initialize
 * @param[in] config Pointer to configuration structure with port and GPIO settings
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument (NULL pointer, invalid port, or invalid GPIO)
 *    - ESP_ERR_NO_MEM: Memory allocation failed
 *    - ESP_ERR_INVALID_STATE: I2C driver already installed on this port
 *    - ESP_ERR_*: Other ESP-IDF error codes from I2C driver initialization
 *
 * @note The GPIO pins specified must support I2C functionality. Check your
 *       ESP32 variant's datasheet for valid I2C GPIO pins.
 * @note Internal pull-ups are enabled by default. If using external pull-up
 *       resistors, you may want to modify this behavior.
 *
 * Example usage:
 * @code
 * i2c_bus_t bus;
 * i2c_bus_config_t config = {
 *     .port = I2C_NUM_0,
 *     .sda_io_num = GPIO_NUM_21,
 *     .scl_io_num = GPIO_NUM_22
 * };
 * esp_err_t ret = i2c_bus_init(&bus, &config);
 * if (ret == ESP_OK) {
 *     // Bus initialized successfully, can now add devices
 * }
 * @endcode
 */
esp_err_t i2c_bus_init(i2c_bus_t *bus, const i2c_bus_config_t *config);

#endif // _I2C_BUS_H_
