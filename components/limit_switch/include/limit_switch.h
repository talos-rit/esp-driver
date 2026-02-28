#ifndef _LIMIT_SWITCH_H
#define _LIMIT_SWITCH_H

#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

#define LIMIT_SWITCH_I2C_MASTER_TIMEOUT_MS 1000

/**
 * @brief limit_switch configuration structure
 */
typedef struct {
    gpio_num_t alert_gpio;                /**< GPIO number for ALERT pin */
} limit_switch_config_t;

/**
 * @brief Initialize the limit switch task
 *
 * This function initializes the limit switch task and sets up the alert GPIO.
 *
 * @param[in] config Pointer to configuration structure
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument
 */
esp_err_t limit_switch_init(const limit_switch_config_t *config);

#endif // _LIMIT_SWITCH_H