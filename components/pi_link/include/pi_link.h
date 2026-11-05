#ifndef _PI_LINK_H_
#define _PI_LINK_H_

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

/**
 * @brief Motion functions the Pi link drives
 */
typedef struct {
  esp_err_t (*polar_pan_start)(int8_t delta_azimuth, int8_t delta_altitude);
  esp_err_t (*polar_pan_stop)(void);
  esp_err_t (*home)(uint16_t delay_ms);
  esp_err_t (*set_speed_percent)(uint8_t percent); /**< SPEED n, 1 to 100 */
} pi_link_motor_interface_t;

/**
 * @brief Pi link configuration structure
 */
typedef struct {
  int baud_rate;    /**< Must match the Pi's tty */
  bool debug_reply; /**< Reply with what was parsed. Bench only */
  pi_link_motor_interface_t motor; /**< Motion functions, all required */
} pi_link_config_t;

/**
 * @brief Take over UART0 and start reading ACL from the Pi's operator (erv)
 *
 * @param[in] config Pointer to configuration structure
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument
 *    - ESP_ERR_NO_MEM: Task could not be created
 *    - ESP_ERR_*: Errors from the UART driver
 */
esp_err_t pi_link_init(const pi_link_config_t* config);

#endif  // _PI_LINK_H_
