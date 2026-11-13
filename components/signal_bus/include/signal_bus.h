#ifndef _SIGNAL_BUS_H
#define _SIGNAL_BUS_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

// Motor event bits
// Overcurrent, one bit per motor port: CURRENT_0 = port 1 (HAT 1 M1) ...
// CURRENT_7 = port 8 (HAT 2 M4)
#define CURRENT_NUM_BITS 8
#define CURRENT_BIT(port_index) (1 << (port_index))  // port_index 0-7
#define CURRENT_0   (1 << 0)
#define CURRENT_1   (1 << 1)
#define CURRENT_2   (1 << 2)
#define CURRENT_3   (1 << 3)
#define CURRENT_4   (1 << 4)
#define CURRENT_5   (1 << 5)
#define CURRENT_6   (1 << 6)
#define CURRENT_7   (1 << 7)

#define LIMIT_0 (1 << 8)
#define LIMIT_1 (1 << 9)
#define LIMIT_2 (1 << 10)
#define LIMIT_3 (1 << 11)
#define LIMIT_4 (1 << 12)
#define LIMIT_5 (1 << 13)
#define LIMIT_6 (1 << 14)

#define HOMING_FLAG (1 << 15)

#define CURRENT_ANY (0x00FF)  // bits 0-7
#define LIMIT_ANY   (0x7F00)  // bits 8-14
#define HOME_EVENT  (CURRENT_ANY | LIMIT_ANY)
#define EVENT_ANY   (HOME_EVENT | HOMING_FLAG)

extern EventGroupHandle_t g_motor_events;

/**
 * @brief Initialize the signal bus
 *
 * This function initializes the signal bus and sets up the necessary resources.
 *
 * @return
 *    - ESP_OK: Success
 *    - ESP_ERR_INVALID_ARG: Invalid argument
 *    - ESP_FAIL: Failed to initialize signal bus
 */
esp_err_t signal_bus_init();

#endif  // _SIGNAL_BUS_H