#ifndef _SIGNAL_BUS_H
#define _SIGNAL_BUS_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

// Motor event bits
#define CURRENT_0   (1 << 0)
#define CURRENT_1   (1 << 1)
#define CURRENT_2   (1 << 2)
#define CURRENT_3   (1 << 3)
#define CURRENT_4   (1 << 4)
#define CURRENT_5   (1 << 5)
#define CURRENT_6   (1 << 6)

#define LIMIT_0 (1 << 7)
#define LIMIT_1 (1 << 8)
#define LIMIT_2 (1 << 9)
#define LIMIT_3 (1 << 10)
#define LIMIT_4 (1 << 11)
#define LIMIT_5 (1 << 12)
#define LIMIT_6 (1 << 13)

#define HOMING_FLAG (1 << 14)

#define CURRENT_ANY (0x007F)  // bits 0-6
#define LIMIT_ANY   (0x3F80)  // bits 7-13
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