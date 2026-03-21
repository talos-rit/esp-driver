#ifndef _SIGNAL_BUS_H
#define _SIGNAL_BUS_H

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_err.h"

// Motor limit switch and fault event bits
#define LIMIT_MOTOR_0     (1 << 0)
#define LIMIT_MOTOR_1     (1 << 1)
#define LIMIT_MOTOR_2     (1 << 2)
#define LIMIT_MOTOR_3     (1 << 3)
#define LIMIT_MOTOR_4     (1 << 4)
#define LIMIT_MOTOR_5     (1 << 5)
#define LIMIT_MOTOR_6     (1 << 6)
#define ESTOP_OVERCURRENT (1 << 7)

#define ESTOP_ANY       (ESTOP_OVERCURRENT)
#define LIMIT_ANY       (0x7F)  // bits 0-6
#define FAULT_ANY       (LIMIT_ANY | ESTOP_ANY)

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

#endif // _SIGNAL_BUS_H