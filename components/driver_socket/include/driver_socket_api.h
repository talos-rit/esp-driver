#ifndef _DRIVER_SOCKET_API_H_
#define _DRIVER_SOCKET_API_H_

#include "freertos/FreeRTOS.h"
#include <stdint.h>

extern QueueHandle_t driver_socket_queue;

esp_err_t driver_socket_api_process(uint8_t* buffer, size_t buffer_size);

#endif // _DRIVER_SOCKET_API_H_
