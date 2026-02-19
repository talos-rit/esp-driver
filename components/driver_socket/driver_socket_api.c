#include "driver_socket_api.h"
#include "esp_log.h"
#include "freertos//FreeRTOS.h"

#define TAG "driver_socket_api"

esp_err_t driver_socket_api_process(uint8_t *buffer, size_t buffer_size) {
  ESP_LOGI(TAG, "Received data size: %d", buffer_size);
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, buffer, buffer_size, ESP_LOG_INFO);
  return ESP_OK;
}
