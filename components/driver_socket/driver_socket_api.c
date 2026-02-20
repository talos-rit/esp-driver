#include "driver_socket_api.h"
#include "endian.h"
#include "esp_log.h"
#include "freertos//FreeRTOS.h"

#define TAG "driver_socket_api"

static void
driver_socket_api_header_swap_endianess(driver_socket_api_header_t *header) {
  header->msg_id = be32toh(header->msg_id);
  header->cmd_id = be16toh(header->cmd_id);
  header->len = be16toh(header->len);
}

static void driver_socket_api_polar_pan_payload_swap_endianess(driver_socket_api_polar_pan_payload_t *payload) {
  payload->delta_azimuth = be32toh(payload->delta_azimuth);
  payload->delta_altitude = be32toh(payload->delta_altitude);
  payload->delay_ms = be32toh(payload->delay_ms);
  payload->time_ms = be32toh(payload->time_ms);
}

static void driver_socket_api_home_payload_swap_endianess(driver_socket_api_home_payload_t *payload) {
  payload->delay_ms = be32toh(payload->delay_ms);
}

static uint8_t xor_checksum(uint8_t *data, size_t len) {
  uint8_t result = 0;
  for (size_t i = 0; i < len; i++) {
    result ^= data[i];
  }
  return result;
}

static esp_err_t validate_checksum(uint8_t *data, size_t len, uint8_t checksum) {
  uint8_t calculated_checksum = xor_checksum((uint8_t *)data, len);
  return (calculated_checksum == checksum) ? ESP_OK : ESP_ERR_INVALID_CRC;
}

esp_err_t driver_socket_api_process(uint8_t *buffer, size_t buffer_size) {
  ESP_LOGI(TAG, "Received data size: %d", buffer_size);
  ESP_LOG_BUFFER_HEX_LEVEL(TAG, buffer, buffer_size, ESP_LOG_INFO);

  esp_err_t err = validate_checksum(buffer, buffer_size - 1, buffer[buffer_size - 1]);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Checksum validation failed");
    return err;
  }

  driver_socket_api_wrapper_t *wrapper = (driver_socket_api_wrapper_t *)buffer;

  driver_socket_api_header_swap_endianess(&wrapper->header);

  ESP_LOGI(TAG, "Parsed header: msg_id=%d, cmd_id=%d, len=%d",
           wrapper->header.msg_id, wrapper->header.cmd_id, wrapper->header.len);

  switch (wrapper->header.cmd_id) {
    case DRIVER_SOCKET_API_CMD_ID_HANDSHAKE:
      ESP_LOGI(TAG, "Processing HANDSHAKE command");
      break;
    case DRIVER_SOCKET_API_CMD_ID_POLAR_PAN:
      driver_socket_api_polar_pan_payload_t *polar_pan_payload = (driver_socket_api_polar_pan_payload_t *)&wrapper->payload_head;
      driver_socket_api_polar_pan_payload_swap_endianess(polar_pan_payload);
      ESP_LOGI(TAG, "Processing POLAR_PAN command: delta_azimuth=%d, delta_altitude=%d, delay_ms=%d, time_ms=%d",
               polar_pan_payload->delta_azimuth, polar_pan_payload->delta_altitude, polar_pan_payload->delay_ms, polar_pan_payload->time_ms);
      break;
    case DRIVER_SOCKET_API_CMD_ID_HOME:
      driver_socket_api_home_payload_t *home_payload = (driver_socket_api_home_payload_t *)&wrapper->payload_head;
      driver_socket_api_home_payload_swap_endianess(home_payload);
      ESP_LOGI(TAG, "Processing HOME command: delay_ms=%d", home_payload->delay_ms);
      break;
    case DRIVER_SOCKET_API_CMD_ID_POLAR_PAN_START:
      driver_socket_api_polar_pan_start_payload_t *polar_pan_start_payload = (driver_socket_api_polar_pan_start_payload_t *)&wrapper->payload_head;
      // No endianess swap needed for int8_t fields
      ESP_LOGI(TAG, "Processing POLAR_PAN_START command: delta_azimuth=%d, delta_altitude=%d",
               polar_pan_start_payload->delta_azimuth, polar_pan_start_payload->delta_altitude);
      break;
    case DRIVER_SOCKET_API_CMD_ID_POLAR_PAN_STOP:
      ESP_LOGI(TAG, "Processing POLAR_PAN_STOP command");
      break;
    default:
      ESP_LOGW(TAG, "Unknown command ID: %d", wrapper->header.cmd_id);
      return ESP_ERR_INVALID_ARG;
  }

  return ESP_OK;
}
