#ifndef _DRIVER_SOCKET_API_H_
#define _DRIVER_SOCKET_API_H_

#include "freertos/FreeRTOS.h"
#include <stdint.h>

/** Command IDs for the socket API */
typedef enum {
  DRIVER_SOCKET_API_CMD_ID_HANDSHAKE = 0x0000,
  DRIVER_SOCKET_API_CMD_ID_POLAR_PAN = 0x0001,
  DRIVER_SOCKET_API_CMD_ID_HOME = 0x0002,
  DRIVER_SOCKET_API_CMD_ID_POLAR_PAN_START = 0x0003,
  DRIVER_SOCKET_API_CMD_ID_POLAR_PAN_STOP = 0x0004,
} driver_socket_api_cmd_id_t;

/** Header Struct */
typedef struct {
  uint32_t msg_id;
  uint16_t reserved_1;
  uint16_t cmd_id;
  uint16_t len;
} __attribute__((packed)) driver_socket_api_header_t;

/** Wrapper struct for the entire message, including header and payload. The payload
 * is variable length, so we can't define it as a fixed array. Instead, we can
 * use a flexible array member or just treat it as a pointer when processing.
 */
typedef struct {
  driver_socket_api_header_t header;
  uint8_t payload_head;

  // Since the payload is variable length, we can't define it as a fixed array.

  // uint_t checksum;
} __attribute__((packed)) driver_socket_api_wrapper_t;


/** Payload structures for different command types */
typedef struct {
  int32_t delta_azimuth;  /** Requested change in azimuth */
  int32_t delta_altitude; /** Requested change in altitude */
  uint32_t delay_ms;      /** How long to wait until executing pan */
  uint32_t time_ms;       /** How long the pan should take to execute */
} __attribute__((packed)) driver_socket_api_polar_pan_payload_t;

typedef struct {
  uint32_t delay_ms; /** How long to wait until executing pan */
} __attribute__((packed)) driver_socket_api_home_payload_t;

typedef struct {
  int8_t delta_azimuth;  /** Requested change in azimuth */
  int8_t delta_altitude; /** Requested change in altitude */
} __attribute__((packed)) driver_socket_api_polar_pan_start_payload_t;


/**
* @brief Process incoming socket API messages. This function should be called
* from the socket task when a complete message has been received. It will parse
* the message, validate it, and execute the corresponding command.
* @param[in] buffer Pointer to the received message buffer
* @param[in] buffer_size Size of the received message buffer
* @return
*          ESP_OK : Message processed successfully
*          ESP_ERR_INVALID_ARG : Invalid message format or command ID
*          ESP_ERR_INVALID_SIZE : Message size does not match expected size for the command
*          ESP_ERR_INVALID_CRC : Checksum validation failed (if implemented)
*          ESP_ERR_FAIL : Command execution failed (e.g., invalid parameters)
*/
esp_err_t driver_socket_api_process(uint8_t *buffer, size_t buffer_size);

#endif // _DRIVER_SOCKET_API_H_
