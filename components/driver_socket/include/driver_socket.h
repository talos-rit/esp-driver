#ifndef _DRIVER_SOCKET_H_
#define _DRIVER_SOCKET_H_

#include "freertos/FreeRTOS.h"
#include "sys/socket.h"

typedef struct {
  int fd;
} driver_socket_handle_t;

typedef struct {
  const char *ip;
  const char *port;
} driver_socket_config_t;

typedef struct {
  driver_socket_handle_t socket;
  driver_socket_config_t config;
  SemaphoreHandle_t server_ready;
} task_args_t;

/**
 * @brief Initializes the socket driver and starts the socket task. This
 * function blocks until the socket task is ready to accept connections.
 *
 * @param[out] socket_handle Socket handle to be initialized by this function
 * @param[in] config Socket configuration
 * @return
 *          ESP_OK : Socket driver initialized successfully
 *          ESP_ERR_NO_MEM : Unable to allocate memory for task arguments or
 * server_ready semaphore
 */
esp_err_t driver_socket_init(driver_socket_handle_t *socket_handle,
                             const driver_socket_config_t *config);

/**
 * @brief Utility to log socket errors
 *
 * @param[in] tag Logging tag
 * @param[in] sock Socket number
 * @param[in] err Socket errno
 * @param[in] message Message to print
 */
void log_socket_error(const char *tag, const int sock, const int err,
                             const char *message);

/**
 * @brief Returns the string representation of client's address (accepted on
 * this server)
 */
char *get_clients_address(struct sockaddr_storage *source_addr);

/**
 * @brief Tries to receive data from specified sockets in a non-blocking way,
 *        i.e. returns immediately if no data.
 *
 * @param[in] tag Logging tag
 * @param[in] sock Socket for reception
 * @param[out] data Data pointer to write the received data
 * @param[in] max_len Maximum size of the allocated space for receiving data
 * @return
 *          >0 : Size of received data
 *          =0 : No data available
 *          -1 : Error occurred during socket read operation
 *          -2 : Socket is not connected, to distinguish between an actual
 * socket error and active disconnection
 */
int try_receive(const char *tag, const int sock, uint8_t *data,
                       size_t max_len);

/**
 * @brief Sends the specified data to the socket. This function blocks until all
 * bytes got sent.
 *
 * @param[in] tag Logging tag
 * @param[in] sock Socket to write data
 * @param[in] data Data to be written
 * @param[in] len Length of the data
 * @return
 *          >0 : Size the written data
 *          -1 : Error occurred during socket write operation
 */
int socket_send(const char *tag, const int sock, const uint8_t *data,
                       const size_t len);

#endif // _DRIVER_SOCKET_H_
