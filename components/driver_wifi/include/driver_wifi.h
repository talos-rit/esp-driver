#ifndef _DRIVER_WIFI_H_
#define _DRIVER_WIFI_H_

#include "esp_err.h"

/** WiFi configuration structure */
typedef struct {
  uint8_t ssid[32];
  uint8_t password[64];
} driver_wifi_config_t;

/**
  * @brief Initializes the WiFi driver and connects to the specified network. This
  * does not block until the connection is established, so you should call
  * wait_for_wifi_connection() after this to block until the connection is ready.
  * @param[in] config WiFi configuration (SSID and password)
  * @return
  *          ESP_OK : WiFi initialization started successfully
  *          ESP_ERR_INVALID_ARG : Invalid configuration (e.g., empty SSID)
  *          ESP_ERR_WIFI_NOT_INIT : WiFi driver not initialized
  *          ESP_ERR_WIFI_CONN : Failed to start WiFi connection process
  */
esp_err_t wifi_init(driver_wifi_config_t *config);

/**
  * @brief Blocks until the WiFi connection is established. This should be called
  * after wifi_init() to wait for the connection to be ready before proceeding.
  * @return
  *          ESP_OK : WiFi connected successfully
  *          ESP_ERR_WIFI_NOT_INIT : WiFi driver not initialized
  *          ESP_ERR_WIFI_CONN : Failed to connect to WiFi (e.g., wrong password)
  */
esp_err_t wait_for_wifi_connection();

#endif // _WIFI_H
