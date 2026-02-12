#ifndef _DRIVER_WIFI_H_
#define _DRIVER_WIFI_H_

#include "esp_err.h"
typedef struct {
  uint8_t ssid[32];
  uint8_t password[64];
} driver_wifi_config_t;

esp_err_t wifi_init(driver_wifi_config_t *config);

esp_err_t wait_for_wifi_connection();

#endif // _WIFI_H
