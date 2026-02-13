#pragma once

#include "esp_err.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <ads111x.h>

typedef struct {
    gpio_num_t alert_gpio;
} ads1015_config_t;

typedef struct ads1015_handle_t ads1015_handle_t;

ads1015_handle_t *ads1015_create(void);
void ads1015_destroy(ads1015_handle_t *ads);

esp_err_t ads1015_init(ads1015_handle_t *ads, const ads1015_config_t *config);
