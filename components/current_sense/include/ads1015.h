#pragma once

#include "esp_err.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include <ads111x.h>

typedef struct {
    gpio_num_t alert_gpio;
    i2c_port_t i2c_port;
    uint8_t i2c_addr;
    gpio_num_t sda_io_num;
    gpio_num_t scl_io_num;
} ads1015_config_t;

typedef struct {
    gpio_num_t alert_gpio;
    i2c_dev_t dev;
    SemaphoreHandle_t rdy_sem;
} ads1015_handle_t;

ads1015_handle_t *ads1015_create(void);
void ads1015_destroy(ads1015_handle_t *ads);

esp_err_t ads1015_init(ads1015_handle_t *ads, const ads1015_config_t *config);
