#ifndef _I2C_BUS_H_
#define _I2C_BUS_H_

#include "driver/i2c_types.h"
#include "soc/gpio_num.h"
#include "esp_err.h"

typedef struct {
  i2c_port_t port;
  gpio_num_t sda_io_num;
  gpio_num_t scl_io_num;
} i2c_bus_config_t;

typedef struct {
  i2c_master_bus_handle_t handle;
} i2c_bus_t;

esp_err_t i2c_bus_init(i2c_bus_t *bus, const i2c_bus_config_t *config);

#endif // _I2C_BUS_H_
