#include "i2c_bus.h"
#include "driver/i2c_master.h"

esp_err_t i2c_bus_init(i2c_bus_t *bus, const i2c_bus_config_t *config) {

  i2c_master_bus_config_t bus_config = {
      .i2c_port = config->port,
      .sda_io_num = config->sda_io_num,
      .scl_io_num = config->scl_io_num,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };

  return i2c_new_master_bus(&bus_config, &bus->handle);
}
