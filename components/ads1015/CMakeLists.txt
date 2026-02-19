idf_component_register(
  SRCS "ads1015.c"
  INCLUDE_DIRS "include"
  REQUIRES esp_driver_i2c esp_driver_gpio i2c_bus
)