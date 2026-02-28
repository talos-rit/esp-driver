idf_component_register(SRCS "esp_driver.c"
                    INCLUDE_DIRS "." PRIV_REQUIRES i2c_bus motorhat driver_socket driver_wifi ads1015)
