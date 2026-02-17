idf_component_register(SRCS "esp-driver.c"
                    INCLUDE_DIRS "." PRIV_REQUIRES i2c_bus motorhat driver_socket driver_wifi)
