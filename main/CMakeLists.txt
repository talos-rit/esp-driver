idf_component_register(SRCS "wiring_check.c" "esp_driver.c"
                    INCLUDE_DIRS "." PRIV_REQUIRES i2c_bus motorhat driver_socket driver_wifi ads1015 encoder limit_switch signal_bus esp_driver_uart)
