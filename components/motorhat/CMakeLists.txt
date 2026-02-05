idf_component_register(SRCS "motorhat.c" "pca9685.c"
                    INCLUDE_DIRS "include" REQUIRES esp_driver_i2c i2c_bus)
