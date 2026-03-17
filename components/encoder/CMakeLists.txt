idf_component_register(SRCS "encoder.c"
                    REQUIRES esp_driver_pcnt esp_driver_gpio
                    INCLUDE_DIRS "include")
