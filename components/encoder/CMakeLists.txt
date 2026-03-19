idf_component_register(SRCS "encoder.c"
                    PRIV_REQUIRES  esp_driver_gpio
                    REQUIRES esp_driver_pcnt
                    INCLUDE_DIRS "include")
