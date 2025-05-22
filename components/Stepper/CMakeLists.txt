idf_component_register(SRCS "stepper.c"
                    PRIV_REQUIRES esp_driver_gpio
                    REQUIRES "accelstepper" "tmc2209"
                    INCLUDE_DIRS ".")
