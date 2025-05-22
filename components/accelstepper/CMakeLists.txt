idf_component_register(SRCS "MultiStepper.c" "AccelStepper.c"
                    PRIV_REQUIRES esp_driver_gpio esp_timer esp_driver_uart
                    INCLUDE_DIRS ".")
