idf_component_register(SRCS "app_Main.c"
                    PRIV_REQUIRES nvs_flash esp_driver_gpio
                    REQUIRES "esp_wifi" 
                    INCLUDE_DIRS ".")
