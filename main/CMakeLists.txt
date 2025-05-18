idf_component_register(SRCS "stepper_control.cpp" "time_handler.c" "nvs_handler.c" "webserver.c" "wifi_manager.c" "app_Main.c" "util.c"
                    PRIV_REQUIRES nvs_flash esp_driver_gpio
                    REQUIRES "esp_wifi" "esp_http_server" "Stepper"
                    INCLUDE_DIRS "." "../components/Stepper/include/")
