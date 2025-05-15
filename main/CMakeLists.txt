idf_component_register(SRCS "webserver.c" "wifi_manager.c" "time_sync.c" "app_Main.c"
                    PRIV_REQUIRES nvs_flash esp_driver_gpio
                    REQUIRES "esp_wifi" "esp_http_server"
                    INCLUDE_DIRS ".")
