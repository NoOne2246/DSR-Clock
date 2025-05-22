idf_component_register(SRCS "TMC2209.c"
                    PRIV_REQUIRES esp_timer
                    REQUIRES driver
                    INCLUDE_DIRS ".")
