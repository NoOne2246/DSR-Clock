if(${IDF_TARGET} STREQUAL esp8266)
    set(req "esp8266" "freertos" "esp_idf_lib_helpers")
else()
    set(req "driver" "freertos" "esp_idf_lib_helpers")
endif()

idf_component_register(
    SRCS "i2cdev.c"
    INCLUDE_DIRS "."
    REQUIRES ${req}
)
