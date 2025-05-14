#include "driver/gpio.h"

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include <time.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "time_sync.h"

#include "gpio_definitions.h"

void app_main(void)
{
    //----------------------------
    //**    Initialise Time     **
    //----------------------------
    
    setenv("TZ", "AEST-10AEDT,M10.1.0,M4.1.0", 1);
    tzset();

    set_time();

    //----------------------------
    //**   Initialise Sensor    **
    //----------------------------
    ESP_LOGI("Setup","Initializing Sensors");
    gpio_set_direction(SENSOR_HOUR, GPIO_MODE_INPUT);
    gpio_set_pull_mode(SENSOR_HOUR, GPIO_PULLDOWN_ONLY);

    gpio_set_direction(SENSOR_MINUTE, GPIO_MODE_INPUT);
    gpio_set_pull_mode(SENSOR_MINUTE, GPIO_PULLDOWN_ONLY);
    ESP_LOGI("Setup","Input Sensor Initialized");

    //----------------------------
    //**    Initialise Motor    **
    //----------------------------


    //----------------------------
    //**      Loop Update       **
    //----------------------------

}

