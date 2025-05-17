#include "driver/gpio.h"

#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include <time.h>
#include "nvs_flash.h"
#include "esp_netif.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "time_handler.h"
#include "wifi_manager.h"
#include "webserver.h"
#include "nvs_handler.h"

#include "gpio_definitions.h"

#define WIFI_SSID "WiFi-12A6"
#define WIFI_PASSWORD "04371304"

void getClock(void *pvParameters)
{
	
	// Initialise the xLastWakeTime variable with the current time.
	TickType_t xLastWakeTime = xTaskGetTickCount();

	// Get RTC date and time
	while (1) {
		time_t now;
        struct tm timeinfo;
        char strftime_buf[64];
        time(&now);
        localtime_r(&now, &timeinfo);
        strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
        ESP_LOGI(pcTaskGetName(0), "The current date/time is: %s", strftime_buf);
	    vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10000));
	}
}

void app_main(void)
{
    
    //----------------------------
    //**    Initialise NVS      **
    //----------------------------
    init_nvs();

    //----------------------------
    //**    Initialise Time     **
    //----------------------------
        
    set_TZ(NULL);
    
    // ESP_ERROR_CHECK(wifi_init());
    // wifi_connect(WIFI_SSID, WIFI_PASSWORD);
    // start_webserver();

    xTaskCreate(set_time, "Set Time", 4 * 1024, NULL, 3, NULL);


    //----------------------------
    //**   Initialise Sensor    **
    //----------------------------
    ESP_LOGI("Setup", "Initializing Sensors");
    gpio_set_direction(SENSOR_HOUR, GPIO_MODE_INPUT);
    gpio_set_pull_mode(SENSOR_HOUR, GPIO_PULLDOWN_ONLY);

    gpio_set_direction(SENSOR_MINUTE, GPIO_MODE_INPUT);
    gpio_set_pull_mode(SENSOR_MINUTE, GPIO_PULLDOWN_ONLY);
    ESP_LOGI("Setup", "Input Sensor Initialized");

    //----------------------------
    //**    Initialise Motor    **
    //----------------------------

    static httpd_handle_t server = NULL;
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_AP_STAIPASSIGNED, &connect_handler, &server));

    //----------------------------
    //**      Loop Update       **
    //----------------------------

    xTaskCreate(getClock, "Show Time", 4*1024, NULL, 3, NULL);
}

