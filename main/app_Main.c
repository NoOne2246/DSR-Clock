// #include <stdio.h>
// #include <string.h>
// #include <time.h>
// #include "esp_system.h"
// #include "esp_event.h"
// 
// #include "esp_attr.h"
// #include "esp_sleep.h"
// #include "nvs_flash.h"
// #include "protocol_examples_common.h"
// #include "esp_netif_sntp.h"
// #include "lwip/ip_addr.h"
// #include "esp_sntp.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/event_groups.h"

#include "driver/gpio.h"
#include "esp_log.h"

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <ds3231.h>
#include <string.h>

#define HOURSENSOR GPIO_NUM_6
#define MINUTESENSOR GPIO_NUM_7

// static void obtain_time(void);

void ds3231_test(void *pvParameters)
{
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(ds3231_init_desc(&dev, 0, GPIO_NUM_8, GPIO_NUM_9));

    // setup datetime: 2016-10-09 13:50:10
    struct tm time = {
        .tm_year = 116, //since 1900 (2016 - 1900)
        .tm_mon  = 9,  // 0-based
        .tm_mday = 9,
        .tm_hour = 13,
        .tm_min  = 50,
        .tm_sec  = 10
    };
    ESP_ERROR_CHECK(ds3231_set_time(&dev, &time));

    while (1)
    {
        float temp;

        vTaskDelay(pdMS_TO_TICKS(250));

        if (ds3231_get_time(&dev, &time) != ESP_OK)
        {
            printf("Could not get time\n");
            continue;
        }

        /* float is used in printf(). you need non-default configuration in
         * sdkconfig for ESP8266, which is enabled by default for this
         * example. see sdkconfig.defaults.esp8266
         */
        printf("%04d-%02d-%02d %02d:%02d:%02d, %.2f deg Cel\n", time.tm_year + 1900 /*Add 1900 for better readability*/, time.tm_mon + 1,
            time.tm_mday, time.tm_hour, time.tm_min, time.tm_sec, temp);
    }
}


void app_main(void)
{
    //----------------------------
    //**    Initialise Time     **
    //----------------------------
    
    // // Attempt to get time from ntp
    // time_t now;
    // struct tm timeinfo;
    // time(&now);
    // localtime_r(&now, &timeinfo);
    // if(timeinfo.tm_year < (2021 - 1900)){
    //     obtain_time();
    // }
    // time(&now);
    // setenv("TZ","Australia/Sydney",1);
    // tzset();
    // time(&now);

    // Get time from RTC if ntp failed
    
    ESP_ERROR_CHECK(i2cdev_init());

    //----------------------------
    //**   Initialise Sensor    **
    //----------------------------
    ESP_LOGI("Setup","Initializing Sensors");
    gpio_set_direction(HOURSENSOR, GPIO_MODE_INPUT);
    gpio_set_pull_mode(HOURSENSOR, GPIO_PULLDOWN_ONLY);

    gpio_set_direction(MINUTESENSOR, GPIO_MODE_INPUT);
    gpio_set_pull_mode(MINUTESENSOR, GPIO_PULLDOWN_ONLY);
    ESP_LOGI("Setup","Input Sensor Initialized");

    //----------------------------
    //**    Initialise Motor    **
    //----------------------------


    //----------------------------
    //**      Loop Update       **
    //----------------------------

}

// static void obtain_time(void)
// {
//     ESP_ERROR_CHECK(nvs_flash_init());
//     ESP_ERROR_CHECK(esp_netif_init());
//     ESP_ERROR_CHECK(esp_event_loop_create_default());
//     ESP_ERROR_CHECK(example_connect());

//     esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
//     esp_sntp_setservername(0,"pool.ntp.org");

//     time_t now = 0;
//     struct tm timeinfo = {0};
//     int retry = 0;
//     const int retry_count = 10;
//     while(sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count){
//         vTaskDelay(2000 / portTICK_PERIOD_MS);
//     }
//     time(&now);
//     localtime_r(&now, &timeinfo);
//     ESP_ERROR_CHECK(example_disconnect());
// }
