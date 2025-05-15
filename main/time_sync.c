#include "time_sync.h"
#include "esp_sntp.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "ds3231.h"
#include <time.h>
#include <sys/time.h>

#include "gpio_definitions.h"

#define RETRY_COUNT 5
#define RETRY_DELAY_MS 5000


bool obtain_time_sntp(void)
{
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        ESP_LOGI("WiFi", "Already connected, proceeding to SNTP sync.");
    } else {
        ESP_LOGW("WiFi", "Wi-Fi not connected, unable to connect to Wi-Fi.");
        return false;
    }

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0,"pool.ntp.org");
    esp_sntp_init();

    int retry = 0;
    while(sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < RETRY_COUNT){
        ESP_LOGI("ntp", "Attempting to get time from ntp... (%d/%d)", retry, RETRY_COUNT);
        vTaskDelay(RETRY_DELAY_MS / portTICK_PERIOD_MS);
    }
    
    return retry != RETRY_COUNT;
}

bool obtain_time_rtc(void)
{
    ESP_ERROR_CHECK(i2cdev_init());
    
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(ds3231_init_desc(&dev, 0, SDA_PIN, SCL_PIN));

    struct tm rtctime;

    int retry = 0;
    while(ds3231_get_time(&dev, &rtctime) != ESP_OK && ++retry < RETRY_COUNT){
        ESP_LOGI("rtc", "Attempting to get time from rtc... (%d/%d)", retry, RETRY_COUNT);
        vTaskDelay(RETRY_DELAY_MS / portTICK_PERIOD_MS);
    }
    if(retry != RETRY_COUNT){
        struct timeval tv = {
            .tv_sec = mktime(&rtctime),
            .tv_usec = 0
        };
        settimeofday(&tv, NULL);
    }
    
    return retry != RETRY_COUNT;
}

bool set_time_rtc(void)
{
    ESP_ERROR_CHECK(i2cdev_init());
    
    i2c_dev_t dev;
    memset(&dev, 0, sizeof(i2c_dev_t));

    ESP_ERROR_CHECK(ds3231_init_desc(&dev, 0, SDA_PIN, SCL_PIN));

    time_t now;
    struct tm rtctime;
    time(&now);
    gmtime_r(&now,&rtctime);

    int retry = 0;
    while(ds3231_set_time(&dev, &rtctime) != ESP_OK && ++retry < RETRY_COUNT){
        ESP_LOGI("rtc", "Attempting to set time to rtc... (%d/%d)", retry, RETRY_COUNT);
        vTaskDelay(RETRY_DELAY_MS / portTICK_PERIOD_MS);
        time(&now);
        gmtime_r(&now,&rtctime);
    }
    return retry != RETRY_COUNT;
}

void set_time(void * pvParameter)
{
    while(1) {
        if(obtain_time_sntp()){
            set_time_rtc();
            vTaskDelete(NULL);
        }
        if (obtain_time_rtc()){
            vTaskDelete(NULL);
        }
        vTaskDelay(60*60*1000/portTICK_PERIOD_MS);
    }
}