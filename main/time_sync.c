#include "time_sync.h"
#include "esp_sntp.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "ds3231.h"
#include <time.h>
#include <sys/time.h>
#include "nvs_flash.h"

#include "gpio_definitions.h"

#define RETRY_COUNT 5
#define RETRY_DELAY_MS 5000


esp_err_t wifi_connect(void) {
    ESP_LOGI("WiFi", "Initializing Wi-Fi...");

    ESP_ERROR_CHECK(esp_netif_init());  // Initialize network interfaces
    ESP_ERROR_CHECK(esp_event_loop_create_default());  // Create event loop
    esp_netif_create_default_wifi_sta();  // Configure as a station (STA mode)

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = "WiFi-12A6-5G",      // Replace with your Wi-Fi SSID
            .password = "04371304",  // Replace with your Wi-Fi password
        }
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));  // Set Wi-Fi to station mode
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));  // Apply config

    ESP_ERROR_CHECK(esp_wifi_start());  // Start Wi-Fi

    ESP_LOGI("WiFi", "Wi-Fi setup complete, attempting to connect...");
    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGE("WiFi", "Wi-Fi connect failed! Error code: %d", err);
        return err;
    }
    ESP_LOGI("WiFi", "Wi-Fi connected successfully.");
    return ESP_OK;
}

esp_err_t wifi_disconnect(void) {
    esp_err_t err;

    err = esp_wifi_disconnect();
    if (err != ESP_OK) {
        ESP_LOGE("WiFi", "Wi-Fi disconnect failed! Error code: %d", err);
        return err;
    }

    err = esp_wifi_stop();
    if (err != ESP_OK) {
        ESP_LOGE("WiFi", "Wi-Fi stop failed! Error code: %d", err);
        return err;
    }

    ESP_LOGI("WiFi", "Wi-Fi disconnected successfully.");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    return ESP_OK;
}


bool obtain_time_sntp(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    ESP_ERROR_CHECK(wifi_connect());

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

    bool success = (sntp_get_sync_status() != SNTP_SYNC_STATUS_RESET);
    if (success) {
        ESP_LOGI("SNTP", "Time synchronized successfully!");
        ESP_ERROR_CHECK(wifi_disconnect());  // Disconnect only if sync succeeds
    } else {
        ESP_LOGE("SNTP", "Failed to synchronize time. Keeping Wi-Fi active for retry.");
    }
    return success;
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
    struct tm timeinfo;
    time(&now);
    gmtime_r(&now,&timeinfo);
    struct tm rtctime = {
        .tm_year = timeinfo.tm_year,
        .tm_mon = timeinfo.tm_mon,
        .tm_mday = timeinfo.tm_mday,
        .tm_hour = timeinfo.tm_hour,
        .tm_min = timeinfo.tm_min,
        .tm_sec = timeinfo.tm_sec
    };

    int retry = 0;
    while(ds3231_set_time(&dev, &rtctime) != ESP_OK && ++retry < RETRY_COUNT){
        ESP_LOGI("rtc", "Attempting to get time from rtc... (%d/%d)", retry, RETRY_COUNT);
        vTaskDelay(RETRY_DELAY_MS / portTICK_PERIOD_MS);
    }
    return retry != RETRY_COUNT;
}

void set_time(void)
{
    if(obtain_time_sntp()){
        if (!set_time_rtc()) {
            ESP_LOGE("RTC", "Failed to set RTC time.");
        }
        return;
    }
    if (obtain_time_rtc()){
        return;
    }
    ESP_LOGE("Time", "Unable to set time after multiple attempts");
    while(1){vTaskDelay(1);}
}