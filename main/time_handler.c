#include "time_handler.h"
#include "tz_mapping.h"
#include "esp_sntp.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "ds3231.h"
#include <time.h>
#include <sys/time.h>
#include "nvs_handler.h"
#include "util.h"

#include "gpio_definitions.h"

#define RETRY_COUNT 5
#define RETRY_DELAY_MS 5000

static const char *TAG = "TIME_HANDLER";

TaskHandle_t set_time_task_handle = NULL; 

time_t my_timegm(struct tm *tm) {
    time_t ret;
    char *tz = getenv("TZ");  // Save current timezone

    setenv("TZ", "", 1);  // Temporarily set to UTC
    tzset();  // Apply timezone change

    ret = mktime(tm);  // Convert to time_t (now in UTC)

    // Restore original timezone
    if (tz) {
        setenv("TZ", tz, 1);
    } else {
        unsetenv("TZ");
    }
    tzset();

    return ret;
}


bool obtain_time_sntp(void)
{
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        ESP_LOGI(TAG, "Already connected, proceeding to SNTP sync.");
    } else {
        ESP_LOGW(TAG, "Wi-Fi not connected, unable to connect to Wi-Fi.");
        return false;
    }

    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0,"pool.ntp.org");
    esp_sntp_init();

    int retry = 0;
    while(sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < RETRY_COUNT){
        ESP_LOGI(TAG, "Attempting to get time from ntp... (%d/%d)", retry, RETRY_COUNT);
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
        ESP_LOGI(TAG, "Attempting to get time from rtc... (%d/%d)", retry, RETRY_COUNT);
        vTaskDelay(RETRY_DELAY_MS / portTICK_PERIOD_MS);
    }
    if(retry != RETRY_COUNT){
        struct timeval tv = {
            .tv_sec = my_timegm(&rtctime),
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
        ESP_LOGI(TAG, "Attempting to set time to rtc... (%d/%d)", retry, RETRY_COUNT);
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

void create_set_time_task() {
    
    ESP_LOGI(TAG, "Attempt to update time");
    if (set_time_task_handle == NULL) {  // Ensure task is not running
        xTaskCreate(set_time, "Set Time", 4 * 1024, NULL, 3, &set_time_task_handle);
    } else {
        vTaskDelete(set_time_task_handle);
        xTaskCreate(set_time, "Set Time", 4 * 1024, NULL, 3, &set_time_task_handle);
    }
}

void obtain_time_manual(const char *datetime_str) {
    struct tm local_time = {0};

    // Parse YYYY-MM-DDTHH:MM format
    if (sscanf(datetime_str, "%4d-%2d-%2dT%2d:%2d", 
               &local_time.tm_year, &local_time.tm_mon, &local_time.tm_mday, 
               &local_time.tm_hour, &local_time.tm_min) != 5) {
        ESP_LOGE(TAG, "Invalid datetime format.");
        return;
    }

    local_time.tm_year -= 1900;  // Adjust year for struct tm format
    local_time.tm_mon -= 1;  // Adjust month (0-based)
    local_time.tm_sec = 0; // Default seconds to 0
    
    // Convert local time to UTC epoch
    time_t utc_epoch = my_timegm(&local_time);
    
    // Set system time using UTC epoch
    struct timeval tv = {
        .tv_sec = utc_epoch,
        .tv_usec = 0
    };

    settimeofday(&tv, NULL);
    set_time_rtc();
}

/**
 * @brief Sets the system timezone.
 *
 * If a POSIX timezone identifier is provided, it updates the system timezone
 * and stores it in NVS for persistence. Otherwise, it retrieves the stored
 * timezone from NVS and applies it.
 *
 * @param tz_identifier Optional timezone identifier (e.g., "Australia/Sydney").
 */
void set_TZ(const char *tz_identifier) {
    const char *timezone;

    if (tz_identifier) {
        timezone = get_posix_from_id(tz_identifier); // Use modified temp_tz
        if(timezone == NULL){
            ESP_LOGI(TAG, "Failed to find matching timezone, no change applied");
            set_value_in_nvs("timezone", timezone);
        }
    } else {
        timezone = get_value_from_nvs("timezone", "GMT0");
    }
    if(timezone != NULL){
        ESP_LOGI(TAG, "Setting POSIX to: %s", timezone);
        setenv("TZ", timezone, 1);
        tzset();
    }
}

/**
 * @brief Retrieves the POSIX timezone string for a given IANA timezone identifier.
 *
 * This function searches the predefined `Mapping[]` array for a match with the provided
 * `zone_identifier`. If a match is found, it returns the corresponding POSIX timezone string.
 * If no match exists, it defaults to "GMT0".
 *
 * @param zone_identifier The IANA timezone identifier (e.g., "America/New_York").
 * @return const char* The corresponding POSIX timezone string (e.g., "EST5EDT,M3.2.0,M11.1.0"),
 *         or "GMT0" if the identifier is not found.
 */
const char *get_posix_from_id(const char *zone_identifier) {
    for (size_t i = 0; i < sizeof(Mapping) / sizeof(Mapping[0]); i++) {
        if (strcmp(Mapping[i].iana_timezone, zone_identifier) == 0) {
            ESP_LOGI(TAG,"matched");
            return Mapping[i].posix_timezone;
        }
    }
    return NULL;  // Return NULL if not found
}

/**
 * @brief Generates an HTML dropdown list containing timezone options.
 * 
 * This function constructs a `<select>` element by iterating through a predefined
 * list of timezones stored in `Mapping[]`. Each entry is formatted as an `<option>`
 * tag representing an available timezone. Memory allocation is handled dynamically
 * to prevent buffer overflow.
 *
 * @return char* A dynamically allocated string containing the HTML dropdown list.
 *         The caller is responsible for freeing this memory after use.
 */
char *generateSelectList(void) {
    size_t buffer_size = 1024;  // Initial size
    size_t position = 0;  // Tracks the current length
    char *html = malloc(buffer_size);
    
    if (!html) return NULL;

    // Append opening select tag
    if (!append_to_buffer(&html, &buffer_size, &position, "<select name=\"timezone\">\n")) {
        return NULL; // Handle allocation failure
    }

    // Loop through Mapping array and add options
    for (size_t i = 0; i < sizeof(Mapping) / sizeof(Mapping[0]); i++) {
        if (!append_to_buffer(&html, &buffer_size, &position, 
            "<option value=\"%s\">%s</option>\n", Mapping[i].iana_timezone, Mapping[i].iana_timezone)) {
            return NULL; // Handle allocation failure
        }
    }

    // Append closing select tag
    if (!append_to_buffer(&html, &buffer_size, &position, "</select>\n")) {
        return NULL; // Handle allocation failure
    }

    return html;
}
