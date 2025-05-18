#include "wifi_manager.h"

#include "esp_log.h"
#include <inttypes.h>
#include <string.h>
#include "time_handler.h"
#include "nvs_handler.h"

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static const int WIFI_RETRY_ATTEMPT = 3;
static int wifi_retry_count = 0;

static esp_netif_t *netif_ap = NULL;
static esp_netif_t *netif_sta = NULL;
static EventGroupHandle_t wifi_event_group = NULL;

static esp_event_handler_instance_t ip_event_handler;
static esp_event_handler_instance_t wifi_event_handler;

static const char *TAG = "Wi-Fi_Manager";

static void ip_event_cb(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "Handling IP event, event code 0x%" PRIx32, event_id);
    switch (event_id)
    {
    case (IP_EVENT_STA_GOT_IP):
        ip_event_got_ip_t *event_ip = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event_ip->ip_info.ip));
        wifi_retry_count = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    case (IP_EVENT_STA_LOST_IP):
        ESP_LOGI(TAG, "Lost IP");
        break;
    case (IP_EVENT_GOT_IP6):
        ip_event_got_ip6_t *event_ip6 = (ip_event_got_ip6_t *)event_data;
        ESP_LOGI(TAG, "Got IPv6: " IPV6STR, IPV62STR(event_ip6->ip6_info.ip));
        wifi_retry_count = 0;
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    default:
        ESP_LOGI(TAG, "IP event not handled");
        break;
    }
}

static void wifi_event_cb(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    ESP_LOGI(TAG, "Handling Wi-Fi event, event code 0x%" PRIx32, event_id);

    switch (event_id)
    {
    case (WIFI_EVENT_WIFI_READY):
        ESP_LOGI(TAG, "Wi-Fi ready");
        break;
    case (WIFI_EVENT_SCAN_DONE):
        ESP_LOGI(TAG, "Wi-Fi scan done");
        break;
    case (WIFI_EVENT_STA_START):
        ESP_LOGI(TAG, "Wi-Fi started, connecting to AP...");
        esp_wifi_connect();
        break;
    case (WIFI_EVENT_STA_STOP):
        ESP_LOGI(TAG, "Wi-Fi stopped");
        break;
    case (WIFI_EVENT_STA_CONNECTED):
        ESP_LOGI(TAG, "Wi-Fi connected");
        wifi_retry_count = 0;  // Reset retry count
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        break;
    case (WIFI_EVENT_STA_DISCONNECTED):
        ESP_LOGI(TAG, "Wi-Fi disconnected");
        if (wifi_retry_count < WIFI_RETRY_ATTEMPT) {
            ESP_LOGI(TAG, "Retrying to connect to Wi-Fi network...");
            esp_wifi_connect();
            wifi_retry_count++;
        } else {
            ESP_LOGI(TAG, "Failed to connect to Wi-Fi network");
            xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
        }
        break;
    case (WIFI_EVENT_AP_STACONNECTED):
        ESP_LOGI(TAG, "Wi-Fi AP mode started");
        break;
    case (WIFI_EVENT_STA_AUTHMODE_CHANGE):
        ESP_LOGI(TAG, "Wi-Fi authmode changed");
        break;
    default:
        ESP_LOGI(TAG, "Unhandled Wi-Fi event: base=%s, id=0x%" PRIx32, event_base, event_id);
        break;
    }
}

void wifi_ap_mode(void)
{
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .max_connection = 4,
            .password = WIFI_PASS,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s", WIFI_SSID, WIFI_PASS);
}

void set_device_name(esp_netif_t *netif) {
    if (netif == NULL) {
        ESP_LOGE("WiFi", "Network interface is NULL, cannot set hostname");
        return;
    }

    esp_netif_set_hostname(netif, "DSR Clock");  // Change hostname
    ESP_LOGI("WiFi", "Device name set to DSR Clock");
}


esp_err_t wifi_init(void)
{
    esp_err_t ret = esp_netif_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize TCP/IP network stack");
        return ret;
    }

    wifi_event_group = xEventGroupCreate();
    if (wifi_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
        return ESP_FAIL;
    }
    
    ret = esp_event_loop_create_default();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create default event loop");
        return ret;
    }

    netif_ap = esp_netif_create_default_wifi_ap();
    assert(netif_ap);
    netif_sta = esp_netif_create_default_wifi_sta();
    assert(netif_sta);
    set_device_name(netif_sta);

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_cb, NULL, &wifi_event_handler));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, ESP_EVENT_ANY_ID, &ip_event_cb, NULL, &ip_event_handler));

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA) );
    ESP_ERROR_CHECK(esp_wifi_start() );
    wifi_ap_mode();
    return wifi_connect(NULL, NULL);
    return ret;
}

esp_err_t wifi_connect(char* wifi_ssid, char* wifi_password)
{
    wifi_config_t wifi_config = {
        .sta = {
            // this sets the weakest authmode accepted in fast scan mode (default)
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };

    const char *ssid = (wifi_ssid == NULL || strlen(wifi_ssid) == 0) ? get_value_from_nvs("ssid", "") : wifi_ssid;
    if (strlen(ssid) == 0) {
        ESP_LOGE(TAG, "No SSID found in input or NVS");
        return ESP_FAIL;
    }
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));

    const char *password = (wifi_password == NULL || strlen(wifi_password) == 0) ? get_value_from_nvs("password", "") : wifi_password;
    if (strlen(password) == 0) {
        ESP_LOGE(TAG, "No password found in input or NVS");
        return ESP_FAIL;
    }
    strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));

    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE)); // default is WIFI_PS_MIN_MODEM
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM)); // default is WIFI_STORAGE_FLASH

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));

    ESP_LOGI(TAG,"Attempting to connect to %s using %s", wifi_config.sta.ssid, wifi_config.sta.password);

    // wifi_config_t current_config;
    // ESP_ERROR_CHECK(esp_wifi_get_config(WIFI_IF_STA, &current_config));

    // ESP_LOGI(TAG, "Current SSID: %s", current_config.sta.ssid);
    // ESP_LOGI(TAG, "Current Password: %s", current_config.sta.password);

    ESP_ERROR_CHECK(esp_wifi_start());

    // wifi_ap_record_t ap_info;
    // if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
    //     ESP_LOGI(TAG, "Connected to: %s", ap_info.ssid);
    //     esp_wifi_set_auto_connect(false);
    //     ESP_ERROR_CHECK(esp_wifi_disconnect());
    //     esp_wifi_set_auto_connect(true);
    //     ESP_LOGI(TAG, "Disconnected from Wi-Fi.");
    // } else {
    //     ESP_LOGW(TAG, "Not connected to any Wi-Fi network.");
    // }
    ESP_ERROR_CHECK(esp_wifi_connect());  // Connect to new AP

    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "Connected to Wi-Fi network: %s", wifi_config.sta.ssid);
        //ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        create_set_time_task();
        return ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGE(TAG, "Failed to connect to Wi-Fi network: %s", wifi_config.sta.ssid);
        return ESP_FAIL;
    }

    ESP_LOGE(TAG, "Unexpected Wi-Fi error");
    return ESP_FAIL;
}

esp_err_t wifi_disconnect(void)
{
    if (wifi_event_group) {
        vEventGroupDelete(wifi_event_group);
    }

    return esp_wifi_disconnect();
}

esp_err_t deinit_wifi(void)
{
    esp_err_t ret = esp_wifi_stop();
    if (ret == ESP_ERR_WIFI_NOT_INIT) {
        ESP_LOGE(TAG, "Wi-Fi stack not initialized");
        return ret;
    }

    ESP_ERROR_CHECK(esp_wifi_deinit());
    ESP_ERROR_CHECK(esp_wifi_clear_default_wifi_driver_and_handlers(netif_ap));
    esp_netif_destroy(netif_ap);
    ESP_ERROR_CHECK(esp_wifi_clear_default_wifi_driver_and_handlers(netif_sta));
    esp_netif_destroy(netif_sta);

    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, ESP_EVENT_ANY_ID, ip_event_handler));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler));

    return ESP_OK;
}