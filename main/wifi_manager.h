#ifndef WIFI_MANAGER_H
#define WIFI_MANAGER_H

#include "esp_err.h"

#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_wifi.h"

#define WIFI_SSID "DSR_Clock"
#define WIFI_PASS "Haurchefant"

esp_err_t wifi_init(void);
esp_err_t wifi_connect(char *wifi_ssid, char *wifi_password);
esp_err_t wifi_disconnect(void);
esp_err_t deinit_wifi(void);
void wifi_ap_mode(void);
void set_device_name(esp_netif_t *netif);

#endif // WIFI_MANAGER_H