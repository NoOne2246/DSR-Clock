#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "NVS_HANDLER";

/**
 * @brief Initialize the Non-Volatile Storage (NVS)
 * 
 * This function must be called before any NVS read/write operations.
 * It ensures that NVS is properly set up and ready for use.
 */
void init_nvs() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
    ESP_LOGI(TAG, "NVS initialized successfully.");
}

/**
 * @brief Retrieve a stored value from NVS
 * 
 * This function opens the NVS storage, retrieves a string value associated with the given key,
 * and returns it. If the key is not found, the provided default value is returned.
 * 
 * @param key The key associated with the stored value
 * @param default_value The default value to return if the key is missing
 * @return const char* The retrieved value or the default value
 */
const char *get_value_from_nvs(const char *key, const char *default_value) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(err));
        return default_value;  // Return default value if NVS cannot be opened
    }

    static char value[64];  // Persistent buffer
    size_t required_size = sizeof(value);
    err = nvs_get_str(my_handle, key, value, &required_size);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Retrieved %s: %s", key, value);
        nvs_close(my_handle);
        return value;
    } else {
        ESP_LOGW(TAG, "Key '%s' not found, using default: %s", key, default_value);
        nvs_close(my_handle);
        return default_value;
    }
}

/**
 * @brief Sets a string value in NVS.
 *
 * @param key The key to store the value under.
 * @param value The string value to store.
 * @return esp_err_t ESP_OK if successful, or an error code.
 */
esp_err_t set_value_in_nvs(const char *key, const char *value) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
        return err;
    }

    // Set the value
    err = nvs_set_str(nvs_handle, key, value);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Set key '%s' with value '%s'", key, value);
        nvs_commit(nvs_handle);
    } else {
        ESP_LOGE(TAG, "Failed to set key '%s': %s", key, esp_err_to_name(err));
    }

    nvs_close(nvs_handle);
    return err;
}

