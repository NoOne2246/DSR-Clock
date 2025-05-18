#ifndef NVS_HANDLER_H
#define NVS_HANDLER_H

#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

/**
 * @brief Initialize the Non-Volatile Storage (NVS)
 * 
 * This function must be called before any NVS read/write operations.
 * It ensures that NVS is properly set up and ready for use.
 */
void init_nvs();

/**
 * @brief Sets a string value in NVS.
 *
 * @param key The key to store the value under.
 * @param value The string value to store.
 * @return esp_err_t ESP_OK if successful, or an error code.
 */
esp_err_t set_value_in_nvs(const char *key, const char *value);

/**
 * @brief Retrieves a string value from NVS.
 *
 * @param key The key to look up.
 * @param default_value Default value to return if key is not found.
 * @return const char* Retrieved value or default value.
 */
const char *get_value_from_nvs(const char *key, const char *default_value);


#endif // NVS_HANDLER_H