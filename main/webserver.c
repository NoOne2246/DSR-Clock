#include "webserver.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_heap_caps.h"

static const char *TAG = "WebServer";

/** Handler for POST requests **/
esp_err_t command_handler(httpd_req_t *req) {
    char buffer[128];
    int received = httpd_req_recv(req, buffer, sizeof(buffer) - 1);
    if (received <= 0) return ESP_FAIL;

    buffer[received] = '\0';  // Null-terminate input

    ESP_LOGI(TAG, "Received: %s", buffer);

    if (strcmp(buffer, "Rewind") == 0 ) {
        httpd_resp_send(req, "Rewinding to save my Boy", HTTPD_RESP_USE_STRLEN);
    }
    else if(strcmp(buffer, "Forward") == 0)
    {
        httpd_resp_send(req, "My Boy is saved", HTTPD_RESP_USE_STRLEN);
    } else {
        httpd_resp_send(req, buffer, HTTPD_RESP_USE_STRLEN);
    }

    return ESP_OK;
}

/** Start Web Server **/
void start_webserver() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;
    httpd_start(&server, &config);

    httpd_uri_t command_uri = {
        .uri = "/command",
        .method = HTTP_POST,
        .handler = command_handler,
    };
    httpd_register_uri_handler(server, &command_uri);

    ESP_LOGI(TAG, "Webserver started");
}
