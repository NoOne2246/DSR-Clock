#ifndef WEBSERVER_H
#define WEBSERVER_H

#include "esp_http_server.h"

#define EXAMPLE_HTTP_QUERY_KEY_MAX_LEN  (64)

void update_wifi(const char *input);
void update_timezone(const char *input);
void update_datetime(const char *input);

char *generate_html_page(void);

esp_err_t get_handler(httpd_req_t *req);
esp_err_t post_handler(httpd_req_t *req);
esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err);

httpd_handle_t start_webserver(void);
esp_err_t stop_webserver(httpd_handle_t server);
void disconnect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
void connect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

#endif // WEBSERVER_H