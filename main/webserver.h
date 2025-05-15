#ifndef WEBSERVER_H
#define WEBSERVER_H

#include "esp_http_server.h"

/**
 * @brief Starts the web server.
 * 
 * Initializes the HTTP server and sets up request handling.
 */
void start_webserver(void);

/**
 * @brief Handles incoming requests.
 * 
 * Listens for POST requests and processes commands ("Rewind" or "Forward").
 * 
 * @param req Pointer to the HTTP request.
 * @return esp_err_t Returns ESP_OK on success, ESP_FAIL on error.
 */
esp_err_t command_handler(httpd_req_t *req);

#endif // WEBSERVER_H