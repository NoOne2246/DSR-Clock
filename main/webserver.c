#include "webserver.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "time_handler.h"
#include "wifi_manager.h"
#include "nvs_handler.h"
#include "util.h"
#include <regex.h>
#include <stdio.h>


static const char *TAG = "WebServer";

/** 
 * @brief Handles incoming POST requests. 
 * 
 * Receives and processes data sent via POST, responding accordingly.
 * 
 * @param req HTTP request structure.
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error.
 */
esp_err_t get_handler(httpd_req_t *req)
{
    esp_err_t error;
    ESP_LOGI(TAG, "home page request");
    const char *response = (const char *) req ->user_ctx;
    error = httpd_resp_send(req, response, strlen(response));
    
    if (error != ESP_OK)
    {
        ESP_LOGI(TAG, "Error %d while sending Response", error);
    }
    return error;
}

/**
 * @brief Parses and updates Wi-Fi credentials.
 * 
 * Extracts SSID and password from the input string, connects to the network,
 * and saves the credentials in NVS upon successful connection.
 * 
 * @param input Input string formatted as "ssid=<SSID>&password=<PASSWORD>"
 */
void update_wifi(const char *input){
    char ssid[32], password[64];
    sscanf(input, "ssid=%[^&]&password=%s", ssid, password);

    esp_err_t ret = wifi_connect(ssid, password);
    if (ret == ESP_OK){
        set_value_in_nvs("ssid", ssid);
        set_value_in_nvs("password", password);
    }
}

/**
 * @brief Updates the system timezone.
 * 
 * Extracts the timezone from the input string and applies the new timezone setting.
 * 
 * @param input Input string formatted as "timezone=<TIMEZONE>"
 */
void update_timezone(const char *input){
    char tz[32];
    sscanf(input, "timezone=%[^&]", tz);
    set_TZ(tz);
}

/**
 * @brief Manually updates the system datetime.
 * 
 * Extracts the datetime value from the input string and sets the system time.
 * 
 * @param input Input string formatted as "datetime=<DATETIME>"
 */
void update_datetime(const char *input){
    char dt[32];
    sscanf(input, "datetime=%[^&]", dt);
    obtain_time_manual(dt);  // Pass the extracted timezone
}

/**
 * @brief Handles generic POST requests.
 *
 * Receives and logs incoming POST data, then responds accordingly.
 * 
 * @param req HTTP request structure.
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error.
 */
esp_err_t post_handler(httpd_req_t *req)
{
    char buf[150];
    size_t recv_size = MIN(req->content_len, sizeof(buf));
    int ret = httpd_req_recv(req, buf, recv_size);
    if(ret<=0)
    { 
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    ESP_LOGI(TAG, "Received post data: %sEND", buf);
    //normalize_utf8(buf);
    normalize_line_endings(buf);
    trim_whitespace(buf);
    url_decode(buf);
    ESP_LOGI(TAG, "Decoded data: %s", buf);

    esp_err_t error;
    const char *response = (const char *) req ->user_ctx;
    error = httpd_resp_send(req, response, strlen(response));
    if (error != ESP_OK)
    {
        ESP_LOGI(TAG, "Error %d while sending Response", error);
    }
    vTaskDelay(pdMS_TO_TICKS(500)); 

    if (strstr(buf, "ssid") != NULL) {
        ESP_LOGI(TAG, "Updating WiFi...");
        update_wifi(buf);
    } else if (strstr(buf, "timezone") != NULL) {
        ESP_LOGI(TAG, "setting timezone");
        update_timezone(buf);
    } else if (strstr(buf, "datetime") != NULL) {
        ESP_LOGI(TAG, "setting datetime");
        update_datetime(buf);
    } else if (strstr(buf, "Rewind") != NULL) {
        ESP_LOGI(TAG, "Rewinding...");
        //handle_rewind();  // Implement this function as needed
    } else if (strstr(buf, "Forward") != NULL) {
        ESP_LOGI(TAG, "Moving forward...");
        //handle_forward();  // Implement this function as needed
    }else {
        ESP_LOGW(TAG, "Unknown parameter received");
    }
    
    return ESP_OK;
}

/**
 * @brief Generates a dynamically allocated HTML page.
 *
 * Constructs an HTML page in memory using a resizable buffer. The generated page 
 * provides an interface for configuring the Dragonsong Reprise Clock, including 
 * Wi-Fi settings, timezone selection, and time controls.
 * 
 * - Supports responsive design for improved usability across devices.
 * 
 * - Uses dynamic memory allocation with automatic resizing.
 * 
 * - Includes form elements for Wi-Fi Connection, timezone selection, date/time 
 *   adjustments, Rewind to save our boy and Fast forward for the future
 *
 * @return char* Pointer to the generated HTML string (must be freed by the caller), 
 *         or NULL if allocation fails.
 */
char *generate_html_page(void) {
    size_t buffer_size = 1024 * 4;  // Initial 4 KB allocation
    size_t position = 0;  // Tracks buffer usage
    char *html = malloc(buffer_size);

    if (!html) return NULL;

    // Append HTML structure using `append_to_buffer()`
    if (!append_to_buffer(&html, &buffer_size, &position, "<!DOCTYPE html><html><head><style>"))
        return NULL;

    append_to_buffer(&html, &buffer_size, &position, "* { box-sizing: border-box; }");
    append_to_buffer(&html, &buffer_size, &position, "input[type=submit] { background-color: #008CBA; color: white; padding: 12px 20px; border: none; border-radius: 4px; cursor: pointer; float: right; width: 80%%; }");
    append_to_buffer(&html, &buffer_size, &position, "input, select { width: 100%%; padding: 12px; border: 1px solid #ccc; border-radius: 4px; resize: vertical; }");
    append_to_buffer(&html, &buffer_size, &position, "label { padding: 12px 12px 12px 0; display: inline-block; }");
    append_to_buffer(&html, &buffer_size, &position, ".container { border-radius: 5px; padding: 20px; }");
    append_to_buffer(&html, &buffer_size, &position, ".col-25 { float: left; width: 25%%; margin-top: 6px; }");
    append_to_buffer(&html, &buffer_size, &position, ".col-50 { float: left; width: 50%%; margin-top: 6px; }");
    append_to_buffer(&html, &buffer_size, &position, ".row::after { content: \"\"; display: table; clear: both;}");
    append_to_buffer(&html, &buffer_size, &position, "@media screen and (max-width: 600px) { .col-25, .col-50, input[type=submit] { width: 100%%; margin-top: 10px; }}");
    append_to_buffer(&html, &buffer_size, &position, "</style></head><body><h1 style=\"text-align: center\">Dragonsong Reprise Clock</h1><div class=\"container\">");

    // Wi-Fi Form
    append_to_buffer(&html, &buffer_size, &position, "<form action=\"\" method=\"post\"><div class=\"row\"><div class=\"col-25\"><label>SSID:</label></div>");
    append_to_buffer(&html, &buffer_size, &position, "<div class=\"col-50\"><input type=\"text\" name=\"ssid\"></div></div>");
    append_to_buffer(&html, &buffer_size, &position, "<div class=\"row\"><div class=\"col-25\"><label>Password:</label></div>");
    append_to_buffer(&html, &buffer_size, &position, "<div class=\"col-50\"><input type=\"password\" name=\"password\"></div>");
    append_to_buffer(&html, &buffer_size, &position, "<div class=\"col-25\"><input type=\"submit\" value=\"Connect\"></div></div></form>");

    // Timezone Form
    append_to_buffer(&html, &buffer_size, &position, "<form action=\"\" method=\"post\"><div class=\"row\"><div class=\"col-25\"><label>Timezone</label></div><div class=\"col-50\">");
    char *timezone_select = generateSelectList();
    if (timezone_select) {
        append_to_buffer(&html, &buffer_size, &position, "%s", timezone_select);
        free(timezone_select);
    }
    append_to_buffer(&html, &buffer_size, &position, "</div><div class=\"col-25\"><input type=\"submit\" value=\"Set\"></div></div></form>");

    // Date & Time Form
    append_to_buffer(&html, &buffer_size, &position, "<form action=\"\" method=\"post\"><div class=\"row\"><div class=\"col-25\"><label>Date and Time</label></div>");
    append_to_buffer(&html, &buffer_size, &position, "<div class=\"col-50\"><input type=\"datetime-local\" name=\"datetime\"></div>");
    append_to_buffer(&html, &buffer_size, &position, "<div class=\"col-25\"><input type=\"submit\" value=\"Set\"></div></div></form>");

    // Rewind & Forward Buttons
    append_to_buffer(&html, &buffer_size, &position, "<div class=\"row\"><div class=\"col-50\"><form action=\"\" method=\"post\">");
    append_to_buffer(&html, &buffer_size, &position, "<input type=\"submit\" style=\"float:left\" name=\"Rewind\" value=\"Rewind\"/></form></div>");
    append_to_buffer(&html, &buffer_size, &position, "<div class=\"col-50\"><form action=\"\" method=\"post\">");
    append_to_buffer(&html, &buffer_size, &position, "<input type=\"submit\" style=\"float:right\" name=\"Forward\" value=\"Forward\"/></form></div></div>");
    append_to_buffer(&html, &buffer_size, &position, "</div></body></html>");

    return html;
}


/**
 * @brief Handles incoming GET requests.
 *
 * Sends a predefined response when the home page is requested.
 * 
 * @param req HTTP request structure.
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error.
 */
httpd_uri_t root_uri_get = {
    .uri       = "/",
    .method    = HTTP_GET,
    .handler   = get_handler,
    .user_ctx  = NULL
};

/**
 * @brief Handles generic POST requests.
 *
 * Receives and logs incoming POST data, then responds accordingly.
 * 
 * @param req HTTP request structure.
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error.
 */
httpd_uri_t root_uri_post = {
    .uri      = "/",
    .method   = HTTP_POST,
    .handler  = post_handler,
    .user_ctx  = NULL
};

/**
 * @brief Handles HTTP 404 errors.
 * 
 * Sends a 404 error message when the requested URI is not found.
 * 
 * @param req HTTP request structure.
 * @param err HTTP error code.
 * @return esp_err_t ESP_FAIL indicating failure.
 */
esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Some 404 error message");
    return ESP_FAIL;
}

/**
 * @brief Starts the HTTP server.
 * 
 * Initializes and registers request handlers, starting the server.
 * 
 * @return httpd_handle_t Handle of the started server, or NULL on failure.
 */
httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable = true;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        root_uri_get.user_ctx = generate_html_page();
        root_uri_post.user_ctx = generate_html_page();
        httpd_register_uri_handler(server, &root_uri_get);
        httpd_register_uri_handler(server, &root_uri_post);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

/**
 * @brief Stops the HTTP server.
 * 
 * Gracefully shuts down the web server.
 * 
 * @param server Handle of the running server.
 * @return esp_err_t ESP_OK on success, ESP_FAIL on error.
 */
esp_err_t stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    return httpd_stop(server);
}

/**
 * @brief Handles network disconnect events.
 * 
 * Stops the web server when a disconnect event occurs.
 * 
 * @param arg Pointer to the server handle.
 * @param event_base Event base identifier.
 * @param event_id Event ID.
 * @param event_data Additional event data.
 */
void disconnect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
        if (stop_webserver(*server) == ESP_OK) {
            *server = NULL;
        } else {
            ESP_LOGE(TAG, "Failed to stop http server");
        }
    }
}

/**
 * @brief Handles network connect events.
 * 
 * Starts the web server upon a successful network connection.
 * 
 * @param arg Pointer to the server handle.
 * @param event_base Event base identifier.
 * @param event_id Event ID.
 * @param event_data Additional event data.
 */
void connect_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}
