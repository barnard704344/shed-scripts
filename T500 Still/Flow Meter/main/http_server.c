#include "http_server.h"
#include "web_dashboard.h"
#include "ota_update.h"
#include "esp_log.h"

static const char *TAG = "HTTP_SERVER";

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
        
        httpd_uri_t dashboard_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = dashboard_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &dashboard_uri);
        
        httpd_uri_t status_uri = {
            .uri       = "/status",
            .method    = HTTP_GET,
            .handler   = status_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &status_uri);
        
        httpd_uri_t ota_uri = {
            .uri       = "/ota",
            .method    = HTTP_POST,
            .handler   = trigger_ota_handler,
            .user_ctx  = NULL
        };
        httpd_register_uri_handler(server, &ota_uri);
        
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

void stop_webserver(httpd_handle_t server)
{
    if (server) {
        httpd_stop(server);
    }
}
