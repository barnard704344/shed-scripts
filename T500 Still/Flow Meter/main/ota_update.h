#ifndef OTA_UPDATE_H
#define OTA_UPDATE_H

#include "esp_http_server.h"
#include "esp_err.h"

// OTA Configuration
#define OTA_URL_SIZE 256

// Global OTA status
extern bool ota_in_progress;

// Function declarations
esp_err_t trigger_ota_handler(httpd_req_t *req);
void simple_ota_example_task(void *pvParameter);
esp_err_t start_ota_update(const char* url);

#endif // OTA_UPDATE_H
