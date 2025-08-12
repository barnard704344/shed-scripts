#include "ota_update.h"
#include "config.h"
#include "esp_log.h"
#include "esp_https_ota.h"
#include "esp_http_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "OTA_UPDATE";

// Global OTA status
bool ota_in_progress = false;

void simple_ota_example_task(void *pvParameter)
{
    char *url = (char *)pvParameter;
    ESP_LOGI(TAG, "Starting OTA update from URL: %s", url);
    
    ota_in_progress = true;
    
    esp_http_client_config_t config = {
        .url = url,
        .cert_pem = NULL,
        .timeout_ms = 5000,
        .keep_alive_enable = true,
    };

    esp_https_ota_config_t ota_config = {
        .http_config = &config,
    };

    esp_err_t ret = esp_https_ota(&ota_config);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "OTA Succeed, Rebooting...");
        esp_restart();
    } else {
        ESP_LOGE(TAG, "OTA Failed");
    }
    
    ota_in_progress = false;
    free(url);
    vTaskDelete(NULL);
}

esp_err_t start_ota_update(const char* url)
{
    if (ota_in_progress) {
        ESP_LOGW(TAG, "OTA update already in progress");
        return ESP_ERR_INVALID_STATE;
    }
    
    char *url_copy = malloc(OTA_URL_SIZE);
    if (url_copy == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for OTA URL");
        return ESP_ERR_NO_MEM;
    }
    
    strncpy(url_copy, url, OTA_URL_SIZE);
    
    BaseType_t result = xTaskCreate(&simple_ota_example_task, "ota_task", 8192, url_copy, 5, NULL);
    if (result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create OTA task");
        free(url_copy);
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t trigger_ota_handler(httpd_req_t *req)
{
    const char* resp_str = "OTA Update Started";
    httpd_resp_send(req, resp_str, strlen(resp_str));
    
    // Start OTA task
    esp_err_t ret = start_ota_update(FIRMWARE_UPGRADE_URL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start OTA update");
    }
    
    return ESP_OK;
}
