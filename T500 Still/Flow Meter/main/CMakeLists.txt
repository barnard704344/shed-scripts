idf_component_register(SRCS "main.c" "web_dashboard.c" "ota_update.c" "http_server.c" "temperature.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_wifi nvs_flash esp_http_server esp_https_ota app_update esp_driver_gpio esp_driver_ledc esp_timer esp_adc)