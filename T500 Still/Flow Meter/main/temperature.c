#include "temperature.h"
#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include <math.h>

static const char *TAG = "TEMPERATURE";

// Global temperature readings with T500 location descriptions
temp_reading_t temp_sensor_1 = {
    .temperature_c = 0.0f,
    .temperature_f = 32.0f,
    .valid = false,
    .last_update_ms = 0,
    .location = "T500 Water Outlet (Point A)"
};

temp_reading_t temp_sensor_2 = {
    .temperature_c = 0.0f,
    .temperature_f = 32.0f,
    .valid = false,
    .last_update_ms = 0,
    .location = "T500 Cooling Water Inlet (Point B)"
};

// ADC handles (will be initialized based on sensor type)
static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_cali_handle_t adc1_cali_handle = NULL;

esp_err_t temperature_init(void)
{
    ESP_LOGI(TAG, "Initializing temperature monitoring");
    
    // Initialize ADC for analog temperature sensors
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
    };
    esp_err_t ret = adc_oneshot_new_unit(&init_config1, &adc1_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize ADC unit: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure ADC channels for temperature sensors
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12,  // 0-3.1V range
    };
    
    // Configure sensor 1 - T500 Water Outlet Temperature (Point A)
    ret = adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &config);  // GPIO34
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure T500 water outlet temp sensor (Point A): %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure sensor 2 - T500 Cooling Water Inlet (Point B)
    ret = adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_7, &config);  // GPIO35
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure T500 cooling water inlet sensor (Point B): %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Initialize temperature readings
    temp_sensor_1.valid = false;
    temp_sensor_1.temperature_c = 0.0f;
    temp_sensor_1.temperature_f = 32.0f;
    temp_sensor_1.last_update_ms = 0;
    
    temp_sensor_2.valid = false;
    temp_sensor_2.temperature_c = 0.0f;
    temp_sensor_2.temperature_f = 32.0f;
    temp_sensor_2.last_update_ms = 0;
    
    ESP_LOGI(TAG, "Temperature monitoring initialized");
    return ESP_OK;
}

esp_err_t temperature_read_sensor_1(void)
{
    if (!adc1_handle) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Read T500 Water Outlet Temperature (Point A)
    // This is the primary control sensor for T500 temperature-based flow control
    float temperature = 0.0f;
    esp_err_t ret = read_thermistor(ADC_CHANNEL_6, &temperature);  // GPIO34
    
    if (ret == ESP_OK) {
        temp_sensor_1.temperature_c = temperature;
        temp_sensor_1.temperature_f = celsius_to_fahrenheit(temperature);
        temp_sensor_1.valid = true;
        temp_sensor_1.last_update_ms = esp_timer_get_time() / 1000;
    } else {
        temp_sensor_1.valid = false;
    }
    
    return ret;
}

esp_err_t temperature_read_sensor_2(void)
{
    if (!adc1_handle) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // Read T500 Cooling Water Inlet Temperature (Point B)
    // Read T500 Cooling Water Inlet Temperature (Point B)
    // This is a reference sensor for monitoring inlet water temperature
    float temperature = 0.0f;
    esp_err_t ret = read_thermistor(ADC_CHANNEL_7, &temperature);  // GPIO35
    
    if (ret == ESP_OK) {
        temp_sensor_2.temperature_c = temperature;
        temp_sensor_2.temperature_f = celsius_to_fahrenheit(temperature);
        temp_sensor_2.valid = true;
        temp_sensor_2.last_update_ms = esp_timer_get_time() / 1000;
    } else {
        temp_sensor_2.valid = false;
    }
    
    return ret;
}

void temperature_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Temperature monitoring task started");
    
    while (1) {
        // Read both temperature sensors
        esp_err_t ret1 = temperature_read_sensor_1();
        esp_err_t ret2 = temperature_read_sensor_2();
        
        if (ret1 == ESP_OK) {
            ESP_LOGD(TAG, "Sensor 1: %.2f°C (%.2f°F)", 
                     temp_sensor_1.temperature_c, temp_sensor_1.temperature_f);
        } else {
            ESP_LOGW(TAG, "Failed to read sensor 1");
        }
        
        if (ret2 == ESP_OK) {
            ESP_LOGD(TAG, "Sensor 2: %.2f°C (%.2f°F)", 
                     temp_sensor_2.temperature_c, temp_sensor_2.temperature_f);
        } else {
            ESP_LOGW(TAG, "Failed to read sensor 2");
        }
        
        // Wait 2 seconds between readings
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

float celsius_to_fahrenheit(float celsius)
{
    return (celsius * 9.0f / 5.0f) + 32.0f;
}

float fahrenheit_to_celsius(float fahrenheit)
{
    return (fahrenheit - 32.0f) * 5.0f / 9.0f;
}

// Placeholder implementations for different sensor types
// Replace these with actual sensor-specific code based on your choice

esp_err_t read_thermistor(int adc_channel, float *temperature)
{
    if (!adc1_handle || !temperature) {
        return ESP_ERR_INVALID_ARG;
    }
    
    int adc_reading = 0;
    esp_err_t ret = adc_oneshot_read(adc1_handle, adc_channel, &adc_reading);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Placeholder thermistor calculation (Steinhart-Hart equation)
    // Replace with your specific thermistor parameters
    float voltage = (float)adc_reading / 4095.0f * 3.1f;  // Convert to voltage
    float resistance = 10000.0f * voltage / (3.1f - voltage);  // Assuming 10k pullup
    
    // Steinhart-Hart coefficients (example for 10k NTC thermistor)
    float A = 0.001129148f;
    float B = 0.000234125f;
    float C = 0.0000000876741f;
    
    float ln_r = logf(resistance);
    float temp_kelvin = 1.0f / (A + B * ln_r + C * ln_r * ln_r * ln_r);
    *temperature = temp_kelvin - 273.15f;  // Convert to Celsius
    
    return ESP_OK;
}

esp_err_t read_ds18b20(int gpio_pin, float *temperature)
{
    // Placeholder for DS18B20 1-Wire temperature sensor
    // You'll need to implement 1-Wire protocol or use a library
    ESP_LOGW(TAG, "DS18B20 reading not implemented yet");
    *temperature = 25.0f;  // Placeholder value
    return ESP_OK;
}

esp_err_t read_thermocouple(int adc_channel, float *temperature)
{
    // Placeholder for thermocouple reading
    // Requires cold junction compensation and linearization
    ESP_LOGW(TAG, "Thermocouple reading not implemented yet");
    *temperature = 25.0f;  // Placeholder value
    return ESP_OK;
}

esp_err_t read_rtd(int adc_channel, float *temperature)
{
    // Placeholder for RTD (Resistance Temperature Detector) reading
    // Requires precision current source and linearization
    ESP_LOGW(TAG, "RTD reading not implemented yet");
    *temperature = 25.0f;  // Placeholder value
    return ESP_OK;
}
