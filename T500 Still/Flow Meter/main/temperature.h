#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include <stdbool.h>
#include "esp_err.h"

// Temperature sensor configuration
#define TEMP_SENSOR_1_PIN GPIO_NUM_32  // ADC1_CH4 - Configurable
#define TEMP_SENSOR_2_PIN GPIO_NUM_33  // ADC1_CH5 - Configurable

// Temperature sensor types (placeholders for different sensor types)
typedef enum {
    TEMP_SENSOR_THERMISTOR,
    TEMP_SENSOR_DS18B20,
    TEMP_SENSOR_THERMOCOUPLE,
    TEMP_SENSOR_RTD
} temp_sensor_type_t;

// Temperature reading structure
typedef struct {
    float temperature_c;
    float temperature_f;
    bool valid;
    uint32_t last_update_ms;
} temp_reading_t;

// Global temperature readings (accessible by web dashboard)
extern temp_reading_t temp_sensor_1;
extern temp_reading_t temp_sensor_2;

// Function declarations
esp_err_t temperature_init(void);
esp_err_t temperature_read_sensor_1(void);
esp_err_t temperature_read_sensor_2(void);
void temperature_task(void *pvParameters);

// Utility functions
float celsius_to_fahrenheit(float celsius);
float fahrenheit_to_celsius(float fahrenheit);

// Sensor-specific functions (placeholders - implement based on your sensor choice)
esp_err_t read_thermistor(int adc_channel, float *temperature);
esp_err_t read_ds18b20(int gpio_pin, float *temperature);
esp_err_t read_thermocouple(int adc_channel, float *temperature);
esp_err_t read_rtd(int adc_channel, float *temperature);

#endif // TEMPERATURE_H
