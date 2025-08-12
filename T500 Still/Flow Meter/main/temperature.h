#ifndef TEMPERATURE_H
#define TEMPERATURE_H

#include <stdbool.h>
#include "esp_err.h"

// T500 Temperature sensor configuration based on T500 diagram
#define TEMP_SENSOR_1_PIN GPIO_NUM_34  // ADC1_CH6 - T500 Water Outlet Temperature (Point A)
#define TEMP_SENSOR_2_PIN GPIO_NUM_35  // ADC1_CH7 - T500 Cooling Water Inlet (Point B)

// T500 Sensor Labels (matching diagram)
#define T500_WATER_OUTLET_TEMP_SENSOR    1  // Sensor A - Water outlet temperature sensor
#define T500_COOLING_WATER_INLET_SENSOR  2  // Sensor B - Cooling water inlet temperature

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
    const char* location;  // T500 location description
} temp_reading_t;

// Global temperature readings (accessible by web dashboard)
// Sensor 1: T500 Water Outlet Temperature (Point A) - Primary control sensor
extern temp_reading_t temp_sensor_1;  
// Sensor 2: T500 Cooling Water Inlet (Point B) - Reference sensor
extern temp_reading_t temp_sensor_2;

// Function declarations
esp_err_t temperature_init(void);
esp_err_t temperature_read_sensor_1(void);  // Read T500 water outlet temp (Point A)
esp_err_t temperature_read_sensor_2(void);  // Read T500 cooling water inlet (Point B)
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
