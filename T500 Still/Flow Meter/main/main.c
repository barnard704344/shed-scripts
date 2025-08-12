#include <stdio.h>
#include <string.h>
#include <math.h>
#include <ctype.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_https_ota.h"
#include "esp_ota_ops.h"
#include "esp_app_format.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "config.h"
#include "web_dashboard.h"
#include "ota_update.h"
#include "http_server.h"
#include "temperature.h"

// ================== Hardware wiring ==================
//
// Flow sensors (open-collector):
//  - Power sensors from +5V and GND.
//  - Add 10k pull-up from signal to 3.3V; feed that node to ESP32 GPIO.
//  - Suggested pins below use input-only GPIOs 34, 35.
//  - Keep all grounds common.
//
// Servo:
//  - Power from separate 5–6V supply able to provide surge current.
//  - Common GND with ESP32.
//  - Signal on a PWM-capable pin (GPIO18 by default).
//
// ================== Pin map ==========================
#define FLOW_IN_GPIO    GPIO_NUM_34   // before valve/coil
#define FLOW_OUT_GPIO   GPIO_NUM_35   // after valve/coil
#define SERVO_GPIO      GPIO_NUM_18   // PWM signal to servo

// WiFi Configuration (from config.h)
#define WIFI_MAXIMUM_RETRY  5

// WiFi event group bits
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// OTA Configuration
#define OTA_URL_SIZE 256

// ================== Flow calibration =================
// Jaycar ZD1202 Reed Switch Flowmeter (0.6-8.0 L/min)
// From datasheet calibration table:
// Flow Range: 2.0-8.0 L/min -> 0.0041 liter per pulse
// Calculation: 1/0.0041 = 243.9 pulses per liter
// At 1 L/min: 243.9 pulses/min = 243.9/60 = 4.065 Hz per L/min
// 
// For lower flow rates, use different calibration:
// 1.5-2.0 L/min -> 0.0040 L/pulse -> 4.167 Hz per L/min
// 1.0-1.5 L/min -> 0.0038 L/pulse -> 4.386 Hz per L/min
// 0.8-1.0 L/min -> 0.0036 L/pulse -> 4.630 Hz per L/min
// 0.6-0.8 L/min -> 0.0033 L/pulse -> 5.051 Hz per L/min
//
// Using mid-range value for 2.0-8.0 L/min (most common range):
static float HZ_PER_LPM_IN  = 4.065f;  // ZD1202: 243.9 pulses/L (2.0-8.0 L/min range)
static float HZ_PER_LPM_OUT = 4.065f;  // ZD1202: 243.9 pulses/L (2.0-8.0 L/min range)

// ================== Control mode =====================
// Control modes are defined in web_dashboard.h to avoid conflicts
volatile control_mode_t CONTROL_MODE = MODE_SETPOINT; // start in Setpoint mode

// Legacy compatibility - remove the #define to avoid conflicts

// ================== PID params =======================
static float KP = 0.80f;
static float KI = 0.05f;
static float KD = 0.02f;

static const float INTEGRAL_MIN = -50.0f;
static const float INTEGRAL_MAX =  50.0f;

// Servo limits and dynamics
static const int   SERVO_MIN_DEG      = 10;
static const int   SERVO_MAX_DEG      = 170;
static const float SERVO_MAX_STEP_DEG = 5.0f; // per 1s update

// Servo pulse widths (us)
static const int SERVO_MIN_PW_US = 1000;
static const int SERVO_MAX_PW_US = 2000;

// PID state
static float integralTerm = 0.0f;
static float lastError    = 0.0f;

// Setpoint for OUT line (L/min) in Setpoint mode
// T500 Cooling Water Specifications:
// - 240V units: 0.400-0.600 L/min (400-600 mL/min)
// - 110V units: 0.104 L/min (103.5 mL/min ≈ 3.5 US fl oz/min)
float SETPOINT_LPM = 0.6f;  // Default to T500 240V maximum for safety

// Auto mode parameters for pressure compensation
float AUTO_TARGET_RATIO = 0.85f;  // Target output/input ratio (85% efficiency)
float MIN_INPUT_FLOW = 0.1f;      // Minimum input flow to maintain control (T500 compatible)
float MAX_INPUT_FLOW = 8.0f;      // Maximum expected input flow

// T500 Temperature-Based Flow Control
bool T500_TEMP_CONTROL_ENABLED = false;  // Enable automatic temperature-based flow adjustment
float T500_TEMP_THRESHOLD_HIGH = 50.0f;  // 50°C (122°F) - start water flow control
float T500_TEMP_THRESHOLD_LOW = 45.0f;   // 45°C (113°F) - reduce water flow (hysteresis)
float T500_TEMP_MAX_FLOW = 0.6f;         // Maximum flow at high temperatures
float T500_TEMP_MIN_FLOW = 0.1f;         // Minimum flow to maintain circulation
float T500_TEMP_RAMP_RATE = 0.05f;       // L/min per °C temperature change

// Sampling
static const int SAMPLE_PERIOD_MS = 1000;

// Servo state
float servoAngleDeg = 90.0f;
bool  pidEnabled    = true;

// Counters (ISR)
static volatile uint32_t pulseCountIn  = 0;
static volatile uint32_t pulseCountOut = 0;

// Sampled values (updated each second)
float lpm_in  = 0.0f;
float lpm_out = 0.0f;

// Locks
static portMUX_TYPE spinlock_in  = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE spinlock_out = portMUX_INITIALIZER_UNLOCKED;

// WiFi and OTA
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static httpd_handle_t server = NULL;

static const char *TAG = "T500-FLOW";

// ================== Utils ============================
static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : (v > hi ? hi : v);
}

static inline int angle_to_pulse_us(float angle_deg) {
    angle_deg = clampf(angle_deg, SERVO_MIN_DEG, SERVO_MAX_DEG);
    float t = (angle_deg - SERVO_MIN_DEG) / (float)(SERVO_MAX_DEG - SERVO_MIN_DEG);
    int us = SERVO_MIN_PW_US + (int)roundf(t * (SERVO_MAX_PW_US - SERVO_MIN_PW_US));
    return us;
}

// ================== WiFi Event Handler ===============
static void event_handler(void* arg, esp_event_base_t event_base,
                         int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

// ================== WiFi Initialization ==============
static void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s", WIFI_SSID, WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s", WIFI_SSID, WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }
}

// ================== OTA Update Functions =============

// ================== GPIO ISRs ========================
static void IRAM_ATTR isr_in(void *arg) {
    (void)arg;
    portENTER_CRITICAL_ISR(&spinlock_in);
    pulseCountIn++;
    portEXIT_CRITICAL_ISR(&spinlock_in);
}
static void IRAM_ATTR isr_out(void *arg) {
    (void)arg;
    portENTER_CRITICAL_ISR(&spinlock_out);
    pulseCountOut++;
    portEXIT_CRITICAL_ISR(&spinlock_out);
}

// ================== LEDC (servo) =====================
static ledc_channel_config_t ledc_ch;
static ledc_timer_config_t   ledc_tm;

static void servo_init(void) {
    // LEDC timer: 50 Hz, high resolution
    ledc_tm.speed_mode       = LEDC_LOW_SPEED_MODE;
    ledc_tm.timer_num        = LEDC_TIMER_0;
    ledc_tm.duty_resolution  = LEDC_TIMER_16_BIT;
    ledc_tm.freq_hz          = 50; // 50Hz for hobby servo
    ledc_tm.clk_cfg          = LEDC_AUTO_CLK;
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_tm));

    // Channel
    ledc_ch.channel    = LEDC_CHANNEL_0;
    ledc_ch.duty       = 0;
    ledc_ch.gpio_num   = SERVO_GPIO;
    ledc_ch.speed_mode = LEDC_LOW_SPEED_MODE;
    ledc_ch.hpoint     = 0;
    ledc_ch.timer_sel  = LEDC_TIMER_0;
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_ch));

    // Move to initial angle
    int us = angle_to_pulse_us(servoAngleDeg);
    // duty = us / period * 2^res
    const int period_us = 20000; // 20ms @ 50Hz
    uint32_t max_duty = (1u << ledc_tm.duty_resolution) - 1u;
    uint32_t duty = (uint32_t)((((uint64_t)us) * max_duty) / period_us);
    ESP_ERROR_CHECK(ledc_set_duty(ledc_ch.speed_mode, ledc_ch.channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(ledc_ch.speed_mode, ledc_ch.channel));
}

static void servo_write_angle(float requested_deg) {
    // slew limit
    float delta = requested_deg - servoAngleDeg;
    if (delta >  SERVO_MAX_STEP_DEG) delta =  SERVO_MAX_STEP_DEG;
    if (delta < -SERVO_MAX_STEP_DEG) delta = -SERVO_MAX_STEP_DEG;
    servoAngleDeg = clampf(servoAngleDeg + delta, SERVO_MIN_DEG, SERVO_MAX_DEG);

    int us = angle_to_pulse_us(servoAngleDeg);
    const int period_us = 20000;
    uint32_t max_duty = (1u << ledc_tm.duty_resolution) - 1u;
    uint32_t duty = (uint32_t)((((uint64_t)us) * max_duty) / period_us);
    ESP_ERROR_CHECK(ledc_set_duty(ledc_ch.speed_mode, ledc_ch.channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(ledc_ch.speed_mode, ledc_ch.channel));
}

// ================== PID ==============================
static float pid_step(float error) {
    // Integral with clamp
    integralTerm += error;
    if (integralTerm > INTEGRAL_MAX) integralTerm = INTEGRAL_MAX;
    if (integralTerm < INTEGRAL_MIN) integralTerm = INTEGRAL_MIN;

    float deriv = error - lastError;
    lastError = error;

    float out = (KP * error) + (KI * integralTerm) + (KD * deriv);
    return out; // degrees (to be applied through slew + clamp)
}

// ================== Sample timer =====================
static esp_timer_handle_t sample_timer;

static void sample_timer_cb(void *arg) {
    (void)arg;

    // atomically snapshot counts and clear for next window
    uint32_t c_in, c_out;
    portENTER_CRITICAL(&spinlock_in);
    c_in = pulseCountIn;
    pulseCountIn = 0;
    portEXIT_CRITICAL(&spinlock_in);

    portENTER_CRITICAL(&spinlock_out);
    c_out = pulseCountOut;
    pulseCountOut = 0;
    portEXIT_CRITICAL(&spinlock_out);

    // counts per 1s == Hz
    float hz_in  = (float)c_in;
    float hz_out = (float)c_out;

    lpm_in  = (HZ_PER_LPM_IN  > 0.0f) ? (hz_in  / HZ_PER_LPM_IN ) : 0.0f;
    lpm_out = (HZ_PER_LPM_OUT > 0.0f) ? (hz_out / HZ_PER_LPM_OUT) : 0.0f;

    // Compute error based on control mode
    float error = 0.0f;
    float effective_setpoint = SETPOINT_LPM;
    
    switch (CONTROL_MODE) {
        case MODE_SETPOINT:
            error = SETPOINT_LPM - lpm_out;
            break;
            
        case MODE_EQUALIZE:
            error = lpm_out - lpm_in;
            break;
            
        case MODE_AUTO:
            // Pressure-compensated mode: adjust target based on input flow
            if (lpm_in >= MIN_INPUT_FLOW) {
                effective_setpoint = lpm_in * AUTO_TARGET_RATIO;
                // Clamp to reasonable bounds
                if (effective_setpoint > SETPOINT_LPM) {
                    effective_setpoint = SETPOINT_LPM;  // Don't exceed user setpoint
                }
                error = effective_setpoint - lpm_out;
            } else {
                // Insufficient input flow - fall back to setpoint mode
                error = SETPOINT_LPM - lpm_out;
            }
            break;
            
        case MODE_T500_TEMP:
            // T500 temperature-based flow control
            if (temp_sensor_1.valid) {
                float temp_c = temp_sensor_1.temperature_c;
                
                if (temp_c >= T500_TEMP_THRESHOLD_HIGH) {
                    // Above 50°C - calculate flow based on temperature
                    float temp_factor = (temp_c - T500_TEMP_THRESHOLD_HIGH) * T500_TEMP_RAMP_RATE;
                    effective_setpoint = T500_TEMP_MIN_FLOW + temp_factor;
                    
                    // Clamp to reasonable bounds
                    if (effective_setpoint > T500_TEMP_MAX_FLOW) {
                        effective_setpoint = T500_TEMP_MAX_FLOW;
                    }
                } else if (temp_c <= T500_TEMP_THRESHOLD_LOW) {
                    // Below 45°C - minimum flow for circulation
                    effective_setpoint = T500_TEMP_MIN_FLOW;
                } else {
                    // In hysteresis zone - maintain current setpoint
                    effective_setpoint = SETPOINT_LPM;
                }
                
                error = effective_setpoint - lpm_out;
            } else {
                // No valid temperature reading - fall back to setpoint mode
                error = SETPOINT_LPM - lpm_out;
            }
            break;
    }

    // Drive servo
    if (pidEnabled) {
        float delta_deg = pid_step(error);
        float req = servoAngleDeg + delta_deg;
        servo_write_angle(req);
    }

    // Telemetry
    const char* mode_str = (CONTROL_MODE == MODE_SETPOINT) ? "SETPOINT" : 
                          (CONTROL_MODE == MODE_EQUALIZE) ? "EQUALIZE" : 
                          (CONTROL_MODE == MODE_AUTO) ? "AUTO" : "T500_TEMP";
                          
    ESP_LOGI(TAG,
        "IN: %.3f L/min  OUT: %.3f L/min  MODE:%s  SP:%.3f  ERR:%.3f  ANG:%.1f  PID(Kp=%.3f,Ki=%.3f,Kd=%.3f) I=%.2f",
        lpm_in, lpm_out, mode_str,
        (CONTROL_MODE == MODE_AUTO) ? effective_setpoint : SETPOINT_LPM, 
        error, servoAngleDeg, KP, KI, KD, integralTerm
    );
}

// ================== CLI Task =========================
// Simple line-based CLI on UART0 (idf.py monitor)
// Commands:
//   m0            -> Setpoint mode
//   m1            -> Equalize mode
//   s<value>      -> Setpoint L/min, e.g. s3.2
//   p<kp>         -> KP
//   i<ki>         -> KI
//   d<kd>         -> KD
//   a<deg>        -> Manual angle, disables PID
//   r             -> Re-enable PID
//   h             -> Help
static void print_help(void) {
    printf("\nCommands:\n");
    printf("  m0           -> Setpoint mode (error = setpoint - OUT)\n");
    printf("  m1           -> Equalize mode (error = OUT - IN)\n");
    printf("  m2           -> Auto mode (pressure-compensated setpoint)\n");
    printf("  m3           -> T500 temperature mode (temp-based flow control)\n");
    printf("  s<value>     -> Setpoint L/min (e.g., s0.6)\n");
    printf("  ratio<value> -> Auto mode target ratio (e.g., ratio0.85)\n");
    printf("  t500_240v    -> Set T500 240V preset (0.6 L/min)\n");
    printf("  t500_110v    -> Set T500 110V preset (0.104 L/min)\n");
    printf("  t500_temp    -> Enable T500 temperature control mode\n");
    printf("  temp_high<T> -> Set high temp threshold (°C, e.g., temp_high50)\n");
    printf("  temp_low<T>  -> Set low temp threshold (°C, e.g., temp_low45)\n");
    printf("  p<kp>        -> Set KP\n");
    printf("  i<ki>        -> Set KI\n");
    printf("  d<kd>        -> Set KD\n");
    printf("  a<deg>       -> Manual servo angle (disables PID)\n");
    printf("  r            -> Re-enable PID\n");
    printf("  cin<value>   -> Calibrate input sensor (Hz per L/min)\n");
    printf("  cout<value>  -> Calibrate output sensor (Hz per L/min)\n");
    printf("  status       -> Show system status\n");
    printf("  ota          -> Trigger OTA update\n");
    printf("  wifi         -> Show WiFi status\n");
    printf("  h            -> Help\n\n");
}

static void cli_task(void *arg) {
    (void)arg;
    print_help();
    char line[64];

    while (true) {
        // Read a line from stdin (idf.py monitor)
        if (fgets(line, sizeof(line), stdin) == NULL) {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        // Trim
        size_t n = strlen(line);
        while (n && (line[n-1] == '\r' || line[n-1] == '\n' || line[n-1] == ' ' || line[n-1] == '\t')) {
            line[--n] = 0;
        }
        if (n == 0) continue;

        char c = (char)tolower((int)line[0]);
        const char *v = &line[1];
        while (*v == ' ' || *v == '\t') v++;

        switch (c) {
            case 'm':
                if (strcmp(v, "0") == 0) {
                    CONTROL_MODE = MODE_SETPOINT;
                    integralTerm = 0.0f; lastError = 0.0f;
                    printf("Mode = SETPOINT\n");
                } else if (strcmp(v, "1") == 0) {
                    CONTROL_MODE = MODE_EQUALIZE;
                    integralTerm = 0.0f; lastError = 0.0f;
                    printf("Mode = EQUALIZE\n");
                } else if (strcmp(v, "2") == 0) {
                    CONTROL_MODE = MODE_AUTO;
                    integralTerm = 0.0f; lastError = 0.0f;
                    printf("Mode = AUTO (pressure-compensated)\n");
                    printf("Target ratio: %.2f (%.0f%% efficiency)\n", AUTO_TARGET_RATIO, AUTO_TARGET_RATIO * 100);
                } else if (strcmp(v, "3") == 0) {
                    CONTROL_MODE = MODE_T500_TEMP;
                    integralTerm = 0.0f; lastError = 0.0f;
                    printf("Mode = T500_TEMP (temperature-based flow control)\n");
                    printf("High threshold: %.1f°C, Low threshold: %.1f°C\n", 
                           T500_TEMP_THRESHOLD_HIGH, T500_TEMP_THRESHOLD_LOW);
                    printf("Flow range: %.3f-%.3f L/min\n", T500_TEMP_MIN_FLOW, T500_TEMP_MAX_FLOW);
                } else {
                    printf("Usage: m0 (setpoint), m1 (equalize), m2 (auto), or m3 (T500 temp)\n");
                }
                break;

            case 'p': {
                float val = strtof(v, NULL);
                if (isfinite(val)) { KP = val; printf("KP=%.3f\n", KP); }
            } break;

            case 'i': {
                float val = strtof(v, NULL);
                if (isfinite(val)) { KI = val; printf("KI=%.3f\n", KI); }
            } break;

            case 'd': {
                float val = strtof(v, NULL);
                if (isfinite(val)) { KD = val; printf("KD=%.3f\n", KD); }
            } break;

            case 'a': {
                float deg = strtof(v, NULL);
                if (isfinite(deg)) {
                    pidEnabled = false;
                    servo_write_angle(clampf(deg, SERVO_MIN_DEG, SERVO_MAX_DEG));
                    printf("Manual angle %.1f°, PID disabled\n", servoAngleDeg);
                } else {
                    printf("Bad angle\n");
                }
            } break;

            case 'r':
                if (strcmp(line, "ratio") == 0 || strncmp(line, "ratio", 5) == 0) {
                    if (strlen(v) > 0) {
                        float ratio = strtof(v, NULL);
                        if (ratio > 0.0f && ratio <= 1.0f && isfinite(ratio)) {
                            AUTO_TARGET_RATIO = ratio;
                            printf("Auto mode target ratio set to %.3f (%.1f%% efficiency)\n", 
                                   AUTO_TARGET_RATIO, AUTO_TARGET_RATIO * 100);
                        } else {
                            printf("Invalid ratio. Must be between 0.0 and 1.0\n");
                        }
                    } else {
                        printf("Current ratio: %.3f (%.1f%%)\n", AUTO_TARGET_RATIO, AUTO_TARGET_RATIO * 100);
                    }
                } else {
                    // Original 'r' command - re-enable PID
                    pidEnabled   = true;
                    integralTerm = 0.0f;
                    lastError    = 0.0f;
                    printf("PID re-enabled\n");
                }
                break;

            case 'h':
                print_help();
                break;

            case 'o': // ota
                if (strcmp(line, "ota") == 0) {
                    printf("Starting OTA update...\n");
                    start_ota_update(FIRMWARE_UPGRADE_URL);
                }
                break;

            case 's':
                if (strcmp(line, "status") == 0) {
                    printf("\n=== System Status ===\n");
                    printf("Firmware: %s\n", esp_app_get_description()->version);
                    printf("Free heap: %" PRIu32 " bytes\n", esp_get_free_heap_size());
                    printf("Uptime: %lld seconds\n", esp_timer_get_time() / 1000000LL);
                    printf("Flow IN: %.3f L/min\n", lpm_in);
                    printf("Flow OUT: %.3f L/min\n", lpm_out);
                    printf("Flow Ratio: %.2f (%.1f%%)\n", 
                           (lpm_in > 0.01f) ? (lpm_out / lpm_in) : 0.0f,
                           (lpm_in > 0.01f) ? (lpm_out / lpm_in * 100.0f) : 0.0f);
                    printf("Servo: %.1f degrees\n", servoAngleDeg);
                    
                    const char* mode_name = (CONTROL_MODE == MODE_SETPOINT) ? "SETPOINT" : 
                                           (CONTROL_MODE == MODE_EQUALIZE) ? "EQUALIZE" : 
                                           (CONTROL_MODE == MODE_AUTO) ? "AUTO" : "T500_TEMP";
                    printf("Mode: %s\n", mode_name);
                    
                    if (CONTROL_MODE == MODE_AUTO) {
                        printf("Auto Target Ratio: %.3f (%.1f%% efficiency)\n", 
                               AUTO_TARGET_RATIO, AUTO_TARGET_RATIO * 100);
                        if (lpm_in >= MIN_INPUT_FLOW) {
                            float effective_sp = lpm_in * AUTO_TARGET_RATIO;
                            if (effective_sp > SETPOINT_LPM) effective_sp = SETPOINT_LPM;
                            printf("Effective Setpoint: %.3f L/min (based on input)\n", effective_sp);
                        }
                    } else if (CONTROL_MODE == MODE_T500_TEMP) {
                        printf("T500 Temperature Control:\n");
                        printf("  High threshold: %.1f°C (%.1f°F)\n", T500_TEMP_THRESHOLD_HIGH,
                               T500_TEMP_THRESHOLD_HIGH * 9.0f/5.0f + 32);
                        printf("  Low threshold: %.1f°C (%.1f°F)\n", T500_TEMP_THRESHOLD_LOW,
                               T500_TEMP_THRESHOLD_LOW * 9.0f/5.0f + 32);
                        printf("  Flow range: %.3f-%.3f L/min\n", T500_TEMP_MIN_FLOW, T500_TEMP_MAX_FLOW);
                        if (temp_sensor_1.valid) {
                            printf("  Current temp: %.1f°C (%.1f°F)\n", 
                                   temp_sensor_1.temperature_c, temp_sensor_1.temperature_f);
                            if (temp_sensor_1.temperature_c >= T500_TEMP_THRESHOLD_HIGH) {
                                printf("  Status: ACTIVE COOLING (temp above threshold)\n");
                            } else if (temp_sensor_1.temperature_c <= T500_TEMP_THRESHOLD_LOW) {
                                printf("  Status: MINIMAL FLOW (temp below threshold)\n");
                            } else {
                                printf("  Status: HYSTERESIS ZONE (maintaining current flow)\n");
                            }
                        } else {
                            printf("  Status: NO VALID TEMPERATURE (using setpoint mode)\n");
                        }
                    }
                    
                    printf("Base Setpoint: %.3f L/min\n", SETPOINT_LPM);
                    printf("PID: %s\n", pidEnabled ? "ENABLED" : "DISABLED");
                    printf("PID Params: KP=%.3f, KI=%.3f, KD=%.3f\n", KP, KI, KD);
                    printf("Flow Calibration:\n");
                    printf("  Input: %.3f Hz/LPM (%.0f pulses/L)\n", HZ_PER_LPM_IN, HZ_PER_LPM_IN * 60.0f);
                    printf("  Output: %.3f Hz/LPM (%.0f pulses/L)\n", HZ_PER_LPM_OUT, HZ_PER_LPM_OUT * 60.0f);
                    printf("Temperature: Sensor1=%.1f°C, Sensor2=%.1f°C\n", temp_sensor_1.temperature_c, temp_sensor_2.temperature_c);
                    
                    // T500 compatibility check
                    printf("\nT500 Compatibility:\n");
                    if (lpm_out >= 0.4f && lpm_out <= 0.6f) {
                        printf("✅ Current flow (%.3f L/min) suitable for T500 240V cooling\n", lpm_out);
                    } else if (lpm_out >= 0.09f && lpm_out <= 0.12f) {
                        printf("✅ Current flow (%.3f L/min) suitable for T500 110V cooling\n", lpm_out);
                    } else if (lpm_out < 0.09f) {
                        printf("⚠️  Current flow (%.3f L/min) below T500 minimum\n", lpm_out);
                    } else {
                        printf("⚠️  Current flow (%.3f L/min) above T500 maximum\n", lpm_out);
                    }
                    printf("T500 240V spec: 0.400-0.600 L/min (400-600 mL/min)\n");
                    printf("T500 110V spec: 0.104 L/min (3.5 US fl oz/min)\n");
                    printf("==================\n\n");
                } else if (strlen(v) > 0) {
                    // Handle setpoint command s<value>
                    float sp = strtof(v, NULL);
                    if (sp > 0.0f && isfinite(sp)) {
                        SETPOINT_LPM = sp;
                        printf("Setpoint = %.3f L/min\n", SETPOINT_LPM);
                    } else {
                        printf("Bad setpoint\n");
                    }
                } else {
                    printf("Usage: s<value> for setpoint or 'status' for system info\n");
                }
                break;

            case 'w': // wifi
                if (strcmp(line, "wifi") == 0) {
                    wifi_ap_record_t ap;
                    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
                        printf("WiFi connected to: %s\n", ap.ssid);
                        printf("RSSI: %d dBm\n", ap.rssi);
                        printf("Channel: %d\n", ap.primary);
                    } else {
                        printf("WiFi not connected\n");
                    }
                }
                break;

            case 'c': // calibration commands
                if (strncmp(line, "cin", 3) == 0) {
                    float val = strtof(line + 3, NULL);
                    if (isfinite(val) && val > 0.0f) {
                        HZ_PER_LPM_IN = val;
                        printf("Input calibration set to %.4f Hz per L/min\n", HZ_PER_LPM_IN);
                        printf("This equals %.1f pulses per liter\n", HZ_PER_LPM_IN * 60.0f);
                    } else {
                        printf("Invalid value. Usage: cin<value> (e.g., cin0.2667)\n");
                    }
                } else if (strncmp(line, "cout", 4) == 0) {
                    float val = strtof(line + 4, NULL);
                    if (isfinite(val) && val > 0.0f) {
                        HZ_PER_LPM_OUT = val;
                        printf("Output calibration set to %.4f Hz per L/min\n", HZ_PER_LPM_OUT);
                        printf("This equals %.1f pulses per liter\n", HZ_PER_LPM_OUT * 60.0f);
                    } else {
                        printf("Invalid value. Usage: cout<value> (e.g., cout0.2667)\n");
                    }
                } else {
                    printf("Unknown calibration command. Use 'cin<value>' or 'cout<value>'\n");
                }
                break;

            case 't': // T500 presets and temperature commands
                if (strcmp(line, "t500_240v") == 0) {
                    SETPOINT_LPM = 0.6f;
                    CONTROL_MODE = MODE_SETPOINT;
                    integralTerm = 0.0f; lastError = 0.0f;
                    printf("T500 240V preset: 0.6 L/min setpoint, Setpoint mode\n");
                    printf("T500 spec: 400-600 mL/min cooling water\n");
                } else if (strcmp(line, "t500_110v") == 0) {
                    SETPOINT_LPM = 0.104f;
                    CONTROL_MODE = MODE_SETPOINT;
                    integralTerm = 0.0f; lastError = 0.0f;
                    printf("T500 110V preset: 0.104 L/min setpoint, Setpoint mode\n");
                    printf("T500 spec: 3.5 US fl oz/min cooling water\n");
                    printf("WARNING: Below ZD1202 minimum range (0.6 L/min)\n");
                    printf("Consider using different flowmeter for low flows\n");
                } else if (strcmp(line, "t500_temp") == 0) {
                    CONTROL_MODE = MODE_T500_TEMP;
                    T500_TEMP_THRESHOLD_HIGH = 50.0f;  // 50°C per T500 documentation
                    T500_TEMP_THRESHOLD_LOW = 45.0f;   // 5°C hysteresis
                    T500_TEMP_MAX_FLOW = 0.6f;         // T500 240V maximum
                    T500_TEMP_MIN_FLOW = 0.1f;         // Minimum circulation
                    integralTerm = 0.0f; lastError = 0.0f;
                    printf("T500 Temperature Control Mode activated\n");
                    printf("High threshold: %.1f°C (%.1f°F)\n", T500_TEMP_THRESHOLD_HIGH, 
                           T500_TEMP_THRESHOLD_HIGH * 9.0f/5.0f + 32);
                    printf("Low threshold: %.1f°C (%.1f°F)\n", T500_TEMP_THRESHOLD_LOW,
                           T500_TEMP_THRESHOLD_LOW * 9.0f/5.0f + 32);
                    printf("Flow range: %.3f-%.3f L/min\n", T500_TEMP_MIN_FLOW, T500_TEMP_MAX_FLOW);
                } else if (strncmp(line, "temp_high", 9) == 0) {
                    float temp = strtof(line + 9, NULL);
                    if (temp > 0.0f && temp <= 100.0f && isfinite(temp)) {
                        T500_TEMP_THRESHOLD_HIGH = temp;
                        printf("High temperature threshold set to %.1f°C (%.1f°F)\n", 
                               temp, temp * 9.0f/5.0f + 32);
                    } else {
                        printf("Invalid temperature. Use temp_high<value> (e.g., temp_high50)\n");
                    }
                } else if (strncmp(line, "temp_low", 8) == 0) {
                    float temp = strtof(line + 8, NULL);
                    if (temp > 0.0f && temp <= 100.0f && isfinite(temp)) {
                        T500_TEMP_THRESHOLD_LOW = temp;
                        printf("Low temperature threshold set to %.1f°C (%.1f°F)\n", 
                               temp, temp * 9.0f/5.0f + 32);
                    } else {
                        printf("Invalid temperature. Use temp_low<value> (e.g., temp_low45)\n");
                    }
                } else {
                    printf("Unknown T500 command. Use 't500_240v', 't500_110v', 't500_temp', 'temp_high<T>', or 'temp_low<T>'\n");
                }
                break;

            default:
                printf("Unknown. 'h' for help.\n");
                break;
        }
    }
}

// ================== GPIO init =========================
static void gpio_init_for_sensor(gpio_num_t pin, gpio_isr_t isr) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,   // external 10k to 3.3V
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE       // rising edges
    };
    ESP_ERROR_CHECK(gpio_config(&io));
    ESP_ERROR_CHECK(gpio_isr_handler_add(pin, isr, NULL));
}

void app_main(void) {
    ESP_LOGI(TAG, "Booting T500 dual flow control (ESP-IDF) with OTA support");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize WiFi
    ESP_LOGI(TAG, "Initializing WiFi...");
    wifi_init_sta();

    // Start HTTP server
    server = start_webserver();
    if (server) {
        ESP_LOGI(TAG, "HTTP server started on port 80");
        ESP_LOGI(TAG, "OTA endpoint: http://[ESP32_IP]/ota");
        ESP_LOGI(TAG, "Status endpoint: http://[ESP32_IP]/status");
    }

    // ISR service
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));

    // Init sensors
    gpio_init_for_sensor(FLOW_IN_GPIO,  isr_in);
    gpio_init_for_sensor(FLOW_OUT_GPIO, isr_out);

    // Init servo
    servo_init();

    // Initialize temperature monitoring
    ESP_LOGI(TAG, "Initializing temperature monitoring...");
    ESP_ERROR_CHECK(temperature_init());
    
    // Start temperature monitoring task
    xTaskCreatePinnedToCore(temperature_task, "temperature_task", 4096, NULL, 3, NULL, tskNO_AFFINITY);

    // Periodic 1s sample timer
    const esp_timer_create_args_t targs = {
        .callback = sample_timer_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "sample_timer"
    };
    ESP_ERROR_CHECK(esp_timer_create(&targs, &sample_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(sample_timer, (uint64_t)SAMPLE_PERIOD_MS * 1000ULL));

    // CLI
    xTaskCreatePinnedToCore(cli_task, "cli_task", 4096, NULL, 2, NULL, tskNO_AFFINITY);

    ESP_LOGI(TAG, "System initialized successfully!");

    // Idle loop (everything is timer/ISR/task driven)
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
