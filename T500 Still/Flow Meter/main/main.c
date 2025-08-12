#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "esp_clk.h"

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

// ================== Flow calibration =================
// Typical YF-S201 ≈ 7.5 Hz per L/min (i.e., ~450 pulses/L).
// We integrate for exactly 1.0s windows, so LPM = Hz / HZ_PER_LPM_x
static float HZ_PER_LPM_IN  = 7.50f;
static float HZ_PER_LPM_OUT = 7.50f;

// ================== Control mode =====================
// false = Setpoint mode (error = SETPOINT - OUT)
// true  = Equalize mode (error = OUT - IN)
static volatile bool MODE_EQUALIZE = false; // start in Setpoint mode

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
static float SETPOINT_LPM = 3.0f;

// Sampling
static const int SAMPLE_PERIOD_MS = 1000;

// Servo state
static float servoAngleDeg = 90.0f;
static bool  pidEnabled    = true;

// Counters (ISR)
static volatile uint32_t pulseCountIn  = 0;
static volatile uint32_t pulseCountOut = 0;

// Sampled values (updated each second)
static float lpm_in  = 0.0f;
static float lpm_out = 0.0f;

// Locks
static portMUX_TYPE spinlock_in  = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE spinlock_out = portMUX_INITIALIZER_UNLOCKED;

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

    // Compute error
    float error = MODE_EQUALIZE ? (lpm_out - lpm_in) : (SETPOINT_LPM - lpm_out);

    // Drive servo
    if (pidEnabled) {
        float delta_deg = pid_step(error);
        float req = servoAngleDeg + delta_deg;
        servo_write_angle(req);
    }

    // Telemetry
    ESP_LOGI(TAG,
        "IN: %.3f L/min  OUT: %.3f L/min  MODE:%s  SP:%.3f  ERR:%.3f  ANG:%.1f  PID(Kp=%.3f,Ki=%.3f,Kd=%.3f) I=%.2f",
        lpm_in, lpm_out, MODE_EQUALIZE ? "EQUALIZE" : "SETPOINT",
        SETPOINT_LPM, error, servoAngleDeg, KP, KI, KD, integralTerm
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
    printf("  s<value>     -> Setpoint L/min (e.g., s3.2)\n");
    printf("  p<kp>        -> Set KP\n");
    printf("  i<ki>        -> Set KI\n");
    printf("  d<kd>        -> Set KD\n");
    printf("  a<deg>       -> Manual servo angle (disables PID)\n");
    printf("  r            -> Re-enable PID\n");
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
                    MODE_EQUALIZE = false;
                    integralTerm = 0.0f; lastError = 0.0f;
                    printf("Mode = SETPOINT\n");
                } else if (strcmp(v, "1") == 0) {
                    MODE_EQUALIZE = true;
                    integralTerm = 0.0f; lastError = 0.0f;
                    printf("Mode = EQUALIZE\n");
                } else {
                    printf("Usage: m0 or m1\n");
                }
                break;

            case 's': {
                float sp = strtof(v, NULL);
                if (sp > 0.0f && isfinite(sp)) {
                    SETPOINT_LPM = sp;
                    printf("Setpoint = %.3f L/min\n", SETPOINT_LPM);
                } else {
                    printf("Bad setpoint\n");
                }
            } break;

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
                pidEnabled   = true;
                integralTerm = 0.0f;
                lastError    = 0.0f;
                printf("PID re-enabled\n");
                break;

            case 'h':
                print_help();
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
    ESP_LOGI(TAG, "Booting T500 dual flow control (ESP-IDF)");

    // ISR service
    ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1));

    // Init sensors
    gpio_init_for_sensor(FLOW_IN_GPIO,  isr_in);
    gpio_init_for_sensor(FLOW_OUT_GPIO, isr_out);

    // Init servo
    servo_init();

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

    // Idle loop (everything is timer/ISR/task driven)
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
