#ifndef WEB_DASHBOARD_H
#define WEB_DASHBOARD_H

#include "esp_http_server.h"
#include "temperature.h"

// Control mode enumeration (must match main.c)
typedef enum {
    MODE_SETPOINT = 0,    // Fixed setpoint control
    MODE_EQUALIZE = 1,    // Match output to input
    MODE_AUTO = 2,        // Pressure-compensated setpoint
    MODE_T500_TEMP = 3    // T500 temperature-based control
} control_mode_t;

// Function declarations
esp_err_t dashboard_handler(httpd_req_t *req);
esp_err_t status_handler(httpd_req_t *req);

// External variables from main.c that the dashboard needs access to
extern float lpm_in;
extern float lpm_out;
extern float servoAngleDeg;
extern volatile control_mode_t CONTROL_MODE;
extern float SETPOINT_LPM;
extern bool pidEnabled;

#endif // WEB_DASHBOARD_H
