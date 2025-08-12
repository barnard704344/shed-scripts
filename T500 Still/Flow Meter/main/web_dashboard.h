#ifndef WEB_DASHBOARD_H
#define WEB_DASHBOARD_H

#include "esp_http_server.h"
#include "temperature.h"

// Function declarations
esp_err_t dashboard_handler(httpd_req_t *req);
esp_err_t status_handler(httpd_req_t *req);

// External variables from main.c that the dashboard needs access to
extern float lpm_in;
extern float lpm_out;
extern float servoAngleDeg;
extern volatile bool MODE_EQUALIZE;
extern float SETPOINT_LPM;
extern bool pidEnabled;

#endif // WEB_DASHBOARD_H
