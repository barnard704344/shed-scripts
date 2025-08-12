#ifndef CONFIG_H
#define CONFIG_H

// WiFi Configuration - Change these to match your network
#define WIFI_SSID        "YourWiFiSSID"
#define WIFI_PASS        "YourWiFiPassword"

// OTA Configuration - Change this to your OTA server URL
#define FIRMWARE_UPGRADE_URL "http://192.168.1.100:8070/t500_dual_flow_servo.bin"

// Flow Sensor Calibration
#define HZ_PER_LPM_IN_DEFAULT   7.50f
#define HZ_PER_LPM_OUT_DEFAULT  7.50f

// PID Default Parameters
#define KP_DEFAULT  0.80f
#define KI_DEFAULT  0.05f
#define KD_DEFAULT  0.02f

// Servo Configuration
#define SERVO_MIN_DEG_DEFAULT     10
#define SERVO_MAX_DEG_DEFAULT     170
#define SERVO_MAX_STEP_DEG_DEFAULT 5.0f

// Control Parameters
#define SETPOINT_LPM_DEFAULT 3.0f

#endif // CONFIG_H
