# OTA Setup Guide

## Quick Start

1. **Configure WiFi**: Edit `main/config.h` with your WiFi credentials:
   ```c
   #define WIFI_SSID        "YourWiFiSSID"
   #define WIFI_PASS        "YourWiFiPassword"
   ```

2. **Build and Flash**: 
   ```bash
   idf.py build flash monitor
   ```

3. **Find ESP32 IP**: Check serial output for WiFi connection and IP address

4. **Set up OTA server**: In your project build directory:
   ```bash
   python3 -m http.server 8070
   ```

5. **Update config.h** with your computer's IP:
   ```c
   #define FIRMWARE_UPGRADE_URL "http://192.168.1.100:8070/t500_dual_flow_servo.bin"
   ```

6. **Access Dashboard**: 
   - Open web browser and go to `http://[ESP32_IP]/`
   - View real-time flow data and system status
   - Use web interface for OTA updates and monitoring

## Web Dashboard

### Main Interface (`http://[ESP32_IP]/`)
The dashboard provides a comprehensive view of your flow meter system:

- **📊 Real-time Flow Display**: Large, color-coded flow rate indicators
- **🎛️ System Status Grid**: Firmware version, uptime, memory, connection status
- **🔧 Servo Control**: Visual position indicator with min/max range
- **⚙️ Control Panel**: Mode switching, setpoint adjustment, OTA controls
- **📋 Quick Reference**: Serial command guide for advanced control

### Features:
- **Auto-refresh**: Updates every 2 seconds
- **Responsive Design**: Works on desktop, tablet, and mobile
- **Visual Indicators**: Color-coded status (green=good, red=error)
- **Interactive Controls**: Click buttons to change modes or trigger updates
- **Real-time Servo Position**: Visual bar showing current servo angle

## API Endpoints

### GET / (Root)
Returns a complete HTML dashboard with:
- Real-time system monitoring
- Interactive flow rate display
- Servo position indicator
- Control buttons for mode switching
- OTA update interface
- Responsive design for all devices

### GET /status
Returns JSON with system information:
```json
{
  "firmware_version": "54ca574-dirty",
  "heap_free": 234567,
  "uptime": 123,
  "flow_in": 2.450,
  "flow_out": 2.380,
  "servo_angle": 95.2,
  "mode": "SETPOINT",
  "setpoint": 3.000,
  "pid_enabled": true,
  "ota_in_progress": false
}
```

### POST /ota
Triggers OTA update from the configured URL.

## Security Notes

- This implementation uses HTTP (not HTTPS) for simplicity
- For production use, consider implementing:
  - Authentication/authorization
  - HTTPS with certificate verification
  - Firmware signature verification
  - Rollback capability

## Troubleshooting

- **WiFi not connecting**: Check SSID/password in config.h
- **OTA fails**: Ensure HTTP server is running and URL is correct
- **Out of memory**: The ESP32 needs enough free heap for OTA operations
- **Build fails**: Make sure all component dependencies are included
