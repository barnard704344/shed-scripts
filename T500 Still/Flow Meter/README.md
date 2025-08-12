# T500 Flow Meter Control System

![ESP32](https://img.shields.io/badge/ESP32-ESP--IDF%206.0-blue)
![Build](https://img.shields.io/badge/build-passing-brightgreen)
![License](https://img.shields.io/badge/license-MIT-green)

A sophisticated ESP32-based flow control system for T500 distillation columns featuring PID control, temperature monitoring, web dashboard, and OTA updates.

## 🚀 Features

### Core Functionality
- **Dual Flow Monitoring**: Precision flow measurement with Jaycar ZD1202 flowmeters
- **PID Control**: Servo-controlled valve with adaptive PID algorithm
- **Temperature Monitoring**: Dual temperature sensors with multiple sensor type support
- **Web Dashboard**: Real-time HTML interface with live data visualization
- **OTA Updates**: Over-the-air firmware updates via WiFi
- **Serial Interface**: Comprehensive CLI for configuration and monitoring

### Advanced Features
- **Multiple Control Modes**: Setpoint control and flow equalization
- **Runtime Calibration**: Adjust sensor calibration without reflashing
- **Sensor Flexibility**: Support for thermistors, DS18B20, thermocouples, and RTDs
- **Professional Web UI**: Responsive design with auto-refresh and manual controls
- **JSON API**: RESTful endpoints for integration with external systems

## 📋 Table of Contents

- [Hardware Requirements](#-hardware-requirements)
- [Pin Configuration](#-pin-configuration)
- [Quick Start](#-quick-start)
- [Web Dashboard](#-web-dashboard)
- [API Reference](#-api-reference)
- [Configuration](#-configuration)
- [Calibration](#-calibration)
- [OTA Updates](#-ota-updates)
- [Troubleshooting](#-troubleshooting)

## 🔧 Hardware Requirements

### Core Components
- **ESP32 Development Board** (30-pin recommended)
- **2x Jaycar ZD1202 Flow Sensors** (0.6-8.0 L/min range)
- **Servo Motor** (5V, high-torque for valve control)
- **2x Temperature Sensors** (10kΩ thermistors recommended)
- **Power Supply** (5V for sensors and servo, 3.3V for ESP32)

### Optional Components
- **Pull-up Resistors**: 2x 10kΩ for flow sensors, 2x 10kΩ for thermistors
- **Capacitors**: 100μF for servo power stabilization
- **Level Shifters**: If using 5V sensors with 3.3V ESP32

## 📌 Pin Configuration

| Component | ESP32 Pin | Function | Notes |
|-----------|-----------|----------|-------|
| Flow Sensor (Input) | GPIO 4 | Pulse input | Reed switch, 10kΩ pullup |
| Flow Sensor (Output) | GPIO 5 | Pulse input | Reed switch, 10kΩ pullup |
| Servo Control | GPIO 18 | PWM output | 1000-2000μs pulse width |
| Temperature Sensor 1 | GPIO 34 | ADC input | Thermistor with 10kΩ ref |
| Temperature Sensor 2 | GPIO 35 | ADC input | Thermistor with 10kΩ ref |

### Wiring Diagram
```
ESP32                    ZD1202 Flow Sensor
                        ┌─────────────────┐
GPIO 4  ──10kΩ──3.3V ───┤ Signal          │
                        │                 │
5V      ─────────────────┤ VCC    Flow     │
                        │        ────>    │
GND     ─────────────────┤ GND             │
                        └─────────────────┘

ESP32                    Servo Motor
                        ┌─────────────────┐
GPIO 18 ─────────────────┤ Signal (Yellow) │
                        │                 │
5V      ─────────────────┤ VCC (Red)       │
                        │                 │
GND     ─────────────────┤ GND (Brown)     │
                        └─────────────────┘
```

## 🚀 Quick Start

### 1. Development Environment Setup

```bash
# Install ESP-IDF 6.0
cd ~
git clone --recursive https://github.com/espressif/esp-idf.git
cd esp-idf
./install.sh

# Source environment (add to ~/.bashrc for persistence)
. ~/esp-idf/export.sh
```

### 2. Clone and Configure

```bash
# Clone repository
git clone https://github.com/barnard704344/shed-scripts.git
cd "shed-scripts/T500 Still/Flow Meter"

# Configure WiFi credentials
cp main/config.h.example main/config.h
# Edit main/config.h with your WiFi details
```

### 3. Build and Flash

```bash
# Build firmware
idf.py build

# Flash to ESP32 (replace PORT with your serial port)
idf.py -p /dev/ttyUSB0 flash monitor

# Or combined command
idf.py -p /dev/ttyUSB0 flash monitor
```

### 4. First Boot

1. Monitor serial output for WiFi connection
2. Note the IP address displayed
3. Open web browser to `http://[ESP32_IP]/`
4. Verify flow sensors and temperature readings

## 🌐 Web Dashboard

### Features
- **Real-time Data**: Flow rates, temperatures, servo position
- **Visual Indicators**: Color-coded status for all sensors
- **Interactive Controls**: Mode switching, setpoint adjustment
- **Auto-refresh**: Updates every 2 seconds
- **Responsive Design**: Works on desktop and mobile

### Screenshots
![Dashboard Preview](DASHBOARD_PREVIEW.md)

### Access
```
http://[ESP32_IP]/          # Main dashboard
http://[ESP32_IP]/status    # JSON API data
```

## 🔌 API Reference

### GET /status
Returns complete system status in JSON format.

**Response Example:**
```json
{
  "firmware_version": "1.0.0",
  "heap_free": 234567,
  "uptime": 3600,
  "flow_in": 2.45,
  "flow_out": 2.43,
  "servo_angle": 45.5,
  "mode": "SETPOINT",
  "setpoint": 2.50,
  "pid_enabled": true,
  "temp1_c": 78.5,
  "temp1_f": 173.3,
  "temp1_valid": true,
  "temp2_c": 82.1,
  "temp2_f": 179.8,
  "temp2_valid": true
}
```

### POST /ota
Triggers OTA firmware update from configured URL.

## ⚙️ Configuration

### WiFi Setup
Edit `main/config.h`:
```c
#define WIFI_SSID        "YourWiFiNetwork"
#define WIFI_PASS        "YourWiFiPassword"
#define FIRMWARE_UPGRADE_URL "http://192.168.1.100:8070/firmware.bin"
```

### PID Parameters
Default values (adjustable via serial commands):
```c
KP = 0.80f    // Proportional gain
KI = 0.05f    // Integral gain  
KD = 0.02f    // Derivative gain
```

### Flow Range
Set initial setpoint and limits:
```c
SETPOINT_LPM = 2.5f       // Default target flow rate
SERVO_MIN_DEG = 10        // Minimum servo angle
SERVO_MAX_DEG = 170       // Maximum servo angle
```

## 📏 Calibration

### Flow Sensor Calibration

The system supports multiple flowmeter types with runtime calibration:

| Flowmeter | Range (L/min) | Command | Hz per L/min | Pulses/L |
|-----------|---------------|---------|--------------|----------|
| **ZD1202** (default) | 2.0-8.0 | `cin4.065 cout4.065` | 4.065 | 244 |
| ZD1202 (low flow) | 0.6-0.8 | `cin5.051 cout5.051` | 5.051 | 303 |
| YF-S201 | Variable | `cin7.5 cout7.5` | 7.5 | 450 |

### Serial Commands

#### Flow Control
```bash
m0              # Setpoint mode (control output to match setpoint)
m1              # Equalize mode (match output to input)
s2.5            # Set setpoint to 2.5 L/min
```

#### PID Tuning
```bash
p0.8            # Set proportional gain
i0.05           # Set integral gain  
d0.02           # Set derivative gain
r               # Re-enable PID after manual control
```

#### Sensor Calibration
```bash
cin4.065        # Calibrate input sensor (ZD1202)
cout4.065       # Calibrate output sensor (ZD1202)
status          # Show current calibration values
```

#### Manual Control
```bash
a45             # Set servo to 45 degrees (disables PID)
r               # Re-enable automatic PID control
```

### Temperature Sensor Configuration

Temperature sensors are configurable in `temperature.h`:

```c
typedef enum {
    TEMP_SENSOR_THERMISTOR,   // 10kΩ NTC thermistor (default)
    TEMP_SENSOR_DS18B20,      // Digital 1-Wire sensor
    TEMP_SENSOR_THERMOCOUPLE, // K-type thermocouple
    TEMP_SENSOR_RTD           // PT100 RTD
} temp_sensor_type_t;
```

## 🔄 OTA Updates

### Setup OTA Server
```bash
# In your build directory
cd build
python3 -m http.server 8070

# Your firmware URL: http://[YOUR_IP]:8070/t500_dual_flow_servo.bin
```

### Trigger Update
```bash
# Via serial command
ota

# Via HTTP API
curl -X POST http://[ESP32_IP]/ota

# Via web dashboard
# Click "Trigger OTA Update" button
```

### Update Process
1. ESP32 downloads firmware from configured URL
2. Validates firmware integrity
3. Writes to OTA partition
4. Reboots with new firmware
5. Falls back to previous version if boot fails

## 🛠️ Troubleshooting

### Common Issues

#### Flow Sensors Not Reading
```bash
# Check calibration
status

# Verify sensors are connected and powered
# Check for 10kΩ pullup resistors
# Ensure flow is actually present

# Recalibrate if needed
cin4.065
cout4.065
```

#### WiFi Connection Issues
```bash
# Check credentials in config.h
# Verify WiFi network is 2.4GHz (ESP32 doesn't support 5GHz)
# Check signal strength

# Monitor connection status
wifi
```

#### Temperature Readings Invalid
```bash
# Check sensor connections
# Verify 10kΩ reference resistors for thermistors
# Check ADC calibration

# View temperature status
status
```

#### Servo Not Responding
```bash
# Check servo power supply (5V, adequate current)
# Verify common ground between ESP32 and servo
# Test manual control

a90              # Set servo to 90 degrees
```

### Serial Monitor Output
```
I (1234) T500: Booting T500 dual flow control with OTA support
I (1250) T500: WiFi connected to: YourNetwork
I (1260) T500: HTTP server started on port 80
I (1270) T500: Temperature monitoring initialized
I (1280) T500: Flow IN: 2.45 L/min, OUT: 2.43 L/min
I (1290) T500: Servo: 45.5°, Mode: SETPOINT, PID: ENABLED
```

### Build Issues
```bash
# Clean and rebuild
idf.py clean
idf.py build

# Check ESP-IDF environment
idf.py --version

# Update submodules
git submodule update --init --recursive
```

## 📊 Performance Specifications

| Parameter | Value | Notes |
|-----------|-------|-------|
| **Flow Range** | 0.6-8.0 L/min | ZD1202 specification |
| **Flow Accuracy** | ±10% | Per ZD1202 datasheet |
| **Temperature Range** | -40°C to +125°C | Thermistor dependent |
| **Temperature Resolution** | 0.1°C | 12-bit ADC |
| **Servo Range** | 10° to 170° | Configurable limits |
| **Control Period** | 1 second | PID update rate |
| **Web Update Rate** | 2 seconds | Dashboard refresh |
| **WiFi Range** | ~30m indoor | ESP32 limitation |

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- **ESP-IDF Framework** - Espressif's IoT Development Framework
- **Jaycar Electronics** - ZD1202 flowmeter specifications
- **PID Control Theory** - Classical control system implementation
- **Web Dashboard** - Modern responsive design principles

## 📞 Support

For issues, questions, or contributions:
- **GitHub Issues**: [Project Issues](https://github.com/barnard704344/shed-scripts/issues)
- **Repository**: [shed-scripts](https://github.com/barnard704344/shed-scripts)
- **Documentation**: [context.md](context.md)

---

**Built with ❤️ for the distillation community**
