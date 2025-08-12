# T500 Flow Meter ESP32 Project Context

## Project Overview
ESP32-based dual flow sensor control system for T500 distillation column with PID-controlled servo valve, temperature monitoring, and web dashboard interface.

## Project Structure
```
/home/rob/Documents/Github/shed-scripts/T500 Still/Flow Meter/
├── CMakeLists.txt          # Main ESP-IDF project file
├── context.md              # This file - project documentation
├── README.md               # Comprehensive project documentation
├── DASHBOARD_PREVIEW.md    # Web dashboard screenshots and features
├── OTA_README.md           # OTA update instructions
├── ZD1202_datasheetMain_41533.pdf  # Jaycar flowmeter datasheet
├── main/
│   ├── CMakeLists.txt      # Main component CMake file
│   ├── main.c              # Main application code
│   ├── config.h            # WiFi and system configuration
│   ├── http_server.c/.h    # HTTP server implementation
│   ├── web_dashboard.c/.h  # Web dashboard HTML/CSS/JS
│   ├── ota_update.c/.h     # OTA update functionality
│   └── temperature.c/.h    # Temperature monitoring system
├── build/                  # Build output directory (generated)
├── sdkconfig               # ESP-IDF configuration (generated)
└── sdkconfig.defaults      # Default configuration (if created)
```

## Development Environment Setup

### ESP-IDF Installation
- ESP-IDF located at: `~/esp-idf/`
- Version: ESP-IDF 6.0

### Environment Setup Commands
```bash
# Source ESP-IDF environment (run this in each new terminal)
. ~/esp-idf/export.sh

# Navigate to project directory
cd "/home/rob/Documents/Github/shed-scripts/T500 Still/Flow Meter"

# Build project
idf.py build

# Flash to ESP32
idf.py flash

# Monitor serial output
idf.py monitor

# Combined flash and monitor
idf.py flash monitor
```

## Hardware Configuration

### Pin Assignments
- **Flow Sensor In**: GPIO 4 (before valve/coil) - YF-S201 or ZD1202
- **Flow Sensor Out**: GPIO 5 (after valve/coil) - YF-S201 or ZD1202
- **Servo Control**: GPIO 18 (PWM signal to servo)
- **Temperature Sensor 1**: GPIO 34 (ADC1_CH6) - Thermistor/DS18B20/Thermocouple
- **Temperature Sensor 2**: GPIO 35 (ADC1_CH7) - Thermistor/DS18B20/Thermocouple

### Flow Sensors

#### Jaycar ZD1202 (Current Configuration)
- **Type**: Reed switch flowmeter
- **Range**: 0.6-8.0 L/min
- **Calibration**: 4.065 Hz per L/min (244 pulses/L for 2.0-8.0 L/min range)
- **Power**: +5V and GND
- **Signal**: Reed switch output
- **Pull-up**: 10kΩ from signal to 3.3V

#### Alternative: YF-S201 (Hall Effect)
- **Type**: Hall effect flowmeter
- **Calibration**: 7.5 Hz per L/min (450 pulses/L)
- **Use commands**: `cin7.5` and `cout7.5` to switch calibration

### Temperature Sensors
- **Type**: Configurable (Thermistor/DS18B20/Thermocouple/RTD)
- **Current**: 10kΩ thermistors with Steinhart-Hart equation
- **ADC**: 12-bit resolution with attenuation
- **Reference**: 10kΩ pullup resistors
- **Range**: -40°C to +125°C (thermistor dependent)

### Servo
- Power: Separate 5-6V supply (high current capable)
- Common GND with ESP32
- Signal: PWM on GPIO 18
- Range: 10° to 170°
- Pulse width: 1000-2000 μs

## Control System

### Operating Modes
1. **Setpoint Mode** (default): error = setpoint - OUT_flow
2. **Equalize Mode**: error = OUT_flow - IN_flow

### PID Parameters (default)
- KP = 0.80
- KI = 0.05  
- KD = 0.02
- Integral limits: -50.0 to +50.0
- Max servo step: 5°/second

### CLI Commands (via serial monitor)
- `m0` - Setpoint mode
- `m1` - Equalize mode
- `s<value>` - Set flow setpoint (L/min), e.g., `s3.2`
- `status` - Show detailed system status including temperature
- `p<kp>` - Set proportional gain
- `i<ki>` - Set integral gain
- `d<kd>` - Set derivative gain
- `a<deg>` - Manual servo angle (disables PID)
- `r` - Re-enable PID
- `cin<value>` - Calibrate input flow sensor (Hz per L/min)
- `cout<value>` - Calibrate output flow sensor (Hz per L/min)
- `ota` - Trigger OTA firmware update
- `wifi` - Show WiFi connection status
- `h` - Help

### Flow Sensor Calibration
Different flowmeters require different calibration values:

| Flowmeter | Range (L/min) | Hz per L/min | Pulses/L | Commands |
|-----------|---------------|--------------|----------|----------|
| **ZD1202** (2.0-8.0 range) | 0.6-8.0 | 4.065 | 244 | `cin4.065 cout4.065` |
| ZD1202 (1.5-2.0 range) | 1.5-2.0 | 4.167 | 250 | `cin4.167 cout4.167` |
| ZD1202 (1.0-1.5 range) | 1.0-1.5 | 4.386 | 263 | `cin4.386 cout4.386` |
| ZD1202 (0.8-1.0 range) | 0.8-1.0 | 4.630 | 278 | `cin4.630 cout4.630` |
| ZD1202 (0.6-0.8 range) | 0.6-0.8 | 5.051 | 303 | `cin5.051 cout5.051` |
| YF-S201 | Variable | 7.5 | 450 | `cin7.5 cout7.5` |

## OTA (Over-The-Air) Updates

### WiFi Configuration
Edit `main/config.h` to set your WiFi credentials:
```c
#define WIFI_SSID        "YourWiFiSSID"
#define WIFI_PASS        "YourWiFiPassword"
#define FIRMWARE_UPGRADE_URL "http://192.168.1.100:8070/t500_dual_flow_servo.bin"
```

### OTA Update Methods
1. **Serial Command**: Type `ota` in serial monitor
2. **HTTP API**: POST to `http://[ESP32_IP]/ota`
3. **Web Interface**: Navigate to `http://[ESP32_IP]/status` for system info

### Setting Up OTA Server
To host firmware updates, you can use a simple HTTP server:
```bash
# In your build directory, serve the binary file
python3 -m http.server 8070
# Your firmware will be available at: http://[YOUR_IP]:8070/t500_dual_flow_servo.bin
```

### HTTP Endpoints
- `GET /` - **Web Dashboard** - Interactive HTML interface with real-time data
- `GET /status` - JSON system status (firmware version, heap, flow rates, etc.)
- `POST /ota` - Trigger OTA update from configured URL

## Web Dashboard Features

### Real-time Display
- **System Status**: Connection, firmware version, uptime, memory usage
- **Flow Monitoring**: Live input/output flow rates with color-coded display
- **Temperature Monitoring**: Real-time temperature from dual sensors (°C and °F)
- **Servo Position**: Visual indicator showing current servo angle (10°-170°)
- **Control Mode**: Current operating mode (Setpoint/Equalize)
- **PID Status**: Whether PID control is enabled/disabled
- **Sensor Status**: Visual indicators for temperature sensor validity

### Interactive Controls
- **Mode Switching**: Buttons for Setpoint/Equalize modes
- **Setpoint Adjustment**: Input field for setting target flow rate
- **Auto-refresh**: Toggle automatic data updates (every 2 seconds)
- **Manual Refresh**: On-demand data refresh button
- **OTA Updates**: One-click firmware update button

### JSON API Endpoints
- `GET /status` - Complete system data including:
  - Flow rates (input/output)
  - Temperature readings (both sensors, °C/°F, validity status)
  - Servo position and control mode
  - PID parameters and status
  - System health (memory, uptime, WiFi)

### Access Instructions
1. Flash firmware and connect to WiFi
2. Find ESP32 IP address from serial monitor
3. Open web browser and navigate to: `http://[ESP32_IP]/`
4. Dashboard will load and start updating automatically

## Build Information
- Target: ESP32
- Compiler: xtensa-esp32-elf-gcc 15.1.0
- Binary size: ~949 KB (with WiFi, OTA, and temperature monitoring)
- Partition: 1MB app partition (9% free space)
- Components: esp_adc (for temperature), esp_http_server, esp_wifi, esp_https_ota

## Recent Changes
- **Temperature Monitoring**: Added dual temperature sensor support (GPIO 34/35)
- **ZD1202 Calibration**: Updated for Jaycar ZD1202 flowmeters with datasheet values
- **Runtime Calibration**: Added serial commands for flowmeter calibration adjustment
- **Enhanced Web Dashboard**: Added temperature display with real-time updates
- **JSON API**: Extended status endpoint with temperature data
- **Sensor Support**: Configurable sensor types (thermistor/DS18B20/thermocouple/RTD)
- **ADC Integration**: 12-bit ADC with Steinhart-Hart temperature calculations

## Git Repository
- Repository: `shed-scripts`
- Owner: `barnard704344`
- Current branch: `main`
- Project path: `T500 Still/Flow Meter/`

## Notes
- Use `idf.py menuconfig` to configure ESP-IDF settings if needed
- VS Code may show include errors (red squiggles) but compilation works fine
- Install ESP-IDF VS Code extension for better development experience
- Keep all grounds common between ESP32, sensors, and servo power supply
