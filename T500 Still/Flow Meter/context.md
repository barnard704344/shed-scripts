# T500 Flow Meter ESP32 Project Context

## Project Overview
ESP32-based dual flow sensor control system for T500 distillation column with PID-controlled servo valve.

## Project Structure
```
/home/rob/Documents/Github/shed-scripts/T500 Still/Flow Meter/
├── CMakeLists.txt          # Main ESP-IDF project file
├── context.md              # This file - project documentation
├── main/
│   ├── CMakeLists.txt      # Main component CMake file
│   └── main.c              # Main application code
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
- **Flow Sensor In**: GPIO 34 (before valve/coil) - input only pin
- **Flow Sensor Out**: GPIO 35 (after valve/coil) - input only pin  
- **Servo Control**: GPIO 18 (PWM signal to servo)

### Flow Sensors (YF-S201)
- Power: +5V and GND
- Signal: Open collector output
- Pull-up: 10kΩ from signal to 3.3V
- Calibration: ~7.5 Hz per L/min (~450 pulses/L)

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
- `status` - Show detailed system status
- `p<kp>` - Set proportional gain
- `i<ki>` - Set integral gain
- `d<kd>` - Set derivative gain
- `a<deg>` - Manual servo angle (disables PID)
- `r` - Re-enable PID
- `ota` - Trigger OTA firmware update
- `wifi` - Show WiFi connection status
- `h` - Help

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
- **Servo Position**: Visual indicator showing current servo angle (10°-170°)
- **Control Mode**: Current operating mode (Setpoint/Equalize)
- **PID Status**: Whether PID control is enabled/disabled

### Interactive Controls
- **Mode Switching**: Buttons for Setpoint/Equalize modes
- **Setpoint Adjustment**: Input field for setting target flow rate
- **OTA Updates**: One-click firmware update button
- **Real-time Updates**: Dashboard refreshes every 2 seconds

### Access Instructions
1. Flash firmware and connect to WiFi
2. Find ESP32 IP address from serial monitor
3. Open web browser and navigate to: `http://[ESP32_IP]/`
4. Dashboard will load and start updating automatically

## Build Information
- Target: ESP32
- Compiler: xtensa-esp32-elf-gcc 15.1.0
- Binary size: ~937 KB (with WiFi and OTA support)
- Partition: 1MB app partition (11% free space)

## Recent Changes
- Added WiFi connectivity support
- Implemented OTA (Over-The-Air) firmware updates
- Added HTTP server with REST API endpoints
- Added comprehensive system status reporting
- Fixed compilation errors for ESP-IDF 6.0
- Added configuration file (config.h) for easy setup

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
