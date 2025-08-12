# Shed Scripts & Projects

A collection of automation projects and scripts for various equipment and systems in my workshop/shed.

## 🏗️ Projects

### [T500 Still Flow Meter](T500%20Still/Flow%20Meter/)
ESP32-based flow control system for T500 distillation column with PID control, temperature monitoring, and web dashboard.

**Features:**
- Dual flow sensor monitoring (Jaycar ZD1202)
- PID-controlled servo valve
- Temperature monitoring with dual sensors
- Real-time web dashboard with WiFi connectivity
- Over-the-air (OTA) firmware updates
- Serial command interface for configuration

**Technology:** ESP32, ESP-IDF 6.0, C/C++

### [ESP32 Camera - Obico Integration](esp32cam-obico/)
ESP32-CAM setup for 3D printer monitoring with Obico integration.

**Features:**
- ESP32-CAM firmware for 3D printer monitoring
- Obico cloud service integration
- Remote monitoring capabilities

**Technology:** ESP32-CAM, PlatformIO

## 🛠️ Getting Started

Each project has its own directory with complete documentation:

- Navigate to the project folder
- Read the project-specific README.md
- Follow the setup instructions for that project

## 📋 Project Structure

```
shed-scripts/
├── README.md                           # This file
├── T500 Still/
│   └── Flow Meter/                     # ESP32 flow control system
│       ├── README.md                   # Comprehensive project docs
│       ├── main/                       # Source code
│       ├── build/                      # Build outputs
│       └── ...
└── esp32cam-obico/                     # ESP32-CAM projects
    └── esp32cam-obico/
        ├── platformio.ini
        └── src/
```

## 🔧 Development Environment

Different projects use different development environments:

- **ESP-IDF Projects**: ESP32 flow meter (native ESP-IDF)
- **PlatformIO Projects**: ESP32-CAM projects
- **Scripts**: Various utility scripts for shed automation

## 🎯 Purpose

These projects automate and monitor various equipment in my workshop/shed:

- **Distillation Equipment**: Flow control and temperature monitoring
- **3D Printers**: Remote monitoring and failure detection
- **General Automation**: Various scripts for equipment control
- **Monitoring Systems**: Data collection and visualization

## 📄 License

Projects in this repository are for personal use and learning. Individual projects may have their own licenses - check project-specific documentation.

## 🤝 Contributing

These are personal projects for my shed, but feel free to:
- Use code for your own projects
- Report issues if you find bugs
- Suggest improvements
- Fork for your own shed automation needs

---

**Built for shed automation and learning** 🔧
