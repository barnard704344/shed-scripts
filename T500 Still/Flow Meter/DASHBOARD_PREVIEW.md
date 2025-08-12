# Web Dashboard Preview

## What You'll See

When you navigate to `http://[ESP32_IP]/` in your browser, you'll see a professional dashboard like this:

### Header Section
```
🌡️ T500 Distillation Flow Meter
Real-time monitoring and control dashboard
```

### System Status Grid
| Connection | Firmware | Uptime | Free Memory | PID Control | OTA Status |
|------------|----------|---------|-------------|-------------|------------|
| **Online** | 54ca574-dirty | 1d 5h 23m | 195.8 KB | **ENABLED** | **READY** |

### Flow Rates Display
```
Input Flow          Output Flow
   2.450               2.380
   L/min              L/min
  (blue)             (red)
```

### Servo Control
```
Current Position: 95.2°
[████████████████████░░░░] 
10° (min) ← → 170° (max)
```

### Control Panel
- **Operating Mode**: Current: SETPOINT
  - [Setpoint Mode] [Equalize Mode]
- **Setpoint**: Current: 3.00 L/min
  - [Input: ___] [Set Setpoint]
- **System Control**:
  - [🔄 OTA Update] [🔄 Refresh]

### Instructions Section
- Serial command reference
- Control instructions
- Parameter adjustment guide

## Features

### 🔄 **Auto-Refresh**
- Updates every 2 seconds
- Real-time flow monitoring
- Live servo position tracking

### 📱 **Responsive Design**
- Works on desktop computers
- Tablet-friendly interface
- Mobile phone compatible

### 🎨 **Visual Indicators**
- **Green**: Normal operation, online status
- **Blue**: Input flow, normal values
- **Red**: Output flow, warnings
- **Gray**: Disabled features

### ⚡ **Interactive Controls**
- Click buttons to change modes
- Input fields for setpoint adjustment
- One-click OTA updates
- Real-time feedback

## Browser Compatibility
- ✅ Chrome/Chromium
- ✅ Firefox
- ✅ Safari
- ✅ Edge
- ✅ Mobile browsers

The dashboard will automatically start updating when you open it and provides a complete view of your distillation system status!
