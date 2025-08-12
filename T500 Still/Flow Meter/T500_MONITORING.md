# T500 Temperature Monitoring and Automatic Flow Control

## Overview
This guide explains how to monitor and automatically control your T500 distillation system using the ESP32 dual flow meter system with integrated temperature monitoring.

## T500 Temperature Control Requirements

Based on T500 documentation:
- **Above 50°C (122°F)**: Water flow control required
- **Below 50°C (122°F)**: Shut down/reduce water flow to reduce outlet flow  
- **Expected range**: 50-65°C (122-149°F) during active distillation
- **Dynamic adjustment**: May need several flow adjustments throughout the run

## Temperature Sensor Placement

### Recommended Sensor Positions
1. **Sensor 1 (Primary)**: T500 condenser outlet (vapor temperature)
2. **Sensor 2 (Secondary)**: Cooling water inlet temperature

### Installation Points
```
T500 System Layout:
                    
    Boiler ──── Column Head ──── Condenser ──── Product Outlet
                       │             │
                   [Sensor 1]    [Sensor 2]
                   (Vapor Temp)  (Water Temp)
                       │             │
                       └─────────────┘
                      ESP32 Monitoring
```

### Physical Installation
- **Sensor 1**: Attach to condenser outlet pipe or vapor line
- **Sensor 2**: Insert in cooling water inlet line
- **Insulation**: Wrap sensors for accurate readings
- **Waterproofing**: Ensure electrical connections are protected

## Automatic T500 Mode Operation

### Mode 3: T500 Temperature Control

**Activation:**
```bash
t500_temp         # Enable T500 temperature control mode
```

**How it works:**
1. **Temperature Monitoring**: Continuously reads Sensor 1 (vapor temperature)
2. **Automatic Flow Adjustment**: 
   - **≥50°C**: Increases cooling water flow proportionally
   - **<45°C**: Reduces to minimum circulation flow
   - **45-50°C**: Hysteresis zone (maintains current flow)
3. **Flow Calculation**: `Flow = Min_Flow + (Temp - 50°C) × Ramp_Rate`

### Default Parameters
- **High Threshold**: 50°C (122°F) - Start active cooling
- **Low Threshold**: 45°C (113°F) - Reduce to minimum flow  
- **Maximum Flow**: 0.6 L/min (T500 240V specification)
- **Minimum Flow**: 0.1 L/min (Circulation maintenance)
- **Ramp Rate**: 0.05 L/min per °C above threshold

## Configuration Commands

### Basic T500 Setup
```bash
# Quick setup for T500 temperature control
t500_temp         # Enable temperature-based control
m3               # Alternative way to set mode 3

# Check status
status           # Shows temperature control parameters and current readings
```

### Advanced Configuration
```bash
# Customize temperature thresholds
temp_high55      # Set high threshold to 55°C
temp_low50       # Set low threshold to 50°C

# Adjust flow parameters via source modification:
# T500_TEMP_MAX_FLOW = 0.8f     # Increase max flow
# T500_TEMP_MIN_FLOW = 0.05f    # Reduce min flow  
# T500_TEMP_RAMP_RATE = 0.1f    # More aggressive response
```

### Combined with Pressure Regulation
```bash
# For variable house pressure + T500 temperature control
m3               # T500 temperature mode
# Temperature control overrides pressure compensation
# Flow calculated based on T500 temperature requirements
```

## Monitoring and Operation

### Real-Time Monitoring

#### Web Dashboard
Access `http://[ESP32_IP]/` for:
- **Live temperature readings** (both sensors)
- **Current flow rates** (input/output)
- **Control mode status** (T500_TEMP)
- **Temperature thresholds** and current status
- **Flow adjustment** in real-time

#### Serial Monitor
```bash
# Continuous monitoring output
I (12345) T500: IN: 2.45 L/min  OUT: 0.55 L/min  MODE:T500_TEMP  SP:0.55  ERR:0.00  ANG:45.2°
I (12346) T500: Temp1: 52.3°C (126.1°F) ACTIVE COOLING - Temp2: 18.5°C (65.3°F)

# Detailed status
status
```

### Operation Phases

#### 1. Startup Phase (Temperature Rising)
```
Boiler Temperature: 20°C → 50°C
ESP32 Response:
- Flow: 0.1 L/min (minimum circulation)
- Status: "MINIMAL FLOW (temp below threshold)"
- Action: Monitoring temperature rise
```

#### 2. Active Distillation (50°C+)
```
Vapor Temperature: 50°C → 65°C  
ESP32 Response:
- Flow increases: 0.1 → 0.85 L/min
- Status: "ACTIVE COOLING (temp above threshold)"
- Calculation: 0.1 + (65-50) × 0.05 = 0.85 L/min
```

#### 3. End of Run (Temperature Falling)
```
Vapor Temperature: 65°C → 45°C
ESP32 Response:
- Flow decreases: 0.85 → 0.1 L/min
- Status: "MINIMAL FLOW (temp below threshold)"
- Action: Maintains circulation only
```

### Manual Override Capabilities

```bash
# Emergency manual control
a90              # Set servo to 90° (bypass temperature control)
m0               # Switch to manual setpoint mode
s0.5             # Set specific flow rate

# Return to automatic
m3               # Re-enable T500 temperature control
r                # Re-enable PID control
```

## Integration Examples

### Example 1: Basic T500 Temperature Control
```bash
# Setup sequence
t500_temp        # Enable T500 mode
status          # Verify configuration

# Expected operation:
# Temp < 45°C: Flow = 0.1 L/min
# Temp = 50°C: Flow = 0.1 L/min  
# Temp = 60°C: Flow = 0.6 L/min (0.1 + (60-50)*0.05)
# Temp > 70°C: Flow = 0.6 L/min (clamped to maximum)
```

### Example 2: Advanced Multi-Sensor Monitoring
```bash
# Monitor both temperature sensors
status

# Typical readings during distillation:
# Sensor 1 (Vapor): 78.5°C (173.3°F) - Primary control
# Sensor 2 (Water): 25.2°C (77.4°F) - Reference/verification
# Flow OUT: 0.525 L/min - Calculated based on 78.5°C
```

### Example 3: Integration with Pressure Regulation
```bash
# For systems with variable house pressure
# Temperature control takes priority over pressure compensation
m3               # T500 temperature mode
# System automatically handles both temperature and pressure
```

## Troubleshooting Temperature Control

### Temperature Sensor Issues
```bash
# Check sensor status
status

# Common issues:
# "NO VALID TEMPERATURE" - Check sensor connections
# Temperature readings: -999°C - Sensor fault
# Erratic readings - Poor thermal contact or electrical noise
```

**Solutions:**
1. **Check connections**: GPIO 34/35 for sensors
2. **Verify power**: 3.3V supply to sensors
3. **Thermal contact**: Ensure good heat transfer
4. **Calibration**: Verify against known temperature

### Flow Control Issues
```bash
# Temperature detected but flow not responding
status           # Check if PID is enabled
r               # Re-enable PID if disabled

# Flow too aggressive/slow
temp_high52     # Adjust thresholds
temp_low47      # Fine-tune hysteresis
```

### Mode Conflicts
```bash
# Wrong control mode
m3              # Ensure T500 temperature mode
status          # Verify mode is "T500_TEMP"

# Manual override active
r               # Re-enable automatic control
m3              # Return to temperature mode
```

## Performance Optimization

### PID Tuning for Temperature Control
```bash
# Conservative tuning for stable temperature response
p0.3            # Lower proportional gain (temperature changes slowly)
i0.02           # Slower integral action
d0.08           # Higher derivative to predict temperature trends

# Monitor response
status          # Check temperature and flow stability
```

### Temperature Response Optimization
```bash
# Adjust ramp rate in code for different responses:
# T500_TEMP_RAMP_RATE = 0.03f  # Gentle response
# T500_TEMP_RAMP_RATE = 0.08f  # Aggressive response
# T500_TEMP_RAMP_RATE = 0.05f  # Balanced (default)
```

## Safety Features

### Automatic Safety Systems
1. **Temperature Sensor Failsafe**: Falls back to setpoint mode if sensor invalid
2. **Flow Limiting**: Never exceeds T500 maximum flow specification
3. **Minimum Circulation**: Maintains minimum flow for system protection
4. **Hysteresis**: Prevents rapid on/off cycling around thresholds

### Manual Safety Overrides
```bash
# Emergency stop
a10             # Minimum servo angle (nearly closed)
s0              # Zero flow setpoint

# Emergency maximum cooling
a170            # Maximum servo angle (fully open)
s0.6            # Maximum T500 flow rate
```

### Monitoring Alerts
- **High Temperature Warning**: >80°C indicates possible T500 issue
- **Low Flow Warning**: <0.05 L/min may indicate blockage
- **Sensor Fault Warning**: Invalid temperature readings
- **Servo Position Limits**: Servo angle outside normal range

## Data Logging and Analysis

### Web Dashboard Analytics
- **Temperature Trends**: Track temperature rise/fall rates
- **Flow Response**: Monitor flow adjustments vs temperature
- **Efficiency Metrics**: Temperature differential vs flow rate
- **System Performance**: Overall T500 operation effectiveness

### Historical Data Collection
```bash
# Regular status logging
status          # Manual snapshot

# Continuous monitoring via web dashboard
# JSON API at: http://[ESP32_IP]/status
# Can be logged by external systems for analysis
```

## Conclusion

The T500 temperature monitoring system provides:

✅ **Automatic Flow Control** based on actual T500 temperature
✅ **Precise Temperature Monitoring** with dual sensor capability  
✅ **Integrated Pressure Regulation** for variable house water systems
✅ **Safety Features** with multiple failsafe mechanisms
✅ **Real-time Monitoring** via web dashboard and serial interface
✅ **Manual Override** capabilities for emergency situations

This system ensures optimal T500 cooling performance while protecting against both over-cooling and under-cooling conditions, automatically adapting to the distillation process requirements.

---

**Next Steps:**
1. Install temperature sensors at recommended positions
2. Configure T500 temperature mode: `t500_temp`
3. Monitor initial operation: `status`
4. Fine-tune thresholds as needed: `temp_high<T>`, `temp_low<T>`
5. Access web dashboard for continuous monitoring
