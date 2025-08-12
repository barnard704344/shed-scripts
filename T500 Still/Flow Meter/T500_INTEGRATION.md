# T500 Distillation Column - Flow Control Integration

## Overview
This guide covers integrating the ESP32 dual flow meter system with T500 distillation columns, providing automated cooling water control with proper flow rates and pressure regulation.

## T500 Cooling Water Specifications

### Flow Rate Requirements

| T500 Model | Voltage | Flow Rate | Equivalent |
|------------|---------|-----------|------------|
| **T500 (240V)** | 240V | 400-600 mL/min | 0.400-0.600 L/min |
| **T500 (110V)** | 110V | ~104 mL/min | 0.104 L/min |
| | | 3.5 US fl oz/min | |

### Timing
- **Start adjustment**: ~35 minutes after boiler turned on
- **US/CA units**: ~70 minutes after boiler turned on
- **Method**: Gradually open water flow control valve
- **Measurement**: Use measuring vessel to verify actual flow

## Quick Setup for T500

### 1. Flash ESP32 with T500-Optimized Firmware
```bash
cd "/home/rob/Documents/Github/shed-scripts/T500 Still/Flow Meter"
idf.py -p /dev/ttyUSB0 flash monitor
```

### 2. Configure for Your T500 Model

**For T500 240V Units:**
```bash
t500_240v     # Sets 0.6 L/min setpoint, Setpoint mode
m1            # Switch to Equalize mode for pressure regulation
```

**For T500 110V Units:**
```bash
t500_110v     # Sets 0.104 L/min setpoint, Setpoint mode
# WARNING: Below ZD1202 minimum - see Low Flow Solutions below
```

### 3. Monitor and Adjust
```bash
status        # Check T500 compatibility and current performance
```

## ZD1202 Flowmeter Compatibility

### Compatibility Analysis
- **ZD1202 Range**: 0.6-8.0 L/min (600-8000 mL/min)
- **T500 240V**: ✅ **COMPATIBLE** (0.4-0.6 L/min within range)
- **T500 110V**: ❌ **BELOW MINIMUM** (0.104 L/min < 0.6 L/min minimum)

### Low Flow Solutions for T500 110V

#### Option 1: Use Lower-Range Flowmeter
Replace ZD1202 with flowmeters designed for lower flows:
- **YF-S401** (0.3-6 L/min range)
- **YF-S402** (0.3-6 L/min range)
- **Custom hall-effect sensors**

#### Option 2: Parallel Measurement
- Measure total house water flow (higher range)
- Use ratio control to maintain T500 cooling flow
- Suitable for integrated house water systems

#### Option 3: Reduced Accuracy Mode
- Use ZD1202 with understanding of reduced accuracy
- Apply correction factors for low-flow operation
- Monitor via external flow measurement verification

## Installation Configurations

### Configuration A: Dedicated T500 Cooling Line
```
House Water ──[Input Flowmeter]──[Servo Valve]──[Output Flowmeter]──► T500 Condenser
   Supply         (GPIO 34)                        (GPIO 35)
```

**Recommended Settings:**
- **Mode**: Equalize (`m1`) for pressure compensation
- **Setpoint**: T500-specific (`t500_240v` or `t500_110v`)
- **Benefits**: Direct T500 control, precise flow management

### Configuration B: House Water Integration
```
House Water ──[Input Flowmeter]──[Servo Valve]──[Output Flowmeter]──► Multiple Outlets
   Supply         (GPIO 34)                        (GPIO 35)           ├── T500 Condenser
                                                                       ├── Other Uses
```

**Recommended Settings:**
- **Mode**: Auto (`m2`) for intelligent pressure compensation
- **Setpoint**: Higher value for total flow (2-5 L/min)
- **Ratio**: Configure for efficiency (`ratio0.7`)
- **Benefits**: Whole-house pressure regulation, T500 gets consistent supply

## Advanced T500 Integration

### Temperature-Based Control
Using the dual temperature sensors for enhanced control:

```bash
# Monitor both condenser inlet and outlet temperatures
status    # Shows temp1 and temp2 readings

# Adjust flow based on temperature differential
# Higher temp differential = increase flow
# Lower temp differential = can reduce flow
```

### Distillation Phase Detection
```bash
# During different phases, T500 heat load varies:
# Heat-up phase: Lower cooling needed
# Active distillation: Maximum cooling needed  
# Tail fraction: Reduced cooling acceptable

# Use temperature sensors to detect phase changes
# Automatically adjust setpoint based on phase
```

### Integration with T500 Controls
- **Manual Flow Valve**: Replace with servo-controlled valve
- **Pressure Relief**: Maintain existing T500 safety systems
- **Temperature Monitoring**: Use ESP32 sensors for inlet/outlet temp
- **Data Logging**: Web dashboard provides real-time monitoring

## Calibration for T500 Applications

### Precision Calibration for Low Flows
```bash
# For T500 240V (0.4-0.6 L/min range)
# Use ZD1202 low-flow calibration:
cin5.051      # For 0.6-0.8 L/min range
cout5.051     # Pulses per liter: 303

# Verify with external measurement
# Use graduated cylinder and stopwatch
# Measure actual flow vs. displayed flow
```

### Verification Procedure
1. **Set known setpoint**: `s0.5` (for T500 240V)
2. **Collect water sample**: Use 1L measuring container
3. **Time collection**: Should take ~2 minutes for 1L at 0.5 L/min
4. **Calculate actual flow**: `Actual L/min = Volume(L) / Time(min)`
5. **Adjust calibration**: Use `cin` and `cout` commands if needed

## Operational Procedures

### Startup Sequence for T500 Operation
1. **Power on ESP32**: Wait for WiFi connection
2. **Set T500 mode**: `t500_240v` or `t500_110v`
3. **Enable pressure regulation**: `m1` (Equalize mode)
4. **Start T500 boiler**: Begin normal T500 operation
5. **Wait for thermal equilibrium**: ~35-70 minutes
6. **Fine-tune if needed**: Use `s<value>` to adjust setpoint
7. **Monitor continuously**: Check web dashboard or use `status`

### During Distillation Run
```bash
# Check system status every 30 minutes
status

# Adjust for distillation phases:
s0.4    # Reduce flow during heat-up
s0.6    # Maximum flow during active distillation
s0.5    # Moderate flow during tail fractions
```

### Shutdown Sequence
1. **Reduce flow gradually**: `s0.2` then `s0.1`
2. **Turn off T500 boiler**: Follow normal T500 shutdown
3. **Stop water flow**: `s0` or manual valve closure
4. **Monitor cooldown**: Use temperature sensors

## Troubleshooting T500 Integration

### Flow Too High
**Symptoms**: T500 overcooling, poor separation
**Solutions**:
```bash
t500_240v     # Reset to proper T500 setpoint
m0            # Use Setpoint mode for precise control
s0.4          # Reduce to minimum T500 specification
```

### Flow Too Low  
**Symptoms**: T500 overheating, poor condensation
**Solutions**:
```bash
s0.6          # Increase to maximum T500 specification
m1            # Switch to Equalize mode for pressure boost
status        # Check for input pressure issues
```

### Inconsistent Flow
**Symptoms**: Flow varies significantly, poor control
**Solutions**:
```bash
m1            # Use Equalize mode for pressure regulation
p1.0          # Increase proportional gain
i0.1          # Increase integral gain for steady-state accuracy
```

### ZD1202 Low Flow Issues (110V T500)
**Symptoms**: Erratic readings below 0.6 L/min
**Solutions**:
1. **External verification**: Use measuring cup method
2. **Averaged readings**: Monitor trends rather than instantaneous values
3. **Alternative sensors**: Consider YF-S401 or custom solutions
4. **Flow scaling**: Measure total supply line, calculate T500 portion

## Safety Considerations

### T500 Safety Integration
- **Never interrupt cooling**: During active distillation
- **Monitor temperature**: Use ESP32 sensors for early warning
- **Backup manual valve**: Always maintain manual override capability
- **Pressure relief**: Ensure T500 safety systems remain functional

### ESP32 System Safety
- **Power backup**: Consider UPS for critical operations
- **Network monitoring**: Web dashboard for remote monitoring
- **Watchdog timers**: System will recover from hangs
- **Manual override**: Physical valve control remains available

## Performance Optimization

### PID Tuning for T500
```bash
# Conservative tuning for stable T500 operation:
p0.5          # Lower proportional gain for stability
i0.03         # Slower integral action
d0.01         # Minimal derivative action

# Monitor response:
status        # Check flow stability

# Aggressive tuning for fast response:
p1.2          # Higher proportional gain
i0.08         # Faster integral action  
d0.05         # More derivative damping
```

### Web Dashboard Monitoring
- **Access**: `http://[ESP32_IP]/`
- **Real-time data**: Flow rates, temperatures, servo position
- **T500 indicators**: Flow compatibility warnings
- **Historical trends**: Monitor performance over time

## Integration Examples

### Example 1: Basic T500 240V Setup
```bash
# Initial configuration
t500_240v     # Set T500 240V preset (0.6 L/min)
m1            # Enable pressure regulation
status        # Verify T500 compatibility

# During distillation
# Flow should maintain 0.6 L/min ± 10%
# Temperature differential should be 10-20°C
```

### Example 2: Advanced Multi-Zone Control
```bash
# Configure for house water system with T500 branch
m2            # Auto mode for intelligent control
s3.0          # Total house flow setpoint
ratio0.8      # 80% efficiency target
status        # Monitor effective setpoint

# T500 gets consistent supply despite house pressure variations
```

## Conclusion

This integration provides:
- **Automated T500 cooling control** with proper flow rates
- **Pressure regulation** for variable house water systems  
- **Temperature monitoring** for process optimization
- **Remote monitoring** via web dashboard
- **Safety features** with manual override capability

The dual flow meter system ensures your T500 operates with optimal cooling water flow regardless of house pressure variations, improving distillation quality and system reliability.

---

**For technical support, refer to:**
- [README.md](README.md) - Complete system documentation
- [context.md](context.md) - Technical implementation details
- Web dashboard at `http://[ESP32_IP]/` for real-time monitoring
