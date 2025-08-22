# Power System Firmware Documentation

## Overview

The Power System firmware consists of two main components designed for ESP32 WROOM-DA microcontrollers:

1. **PowerSystem_master.ino** - Main power management controller
2. **PowerSystem_sensor.ino** - Power monitoring and measurement system

This system implements an intelligent dual-battery backup solution with real-time power monitoring, automatic battery switching, and comprehensive state management.

## System Architecture

### Hardware Components

#### Master Controller (PowerSystem_master.ino)
- **ESP32 WROOM-DA** - Main processing unit
- **Dual Battery System** - Two 12V lead-acid batteries
- **Relay Control** - Battery switching between charger and inverter
- **Power Monitoring** - AC/DC voltage and current sensing
- **User Interface** - Push button and serial CLI

#### Sensor Module (PowerSystem_sensor.ino)
- **ESP32 WROOM-DA** - Dedicated measurement processor
- **OLED Display** - Real-time measurement display
- **Multiple Sensors** - 7 analog measurement channels
- **UART Communication** - Data transmission to master

### Communication Protocol

The master and sensor modules communicate via UART2 (115200 baud):
- **Command**: `GET\n` - Request measurements
- **Response**: JSON format with 7 measurement values

```json
{"ac_v1":120.5,"ac_v2":119.8,"ac_i1":2.34,"ac_i2":1.87,"dc_v1":13.2,"dc_v2":12.8,"dc_i":5.6}
```

## Pin Configuration

### Master Controller Pins

| Pin | Function | Description |
|-----|----------|-------------|
| 18 | Button Input | Momentary push switch (debounced) |
| 19 | Power Good Sense | HIGH=normal, LOW=power outage |
| 23 | Power Relay Output | Mirrors power switch state |
| 21 | UART2 RX | Communication with sensor module |
| 22 | UART2 TX | Communication with sensor module |
| 25 | Battery 1 Relay | Switches between CHARGER/INVERTER |
| 26 | Battery 2 Relay | Switches between CHARGER/INVERTER |
| 4 | CAN RX (Reserved) | Future CAN bus implementation |
| 5 | CAN TX (Reserved) | Future CAN bus implementation |

### Sensor Module Pins

| Pin | Function | Sensor Type | Description |
|-----|----------|-------------|-------------|
| 32 | AC Voltage 1 | ZMPT101B | AC voltage measurement channel 1 |
| 33 | AC Voltage 2 | ZMPT101B | AC voltage measurement channel 2 |
| 34 | AC Current 1 | ACS712-20A | AC current measurement channel 1 |
| 35 | AC Current 2 | ACS712-20A | AC current measurement channel 2 |
| 36 (VP) | DC Voltage 1 | 5:1 Divider | DC voltage measurement battery 1 |
| 39 (VN) | DC Voltage 2 | 5:1 Divider | DC voltage measurement battery 2 |
| 25 | DC Current | ACS712-20A | DC current measurement (shared) |

#### OLED Display (SSD1305, 128x32, SPI)
| Pin | Function |
|-----|----------|
| 23 | MOSI |
| 18 | CLK |
| 16 | DC |
| 5 | CS |
| 17 | RST |

## Battery Management System

### State of Charge (SOC) Mapping

The system uses voltage-based SOC estimation:

| Voltage Range | SOC Level | Status |
|---------------|-----------|--------|
| < 10.0V | N/A | Offline/Damaged |
| 10.0V - 12.0V | 0% | Critical |
| 12.0V - 13.15V | 25% | Low |
| 13.15V - 13.20V | 50% | Medium |
| 13.20V - 13.33V | 75% | High |
| ≥ 13.33V | 100% | Full |

### Battery Switching Logic

#### Normal Operation
1. **Initial Selection**: Charge the battery with lower voltage
2. **Switching Condition**: Switch to charging the other battery when the non-charging battery's SOC ≤ charge-switching-SOC (default 0%)
3. **Safety Rules**:
   - Only one battery on CHARGER at any time
   - At least one battery on INVERTER at all times
   - 7-second rest period between charger connections
   - 10-second minimum dwell time on charger

#### Emergency Conditions
1. **Power Outage**: Both batteries immediately switch to INVERTER
2. **Battery Offline**: Offline battery switches to CHARGER for recovery
3. **Both Offline**: Last detected offline battery stays on INVERTER

### Battery Full Detection

- **Threshold**: |DC Current| < 0.5A for 5 seconds
- **Action**: Increment battery full counter (persistent)
- **Global Flag**: Set `g_globalBatteryFull` state

## Multi-Core Task Architecture

### Master Controller Tasks

#### Core 1 (Application Core)
- **TaskIO** (Priority 2): Button debouncing, power monitoring, CLI
- **TaskArbiter** (Priority 2): Battery management logic
- **TaskBatt1** (Priority 2): Battery 1 relay control
- **TaskBatt2** (Priority 2): Battery 2 relay control

#### Core 0 (Protocol Core)
- **TaskUART** (Priority 3): UART communication with sensor module

### Sensor Module Tasks

#### Core 0 (Protocol Core)
- **samplerTask** (Priority 3): High-rate ADC sampling and RMS calculations

#### Core 1 (Application Core)
- **uiTask** (Priority 2): OLED display updates and UART command handling

## Persistent Storage (NVS)

The system stores critical state in Non-Volatile Storage:

| Key | Type | Description |
|-----|------|-------------|
| `pwrSw` | bool | Power switch state |
| `chgSoc` | uint8 | Charge switching SOC threshold |
| `cnt0` | uint32 | Battery 1 full charge counter |
| `cnt1` | uint32 | Battery 2 full charge counter |
| `lastChg` | int8 | Last charging battery (-1, 0, or 1) |

## Command Line Interface (CLI)

Access via USB Serial at 115200 baud:

### Commands
- `S=<n>` - Set charge switching SOC (0, 25, 50, 75, 100)
- `STATE` - Print comprehensive system state
- `COUNTS` - Display battery full charge counters

### Example Output
```
--- STATE --- @12345 ms
PwrSwitch=ON  PwrOutage=NORMAL  requestTurnOff=NO  chgSOC=0  chargerAvailAt=0
Meas: dc_v1=13.20  dc_v2=12.80  dc_i=2.50  (t=12340)
B1: desired=INV actual=INV soc=75 full=N cnt=15 lastSw=10000 chgStart=0
B2: desired=CHG actual=CHG soc=25 full=N cnt=12 lastSw=8000 chgStart=8000
ChargingBattery=1  GlobalFull=N
```

## Safety Features

### Power Outage Handling
1. **Immediate Response**: Both batteries switch to INVERTER
2. **Button Disable**: Power button input disabled during outage
3. **Turn-off Request**: Sets global "request turn off" flag
4. **Recovery**: Re-enables button and clears flags when power returns

### Fault Protection
1. **Offline Detection**: Automatic switching to charger for recovery
2. **Relay Refresh**: Continuous monitoring and correction of relay states
3. **Timing Enforcement**: Strict adherence to dwell and rest periods
4. **Thread Safety**: Mutex protection for all shared state

## Timing Constants

| Parameter | Value | Description |
|-----------|-------|-------------|
| Button Debounce | 50ms | Stable press detection |
| UART Poll Rate | 100ms | Measurement request frequency |
| Charger Rest | 7000ms | Mandatory rest between connections |
| Minimum Dwell | 10000ms | Minimum time on charger |
| Full Detection | 5000ms | Current below threshold duration |
| Full Current Threshold | 0.5A | Battery full detection limit |

## Sensor Specifications

### AC Measurements
- **Voltage Sensors**: ZMPT101B transformers
- **Current Sensors**: ACS712-20A Hall effect sensors
- **Sampling**: High-rate RMS calculation over configurable window
- **Calibration**: Automatic offset tracking with running mean

### DC Measurements
- **Voltage**: 5:1 resistive dividers
- **Current**: ACS712-20A Hall effect sensor (shared)
- **Processing**: Moving average for stable readings

### Display System
- **Type**: Adafruit SSD1305 OLED (128x32 pixels)
- **Interface**: SPI communication
- **Update Rate**: Configurable (typically 200ms)
- **Content**: Real-time display of all 7 measurements

## Error Handling

### Communication Errors
- **JSON Parse Failures**: Logged with error message
- **UART Timeouts**: Automatic retry mechanism
- **Rate Limiting**: Prevents communication flooding

### Hardware Errors
- **Sensor Failures**: Graceful degradation
- **Relay Malfunctions**: Automatic state correction
- **Power Supply Issues**: Emergency battery switching

## Development and Debugging

### Serial Monitoring
- **Baud Rate**: 115200
- **Debug Output**: Comprehensive state logging
- **Real-time Updates**: Continuous system status

### State Inspection
- Use `STATE` command for complete system overview
- Monitor measurement updates in real-time
- Track battery switching events and timing

### Calibration
- Sensor offsets automatically learned
- Voltage/current scaling factors configurable
- SOC thresholds adjustable via constants

## Future Enhancements

### Planned Features
1. **CAN Bus Integration**: Pins 4 and 5 reserved for future implementation
2. **Remote Monitoring**: Network connectivity for remote management
3. **Advanced Analytics**: Historical data logging and analysis
4. **Predictive Maintenance**: Battery health monitoring and alerts

### Expansion Possibilities
1. **Multiple Battery Banks**: Scalable to more than 2 batteries
2. **Solar Integration**: Renewable energy source management
3. **Load Management**: Intelligent load shedding during outages
4. **Mobile App**: Smartphone interface for monitoring and control

## Troubleshooting

### Common Issues

#### Battery Not Switching
- Check relay connections and polarity
- Verify timing constraints (dwell/rest periods)
- Ensure proper voltage readings

#### Communication Failures
- Verify UART wiring (RX/TX crossed)
- Check baud rate settings (115200)
- Monitor for JSON parsing errors

#### Incorrect Measurements
- Calibrate sensor offsets
- Verify scaling factors
- Check ADC reference voltage

#### Power Outage Not Detected
- Test pin 19 voltage levels
- Verify pull-up/pull-down configuration
- Check power good signal source

### Diagnostic Commands

```bash
# Check system state
STATE

# Monitor battery counters
COUNTS

# Adjust switching threshold
S=25
```

## Conclusion

This power system firmware provides a robust, intelligent battery backup solution with comprehensive monitoring and automatic management capabilities. The dual-core, multi-task architecture ensures reliable operation while the extensive safety features protect against various failure modes.

The modular design allows for easy maintenance and future enhancements, while the detailed logging and CLI interface facilitate debugging and system optimization.