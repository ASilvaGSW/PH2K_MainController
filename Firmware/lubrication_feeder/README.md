# Lubrication Feeder Valve System Documentation

## Table of Contents
1. [Introduction](#introduction)
2. [Valve Components and Purpose](#valve-components-and-purpose)
3. [How Valves Operate](#how-valves-operate)
4. [System Architecture](#system-architecture)
5. [Operation Instructions](#operation-instructions)
6. [Safety Precautions](#safety-precautions)
7. [Troubleshooting Guide](#troubleshooting-guide)
8. [Maintenance Guidelines](#maintenance-guidelines)
9. [Technical Specifications](#technical-specifications)
10. [Real-World Applications](#real-world-applications)

## Introduction

### What are Valves?
Valves are mechanical devices that control the flow of fluids (liquids, gases, or slurries) by opening, closing, or partially obstructing various passageways. In our lubrication feeder system, valves are essential components that precisely control the distribution of lubricant to different parts of machinery.

### Purpose in Lubrication Systems
The valve system in this project serves several critical functions:
- **Precise Fluid Control**: Regulates the exact amount of lubricant delivered
- **Timing Control**: Ensures lubricant is delivered at the right moment
- **System Protection**: Prevents over-lubrication and contamination
- **Automation**: Enables automated lubrication cycles without human intervention

## Valve Components and Purpose

### Hardware Components

#### Valve 1 (Primary Lubrication Valve)
- **Location**: Connected to GPIO Pin 2
- **Function**: Controls primary lubricant flow
- **Type**: Solenoid-operated valve
- **Default State**: Normally closed (LOW)

#### Valve 2 (Secondary Lubrication Valve)
- **Location**: Connected to GPIO Pin 4
- **Function**: Controls secondary lubricant flow or backup system
- **Type**: Solenoid-operated valve
- **Default State**: Normally closed (LOW)

#### Valve 3 (Tertiary Lubrication Valve)
- **Location**: Connected to GPIO Pin 5
- **Function**: Controls tertiary lubricant flow
- **Type**: Solenoid-operated valve
- **Default State**: Normally closed (LOW)

#### Valve 4 (Quaternary Lubrication Valve)
- **Location**: Connected to GPIO Pin 18
- **Function**: Controls quaternary lubricant flow
- **Type**: Solenoid-operated valve
- **Default State**: Normally closed (LOW)

#### Valve 5 (Fifth Lubrication Valve)
- **Location**: Connected to GPIO Pin 19
- **Function**: Controls fifth lubricant flow
- **Type**: Solenoid-operated valve
- **Default State**: Normally closed (LOW)

#### Valve 6 (Sixth Lubrication Valve)
- **Location**: Connected to GPIO Pin 21
- **Function**: Controls sixth lubricant flow
- **Type**: Solenoid-operated valve
- **Default State**: Normally closed (LOW)

#### Valve 7 (Seventh Lubrication Valve)
- **Location**: Connected to GPIO Pin 22
- **Function**: Controls seventh lubricant flow
- **Type**: Solenoid-operated valve
- **Default State**: Normally closed (LOW)

#### Valve 8 (Eighth Lubrication Valve)
- **Location**: Connected to GPIO Pin 23
- **Function**: Controls eighth lubricant flow
- **Type**: Solenoid-operated valve
- **Default State**: Normally closed (LOW)

#### Linear Potentiometer (SoftPot)
- **Location**: Connected to GPIO Pin 32 (Analog Input)
- **Model**: Spectra Symbol SoftPot (1.969 inch variant with male pin connector)
- **Function**: Provides position feedback for conditional actuator control
- **Accuracy**: ±1% linearity
- **Activation Threshold**: >100 (analog reading)
- **Purpose**: Enables conditional dual-movement functionality

#### IR Break Beam Sensor (Optical Sensor)
- **Location**: Connected to GPIO Pin 33 (Digital Input)
- **Model**: IR Break Beam Sensors with Premium Wire Header Ends - 3mm LEDs
- **Function**: Detects hose presence and position
- **Operation**: Active LOW (beam broken = LOW signal = hose detected)
- **Configuration**: INPUT_PULLUP (internal pull-up resistor enabled)
- **Purpose**: Provides feedback on hose positioning for quality control

#### Alcohol Level Sensor 1 (2/3 Tank Level)
- **Location**: Connected to GPIO Pin 39 (Digital Input)
- **Function**: Detects alcohol level at 2/3 tank capacity
- **Type**: Digital level sensor
- **Operation**: Active HIGH (HIGH = liquid present, LOW = no liquid)
- **Configuration**: INPUT (standard digital input)
- **Purpose**: Monitors upper tank level for full/medium detection

#### Alcohol Level Sensor 2 (1/3 Tank Level)
- **Location**: Connected to GPIO Pin 36 (Digital Input)
- **Function**: Detects alcohol level at 1/3 tank capacity
- **Type**: Digital level sensor
- **Operation**: Active HIGH (HIGH = liquid present, LOW = no liquid)
- **Configuration**: INPUT (standard digital input)
- **Purpose**: Monitors lower tank level for medium/empty detection

#### Hose Holder Servo
- **Location**: Connected to GPIO Pin 14 (PWM Output)
- **Function**: Controls hose holder position (open/close)
- **Type**: Standard servo motor (50Hz PWM)
- **Operation**: 0° = closed position, 90° = open position
- **Configuration**: PWM output with ESP32Servo library
- **Purpose**: Automated hose positioning and securing mechanism

#### Electromagnet Control
- **Location**: Connected to GPIO Pin 17 (Digital Output)
- **Function**: Controls electromagnet via MOSFET for attachment detection
- **Type**: Digital control signal to MOSFET driver
- **Operation**: HIGH = electromagnet ON, LOW = electromagnet OFF
- **Configuration**: OUTPUT (digital output pin)
- **Purpose**: Electromagnet control and attachment status verification

### Control System Components

```
┌─────────────────┐    ┌──────────────┐    ┌─────────────┐
│   ESP32 MCU     │───▶│  GPIO 27     │───▶│ Valve 1     │
│                 │───▶│  GPIO 32     │───▶│ Valve 2     │
│                 │◀───│  GPIO 33     │◀───│ Potentiometer│
│                 │◀───│  GPIO 16     │◀───│ IR Sensor   │
│                 │◀───│  GPIO 39     │◀───│ Level Sensor 1│
│                 │◀───│  GPIO 36     │◀───│ Level Sensor 2│
│                 │───▶│  GPIO 14     │───▶│ Hose Holder │
│                 │───▶│  GPIO 17     │───▶│ Electromagnet │
│                 │───▶│  GPIO 4      │───▶│ CAN TX      │
│                 │◀───│  GPIO 5      │◀───│ CAN RX      │
└─────────────────┘    └──────────────┘    └─────────────┘
         │                                         │
         ▼                                         ▼
┌─────────────────┐                      ┌─────────────┐
│  CAN Bus        │                      │  Lubricant  │
│  Communication  │                      │  Flow       │
└─────────────────┘                      └─────────────┘
```

## How Valves Operate

### Basic Operation Principle

1. **Electrical Signal**: The ESP32 microcontroller sends a HIGH signal to the valve's GPIO pin
2. **Solenoid Activation**: The electrical signal energizes the solenoid coil
3. **Mechanical Movement**: The energized coil creates a magnetic field that moves the valve plunger
4. **Flow Control**: The plunger movement opens the valve, allowing lubricant to flow
5. **Deactivation**: When the signal goes LOW, the spring returns the plunger to the closed position

### Timing Control
Each valve can be activated for a specific duration:
- **Minimum Duration**: 1ms (theoretical)
- **Default Duration**: 1000ms (1 second)
- **Maximum Duration**: 65,535ms (approximately 65 seconds)
- **Custom Duration**: Configurable via CAN bus commands

### Valve State Diagram

```
    ┌─────────────┐
    │   CLOSED    │ ◄─── Default State
    │  (LOW/0V)   │
    └──────┬──────┘
           │ Activation Command
           ▼
    ┌─────────────┐
    │    OPEN     │
    │  (HIGH/5V)  │
    └──────┬──────┘
           │ Timer Expires
           ▼
    ┌─────────────┐
    │   CLOSED    │
    │  (LOW/0V)   │
    └─────────────┘
```

## System Architecture

### Communication Protocol
The valve system uses CAN bus communication for control:

- **Device CAN ID**: 0x191 (receives commands)
- **Response CAN ID**: 0x591 (sends status)
- **Command Format**: 8-byte CAN frames

#### Acknowledgment Messages
All commands now send an acknowledgment message before executing their main functionality:
- **Format**: [0xAA, function_code, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
- **Purpose**: Confirms command receipt and indicates start of execution
- **Timing**: Sent immediately upon receiving a valid command
- **Example**: For command 0x03 (Valve 1), acknowledgment is [0xAA, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

**Debug Control**: The `enableAckMessages` variable can be set to `false` to disable acknowledgment messages during debugging. This is useful when you want to reduce CAN bus traffic or focus on main command responses only. Set `enableAckMessages = true` (default) to enable acknowledgments, or `enableAckMessages = false` to disable them.

- **Status Codes**:
  - 0x01: Success
  - 0x02: Failure
  - 0x03: Timeout
  - 0x04: Network/Communication Error
  - 0xFF: Unknown Command

### Memory Management
The system tracks valve usage with persistent counters:
- **EEPROM Storage**: Counters saved to flash memory
- **Valve 1 Counter**: Address 12 in EEPROM
- **Valve 2 Counter**: Address 16 in EEPROM
- **Auto-increment**: Counters increase with each activation

## Operation Instructions

### Basic Valve Activation

#### Method 1: Using CAN Commands

**Activate Valve 1:**
```
CAN ID: 0x191
Command: 0x03
Data: [0x03, duration_high, duration_low, 0x00, 0x00, 0x00, 0x00, 0x00]
```

**Activate Valve 2:**
```
CAN ID: 0x191
Command: 0x04
Data: [0x04, duration_high, duration_low, 0x00, 0x00, 0x00, 0x00, 0x00]
```

#### Method 2: Default Duration (1 second)
```
CAN ID: 0x191
Data: [0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]  // Valve 1
Data: [0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]  // Valve 2
```

### Counter Management

**Read Valve Counters:**
- Valve 1 Counter: Send command 0x0A
- Valve 2 Counter: Send command 0x0B

**Reset Valve Counters:**
- Reset Valve 1: Send command 0x0C
- Reset Valve 2: Send command 0x0D

### Step-by-Step Operation Example

1. **System Initialization**
   - Power on the ESP32 controller
   - Wait for "CAN system ready" message
   - Verify both CAN buses are operational

2. **Valve Activation Sequence**
   ```
   Step 1: Send activation command via CAN
   Step 2: System processes command
   Step 3: GPIO pin goes HIGH
   Step 4: Valve opens for specified duration
   Step 5: GPIO pin returns to LOW
   Step 6: Valve closes
   Step 7: Counter increments
   Step 8: Status response sent
   ```

3. **Monitoring and Verification**
   - Check serial monitor for confirmation messages
   - Verify valve counter incrementation
   - Monitor CAN bus for status responses

## Safety Precautions

### ⚠️ Critical Safety Guidelines

#### Electrical Safety
- **Power Isolation**: Always disconnect power before maintenance
- **Voltage Verification**: Ensure 5V supply is stable and clean
- **Ground Connection**: Verify proper system grounding
- **Wire Inspection**: Check for damaged or exposed wiring

#### Fluid Safety
- **Lubricant Compatibility**: Use only approved lubricants
- **Pressure Limits**: Do not exceed maximum system pressure (check valve specifications)
- **Leak Prevention**: Inspect all connections for leaks before operation
- **Containment**: Have spill containment measures in place

#### Operational Safety
- **Emergency Stop**: Implement emergency shutdown procedures
- **Pressure Relief**: Ensure pressure relief valves are functional
- **Personal Protective Equipment**: Wear appropriate PPE when handling lubricants
- **Ventilation**: Ensure adequate ventilation in enclosed spaces

### Best Practices

1. **Pre-Operation Checklist**
   - [ ] Verify power supply stability
   - [ ] Check all electrical connections
   - [ ] Inspect valve mounting and alignment
   - [ ] Test emergency stop functionality
   - [ ] Verify lubricant levels and quality

2. **During Operation**
   - Monitor system pressure continuously
   - Watch for unusual noises or vibrations
   - Check for leaks at regular intervals
   - Maintain communication with control system

3. **Post-Operation**
   - Record operation hours and cycles
   - Check valve counter readings
   - Inspect for wear or damage
   - Clean external surfaces

## Troubleshooting Guide

### Common Issues and Solutions

#### Issue 1: Valve Not Responding
**Symptoms:**
- No valve activation despite command
- GPIO pin remains LOW
- No counter increment

**Possible Causes & Solutions:**
1. **Power Supply Issue**
   - Check 5V supply voltage
   - Verify current capacity (minimum 500mA per valve)
   - Solution: Replace or upgrade power supply

2. **Wiring Problem**
   - Inspect connections to GPIO pins 27 and 32
   - Check for loose or corroded connections
   - Solution: Clean and tighten connections

3. **Software Issue**
   - Verify correct CAN ID (0x191)
   - Check command format
   - Solution: Review command structure and timing

#### Issue 2: Valve Stuck Open
**Symptoms:**
- Continuous lubricant flow
- GPIO pin stuck HIGH
- System unresponsive

**Solutions:**
1. **Immediate Action**: Cut power to system
2. **Check**: Solenoid coil for overheating
3. **Inspect**: Valve plunger for mechanical obstruction
4. **Replace**: Faulty solenoid if necessary

#### Issue 3: Inconsistent Timing
**Symptoms:**
- Valve duration doesn't match command
- Erratic opening/closing behavior

**Solutions:**
1. **Check**: System clock accuracy
2. **Verify**: Timer interrupt functionality
3. **Update**: Firmware if necessary
4. **Calibrate**: Timing parameters

#### Issue 4: Communication Errors
**Symptoms:**
- CAN bus timeout errors
- Status code 0x04 (Network Error)
- No response to commands

**Solutions:**
1. **Check**: CAN bus wiring and termination
2. **Verify**: Baud rate settings (125 kbps)
3. **Inspect**: MCP2515 module connections
4. **Test**: With CAN bus analyzer tool

### Diagnostic Commands

**System Health Check:**
```
Command 0x02: Heartbeat (verifies system responsiveness)
Expected Response: [0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
```

**Counter Verification:**
```
Command 0x0A: Read Valve 1 Counter
Command 0x0B: Read Valve 2 Counter
```

**Advanced Movement Control:**

**Note**: All commands listed below send an acknowledgment message (0xAA + function_code) immediately upon receipt, followed by the main response after execution.

```
Command 0x05: Move Feeder (Speed Mode)
Format: [0x05, speed_high, speed_low, direction, acceleration, 0x00, 0x00, 0x00]
Function: Moves main feeder using speed mode with specified speed, direction, and acceleration
Response: [0x05, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

Command 0x13: Move Pre-feeder (Speed Mode)
Format: [0x13, speed_high, speed_low, direction, acceleration, 0x00, 0x00, 0x00]
Function: Moves pre-feeder using speed mode with specified speed, direction, and acceleration
Response: [0x13, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

Command 0x14: Conditional Dual Movement
Format: [0x14, speed_high, speed_low, direction, acceleration, 0x00, 0x00, 0x00]
Function: Moves feeder in speed mode; also moves pre-feeder if linear potentiometer is activated (>100)
         Pre-feeder direction is determined by potentiometer value: >512 = forward, 100-512 = reverse
Response: [0x14, feeder_status, prefeeder_status, 0x00, 0x00, 0x00, 0x00, 0x00]

Command 0x15: Check Hose Position
Format: [0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
Function: Checks if hose is in position using IR break beam sensor
Response: [0x15, 0x01, hose_status, 0x00, 0x00, 0x00, 0x00, 0x00]
         hose_status: 0x01 = hose in position (beam broken), 0x00 = hose not in position (beam intact)

Command 0x16: Check Alcohol Level
Format: [0x16, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
Function: Monitors alcohol tank level using two digital level sensors at 2/3 and 1/3 positions
Response: [0x16, 0x01, level_status, sensor1_state, sensor2_state, 0x00, 0x00, 0x00]
         level_status: 0x03 = FULL (both sensors detect liquid), 0x02 = MEDIUM (only 1/3 sensor detects liquid), 
                      0x00 = EMPTY (no sensors detect liquid), 0xFF = ERROR (invalid sensor state)
         sensor1_state: 0x01 = liquid present at 2/3 level, 0x00 = no liquid at 2/3 level
         sensor2_state: 0x01 = liquid present at 1/3 level, 0x00 = no liquid at 1/3 level

Command 0x17: Open Hose Holder
Format: [0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
Function: Opens the hose holder servo to 90 degrees (open position)
Response: [0x17, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

Command 0x18: Close Hose Holder
Format: [0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
Function: Closes the hose holder servo to 0 degrees (closed position)
Response: [0x18, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

Command 0x19: Attach Electromagnet
Format: [0x19, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
Function: Activates electromagnet (HIGH)
Response: [0x19, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

Command 0x1A: Detach Electromagnet
Format: [0x1A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
Function: Deactivates electromagnet (LOW)
Response: [0x1A, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

Command 0x1A: Detach Electromagnet
Format: [0x1A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
Function: Deactivates electromagnet (LOW)
Response: [0x1A, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

Command 0x1B: Read Servo Counter
Format: [0x1B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
Function: Returns current servo counter value
Response: [0x1B, 0x01, counter_high, counter_low, 0x00, 0x00, 0x00, 0x00]
         counter_high: High byte of 16-bit counter value
         counter_low: Low byte of 16-bit counter value

Command 0x1C: Reset Servo Counter
Format: [0x1C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
Function: Resets servo counter to zero and saves to EEPROM
Response: [0x1C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

Command 0x1D: Acknowledge Message
Format: [0x1D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
Function: Sends acknowledge response with 0xAA in first byte and received function code in second byte
Response: [0xAA, function_code, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
         function_code: The original command code that was received (0x1D in this case)

Parameters:
- speed_high, speed_low: 16-bit speed value (0-65535)
- direction: 0 = forward, 1 = reverse
- acceleration: 8-bit acceleration value (0-255, default 50 if 0)
- counter_high, counter_low: 16-bit counter value
- function_code: Original command code being acknowledged
```

## Maintenance Guidelines

### Daily Maintenance (5 minutes)
- [ ] Visual inspection for leaks
- [ ] Check system pressure readings
- [ ] Verify valve counter increments
- [ ] Monitor serial output for errors

### Weekly Maintenance (15 minutes)
- [ ] Clean valve exterior surfaces
- [ ] Check electrical connections
- [ ] Test emergency stop function
- [ ] Backup EEPROM counter data
- [ ] Inspect lubricant quality and levels

### Monthly Maintenance (30 minutes)
- [ ] Calibrate valve timing
- [ ] Test all CAN bus commands
- [ ] Inspect wiring for wear
- [ ] Update maintenance log
- [ ] Performance analysis review

### Quarterly Maintenance (1 hour)
- [ ] Disassemble and clean valve internals
- [ ] Replace O-rings and seals
- [ ] Lubricate moving parts
- [ ] Electrical continuity testing
- [ ] Firmware update check
- [ ] System performance optimization

### Annual Maintenance (2 hours)
- [ ] Complete valve overhaul
- [ ] Replace wear components
- [ ] Pressure test entire system
- [ ] Calibrate all sensors
- [ ] Update documentation
- [ ] Training refresh for operators

### Maintenance Schedule Template

| Task | Frequency | Last Done | Next Due | Notes |
|------|-----------|-----------|----------|---------|
| Visual Inspection | Daily | | | |
| Leak Check | Daily | | | |
| Electrical Check | Weekly | | | |
| Valve Cleaning | Monthly | | | |
| Seal Replacement | Quarterly | | | |
| Complete Overhaul | Annual | | | |

## Technical Specifications

### Electrical Specifications
- **Operating Voltage**: 5V DC
- **Current Consumption**: 200-500mA per valve
- **Control Signal**: Digital GPIO (0V/5V)
- **Response Time**: <10ms
- **Duty Cycle**: 100% (continuous operation capable)

### Mechanical Specifications
- **Operating Pressure**: 0-10 bar (check valve datasheet)
- **Temperature Range**: -10°C to +60°C
- **Flow Rate**: Depends on valve size and pressure
- **Connection Type**: Standard pipe fittings
- **Mounting**: Panel or manifold mount

### Communication Specifications
- **Protocol**: CAN 2.0B
- **Baud Rate**: 125 kbps
- **Message Format**: Standard 11-bit identifier
- **Data Length**: 8 bytes
- **Error Detection**: CRC and acknowledgment

## Real-World Applications

### Industrial Automation
**Scenario**: Automated assembly line lubrication
- **Application**: Precise lubricant application to moving parts
- **Benefits**: Consistent lubrication, reduced waste, improved reliability
- **Example**: Automotive manufacturing conveyor systems

### Maintenance Systems
**Scenario**: Scheduled equipment lubrication
- **Application**: Timed lubrication of bearings and gears
- **Benefits**: Extended equipment life, reduced downtime
- **Example**: Wind turbine gearbox lubrication

### Quality Control
**Scenario**: Controlled lubricant application in testing
- **Application**: Precise amounts for product testing
- **Benefits**: Repeatable test conditions, accurate results
- **Example**: Bearing life testing laboratories

### Research and Development
**Scenario**: Experimental lubrication studies
- **Application**: Variable timing and quantity testing
- **Benefits**: Data collection, parameter optimization
- **Example**: Tribology research facilities

## Conclusion

This valve system provides precise, automated control of lubricant flow with comprehensive monitoring and safety features. Regular maintenance and proper operation ensure reliable performance and extended system life.

For technical support or additional information, refer to the source code comments in `lubrication_feeder.ino` or contact the system administrator.

---

**Document Version**: 1.0  
**Last Updated**: 2024  
**Author**: Lubrication System Documentation Team  
**Review Date**: Annual review required