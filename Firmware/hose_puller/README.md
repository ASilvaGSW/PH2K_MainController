# Hose Puller Controller

This firmware controls a hose pulling robotic system using an ESP32 with dual CAN bus support. It manages two linear actuators (Y and Z axes), a digital gripper, and a stepper motor for precise hose manipulation.

## Hardware Connections

### ESP32 Pin Configuration

#### CAN Bus 1 (TWAI - ESP32 Integrated)
- **TX**: GPIO4 (CAN0_TX)
- **RX**: GPIO5 (CAN0_RX)

#### CAN Bus 2 (MCP2515)
- **CS**: GPIO26
- **INT**: GPIO25
- **SPI**: Uses default ESP32 SPI pins (GPIO18, 19, 23)

#### Stepper Motor
- **STEP**: GPIO22
- **DIR**: GPIO21
- **ENABLE**: GPIO20 (active LOW)
- **Configuration**: 200 steps/revolution, 1000 steps/s max speed, 500 steps/s² acceleration

#### Digital Gripper
- **PWM Pin**: GPIO32
- **Open Pin**: GPIO33
- **Close Pin**: GPIO14

#### Linear Actuators
- **Y-Axis**: CAN ID 0x2CB
- **Z-Axis**: CAN ID 0x2CC

#### Important Pin Notes
- **GPIO6-11**: Connected to SPI flash, not recommended for general use
- **GPIO2**: May have boot issues
- **GPIO15**: May have upload issues
- **GPIO35-39**: Analog pins, not recommended for digital signals
- **SPI Reserved**: GPIO18 (MOSI), GPIO19 (MISO), GPIO23 (SCK)

## CAN Protocol

### Device Addressing
- **Device CAN ID**: 0x192
- **Response CAN ID**: 0x592

### Command Reference

| Command | Description | Data Bytes | Response |
|---------|-------------|------------|----------|
| 0x01 | Reset microcontroller | - | - |
| 0x02 | Heartbeat | - | Status (0x01) |
| 0x03 | Home actuators (Y and Z) | - | Status (0x01=OK, 0x02=FAIL, 0x04=NO LOCAL NETWORK) |
| 0x04 | Move Y actuator to absolute position | 4 bytes (int32_t) | Status (0x01=OK, 0x02=FAIL) |
| 0x05 | Move Z actuator to absolute position | 4 bytes (int32_t) | Status (0x01=OK, 0x02=FAIL) |
| 0x06 | Read Z actuator movement counter | - | 4 bytes (uint32_t) |
| 0x07 | Read Y actuator movement counter | - | 4 bytes (uint32_t) |
| 0x08 | Reset Y movement counter | - | Status (0x01) |
| 0x09 | Reset Z movement counter | - | Status (0x01) |
| 0x0A | Read stepper movement counter | - | 4 bytes (uint32_t) |
| 0x0B | Move stepper to position | 4 bytes (int32_t) | Status (0x01=OK) |
| 0x0C | Home stepper | - | Status (0x01=OK) |
| 0x0D | Open gripper | - | Status (0x01=OK) |
| 0x0E | Close gripper | - | Status (0x01=OK) |
| 0x0F | Set gripper force | 1 byte (0-255) | Status (0x01=OK) |
| 0x10 | Read gripper movement counter | - | 4 bytes (uint32_t) |
| 0x11 | Reset gripper movement counter | - | Status (0x01) |
| 0x12 | Reset stepper movement counter | - | Status (0x01) |
| 0xFF | Power off, home all axes | - | - |

## EEPROM Usage
- **Size**: 8 bytes
- **Address 0-3**: Y-axis movement counter
- **Address 4-7**: Z-axis movement counter

## Dependencies
- ESP32-TWAI-CAN
- mcp_can
- SPI
- ESP32Servo
- AccelStepper
- FreeRTOS
- EEPROM (ESP32)

## Status Codes
- **0x01**: OK
- **0x02**: FAIL
- **0x03**: TIMEOUT
- **0x04**: NO LOCAL NETWORK

## Author
Alan Silva  
asilva@gswiring.com  
Date: 2025-07-25
