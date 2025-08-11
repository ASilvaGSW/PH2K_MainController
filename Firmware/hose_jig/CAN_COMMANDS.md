# Hose Jig CAN Commands Reference

This document describes all the available CAN commands for the Hose Jig controller.

## Basic Commands

### 0x01 - Reset Microcontroller
- **Description**: Resets the microcontroller
- **Parameters**: None
- **Response**: `[0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]`

### 0x02 - Ping
- **Description**: Basic ping command to check device responsiveness
- **Parameters**: None
- **Response**: `[0x02, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]`

## Actuator Control

### 0x03 - Home Actuator
- **Description**: Moves the actuator to the home position
- **Parameters**: None
- **Response**: `[0x03, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]`
  - status: 0x01 = Success, 0x04 = Error

### 0x04 - Move Actuator to Absolute Position
- **Description**: Moves the actuator to a specified absolute position
- **Parameters**:
  - data[1]: Position high byte
  - data[2]: Position low byte
  - data[3]: Orientation (0 = positive, 1 = negative)
- **Response**: `[0x04, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]`
  - status: 0x01 = Success, 0x04 = Error

## Servo Control

### 0x05 - Move Servos to Open Position
- **Description**: Moves all servos to their open position
- **Parameters**: None
- **Response**: `[0x05, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]`

### 0x06 - Move Servos to Close Position
- **Description**: Moves all servos to their close position
- **Parameters**: None
- **Response**: `[0x06, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]`

## Counter Operations

### 0x08 - Read Servo Movement Counter
- **Description**: Reads the total number of servo movements
- **Parameters**: None
- **Response**: `[0x08, 0x01, high_byte, low_byte, 0x00, 0x00, 0x00, 0x00]`

### 0x09 - Reset Servo Movement Counter
- **Description**: Resets the servo movement counter to zero
- **Parameters**: None
- **Response**: `[0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]`

### 0x0A - Move Servos to Absolute Position
- **Description**: Moves all servos to the specified absolute position
- **Parameters**:
  - data[1]: Position value (0-255)
- **Response**: `[0x0A, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]`
  - status: 0x01 = Success

### 0x0B - Move Actuator to Insertion Position
- **Description**: Moves the actuator to the predefined insertion position
- **Parameters**: None
- **Response**: `[0x0B, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]`
  - status: 0x01 = Success, 0x04 = Error

### 0x0C - Move Actuator to Deliver Position
- **Description**: Moves the actuator to the predefined deliver position
- **Parameters**: None
- **Response**: `[0x0C, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]`
  - status: 0x01 = Success, 0x04 = Error
### 0x0D - Read Actuator Movement Counter
- **Description**: Reads the total number of actuator movements
- **Parameters**: None
- **Response**: `[0x0D, 0x01, high_byte, low_byte, 0x00, 0x00, 0x00, 0x00]`

### 0x0E - Reset Actuator Movement Counter
- **Description**: Resets the actuator movement counter to zero
- **Parameters**: None
- **Response**: `[0x0E, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]`

## Configuration Settings

### 0x10 - Set SERVO_OPEN_ANGLE
- **Description**: Sets the servo open angle
- **Parameters**:
  - data[1-2]: 16-bit angle value (high byte, low byte)
- **Response**: `[0x10, 0x01, high_byte, low_byte, 0x00, 0x00, 0x00, 0x00]`

### 0x11 - Set SERVO_CLOSE_ANGLE
- **Description**: Sets the servo close angle
- **Parameters**:
  - data[1-2]: 16-bit angle value (high byte, low byte)
- **Response**: `[0x11, 0x01, high_byte, low_byte, 0x00, 0x00, 0x00, 0x00]`

### 0x12 - Set ACTUATOR_DELIVER_POSITION
- **Description**: Sets the actuator deliver position
- **Parameters**:
  - data[1-2]: 16-bit position value (high byte, low byte)
- **Response**: `[0x12, 0x01, high_byte, low_byte, 0x00, 0x00, 0x00, 0x00]`

### 0x13 - Set ACTUATOR_INSERTION_POSITION
- **Description**: Sets the actuator insertion position
- **Parameters**:
  - data[1-2]: 16-bit position value (high byte, low byte)
- **Response**: `[0x13, 0x01, high_byte, low_byte, 0x00, 0x00, 0x00, 0x00]`

### 0x14 - Read SERVO_OPEN_ANGLE
- **Description**: Reads the current servo open angle
- **Parameters**: None
- **Response**: `[0x14, 0x01, high_byte, low_byte, 0x00, 0x00, 0x00, 0x00]`

### 0x15 - Read SERVO_CLOSE_ANGLE
- **Description**: Reads the current servo close angle
- **Parameters**: None
- **Response**: `[0x15, 0x01, high_byte, low_byte, 0x00, 0x00, 0x00, 0x00]`

### 0x16 - Read ACTUATOR_DELIVER_POSITION
- **Description**: Reads the current actuator deliver position
- **Parameters**: None
- **Response**: `[0x16, 0x01, high_byte, low_byte, 0x00, 0x00, 0x00, 0x00]`

### 0x17 - Read ACTUATOR_INSERTION_POSITION
- **Description**: Reads the current actuator insertion position
- **Parameters**: None
- **Response**: `[0x17, 0x01, high_byte, low_byte, 0x00, 0x00, 0x00, 0x00]`

## System Commands

### 0xFF - Power Off
- **Description**: Moves all components to home position and prepares for power off
- **Parameters**: None
- **Response**: 
  1. Initial: `[0xFF, status, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]`
  2. Final: `[0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]`
  - status: 0x01 = Success, 0x04 = Error
