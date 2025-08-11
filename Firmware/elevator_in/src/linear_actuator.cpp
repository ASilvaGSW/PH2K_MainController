#include "linear_actuator.h"

LinearActuator::LinearActuator(uint16_t motor_id, uint16_t speed, uint8_t acc)
    : motor_id(motor_id), angle_ratio(16384.0 / 360.0), speed(speed), acc(acc) {}

void LinearActuator::abs_mode(float angle, uint8_t* payload) {
    int32_t steps = round(angle * angle_ratio);
    if (steps < 0) {
        steps = 16777215 + steps;
    }
    uint8_t byte1 = 0xF5;
    uint8_t byte2 = (speed >> 8) & 0xFF;
    uint8_t byte3 = speed & 0xFF;
    uint8_t byte4 = acc & 0xFF;
    uint8_t byte5 = (steps >> 16) & 0xFF;
    uint8_t byte6 = (steps >> 8) & 0xFF;
    uint8_t byte7 = steps & 0xFF;
    uint8_t crc = (byte1 + byte2 + byte3 + byte4 + byte5 + byte6 + byte7 + motor_id) & 0xFF;

    payload[0] = byte1;
    payload[1] = byte2;
    payload[2] = byte3;
    payload[3] = byte4;
    payload[4] = byte5;
    payload[5] = byte6;
    payload[6] = byte7;
    payload[7] = crc;
}

void LinearActuator::check_connection(uint8_t* payload) {
    uint8_t byte1 = 0x30;
    uint8_t crc = (byte1 + motor_id) & 0xFF;
    payload[0] = byte1;
    payload[1] = crc;
    for (int i = 2; i < 8; i++) payload[i] = 0;
}

void LinearActuator::update_id(uint16_t new_id, uint8_t* payload) {
    uint8_t byte1 = 0x8B;
    uint8_t byte2 = (new_id >> 8) & 0xFF;
    uint8_t byte3 = new_id & 0xFF;
    uint8_t crc = (byte1 + byte2 + byte3 + motor_id) & 0xFF;
    payload[0] = byte1;
    payload[1] = byte2;
    payload[2] = byte3;
    payload[3] = crc;
    for (int i = 4; i < 8; i++) payload[i] = 0;
    motor_id = new_id;
}

void LinearActuator::speed_mode(bool dir, uint16_t speed, uint8_t acc, uint8_t* payload) {
    uint8_t byte1 = 0xF6;
    uint8_t byte2 = ((dir ? 1 : 0) << 7) | ((speed >> 8) & 0x0F);
    uint8_t byte3 = speed & 0xFF;
    uint8_t byte4 = acc;
    uint8_t crc = (byte1 + byte2 + byte3 + byte4 + motor_id) & 0xFF;
    payload[0] = byte1;
    payload[1] = byte2;
    payload[2] = byte3;
    payload[3] = byte4;
    payload[4] = crc;
    for (int i = 5; i < 8; i++) payload[i] = 0;
} 