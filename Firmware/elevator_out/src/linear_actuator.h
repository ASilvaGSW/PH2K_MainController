#ifndef LINEAR_ACTUATOR_H
#define LINEAR_ACTUATOR_H

#include <Arduino.h>

class LinearActuator {
public:
    uint16_t motor_id;
    float angle_ratio;
    uint16_t speed;
    uint8_t acc;

    LinearActuator(uint16_t motor_id, uint16_t speed = 1500, uint8_t acc = 236);

    // Add definition variables instead of fixed values

    void abs_mode(float angle, uint8_t* payload);
    void check_connection(uint8_t* payload);
    void update_id(uint16_t new_id, uint8_t* payload);
    void speed_mode(bool dir, uint16_t speed, uint8_t acc, uint8_t* payload);
    void go_home(uint8_t* payload);
    void relative_move_with_speed_control(float angle, uint16_t local_speed, uint8_t acc, uint8_t* payload);
};

#endif // LINEAR_ACTUATOR_H