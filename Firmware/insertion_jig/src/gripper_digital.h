#ifndef GRIPPER_DIGITAL_H
#define GRIPPER_DIGITAL_H

#include <Arduino.h>

class GripperDigital {
public:
    int pin_in0, pin_in1, pin_in2;
    uint8_t fuerza;
    GripperDigital(int in0, int in1, int in2);
    void open();
    void close();
    void setFuerza(uint8_t f);
    uint8_t getFuerza() const;
};

#endif // GRIPPER_DIGITAL_H 