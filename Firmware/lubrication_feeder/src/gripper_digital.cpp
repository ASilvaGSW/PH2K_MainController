#include "gripper_digital.h"

GripperDigital::GripperDigital(int in0, int in1, int in2)
    : pin_in0(in0), pin_in1(in1), pin_in2(in2), fuerza(4) {
    pinMode(pin_in0, OUTPUT);
    pinMode(pin_in1, OUTPUT);
    pinMode(pin_in2, OUTPUT);
}

void GripperDigital::open() {
    // Comando 0: Open to limit, strength 100%
    digitalWrite(pin_in0, LOW); // IN0
    digitalWrite(pin_in1, LOW); // IN1
    digitalWrite(pin_in2, LOW); // IN2
}

void GripperDigital::close() {
    // Usa la tabla para 4-7
    switch (fuerza) {
        case 4: // 1 0 0
            digitalWrite(pin_in0, HIGH);
            digitalWrite(pin_in1, LOW);
            digitalWrite(pin_in2, LOW);
            break;
        case 5: // 1 0 1
            digitalWrite(pin_in0, HIGH);
            digitalWrite(pin_in1, LOW);
            digitalWrite(pin_in2, HIGH);
            break;
        case 6: // 1 1 0
            digitalWrite(pin_in0, HIGH);
            digitalWrite(pin_in1, HIGH);
            digitalWrite(pin_in2, LOW);
            break;
        case 7: // 1 1 1
            digitalWrite(pin_in0, HIGH);
            digitalWrite(pin_in1, HIGH);
            digitalWrite(pin_in2, HIGH);
            break;
        default: // fallback a 4
            digitalWrite(pin_in0, HIGH);
            digitalWrite(pin_in1, LOW);
            digitalWrite(pin_in2, LOW);
            break;
    }
}

void GripperDigital::setFuerza(uint8_t f) {
    if (f >= 4 && f <= 7) {
        fuerza = f;
    }
}

uint8_t GripperDigital::getFuerza() const {
    return fuerza;
} 