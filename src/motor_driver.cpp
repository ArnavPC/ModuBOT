#include "motor_driver.h"

MotorDriver::MotorDriver(const MotorPins &pins) : pins_(pins), configured_(false) {}

bool MotorDriver::isConfigured() const {
    return configured_;
}

void MotorDriver::begin() {
    if (pins_.rPwm == PIN_UNASSIGNED || pins_.lPwm == PIN_UNASSIGNED ||
        pins_.rEn == PIN_UNASSIGNED || pins_.lEn == PIN_UNASSIGNED) {
        configured_ = false;
        return;
    }

    pinMode(pins_.rPwm, OUTPUT);
    pinMode(pins_.lPwm, OUTPUT);
    pinMode(pins_.rEn, OUTPUT);
    pinMode(pins_.lEn, OUTPUT);

    digitalWrite(pins_.rEn, HIGH);
    digitalWrite(pins_.lEn, HIGH);

    stop();
    configured_ = true;
}

void MotorDriver::applyPwm(uint8_t pin, uint8_t duty) {
    analogWrite(pin, duty);
}

void MotorDriver::stop() {
    if (!configured_) return;

    applyPwm(pins_.rPwm, 0);
    applyPwm(pins_.lPwm, 0);
}

void MotorDriver::drive(int16_t command) {
    if (!configured_) return;

    command = constrain(command, -255, 255);
    if (command == 0) {
        stop();
        return;
    }

    uint8_t duty = static_cast<uint8_t>(abs(command));

    if (command > 0) {
        applyPwm(pins_.rPwm, duty);
        applyPwm(pins_.lPwm, 0);
    } else {
        applyPwm(pins_.rPwm, 0);
        applyPwm(pins_.lPwm, duty);
    }
}
