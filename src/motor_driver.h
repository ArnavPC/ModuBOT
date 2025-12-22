#pragma once

#include <Arduino.h>
#include "pins.h"

class MotorDriver {
public:
    explicit MotorDriver(const MotorPins &pins);
    void begin();
    void stop();
    void drive(int16_t command); // command range: -255..255
    bool isConfigured() const;

private:
    MotorPins pins_;
    bool configured_;
    void applyPwm(uint8_t pin, uint8_t duty);
};
