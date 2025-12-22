#pragma once

#include <Arduino.h>
#include <IBusBM.h>
#include "pins.h"

struct RCState {
    int16_t channels[16];
    bool failsafe;
    bool frameLost;
};

class RCInput {
public:
    RCInput();
    void begin();
    // Call frequently to poll SBUS data.
    void update();
    bool hasNewFrame() const;
    RCState consumeState();
    void clearNewFlag();

private:
    IBusBM ibus_;
    RCState state_;
    bool newFrame_;
};
