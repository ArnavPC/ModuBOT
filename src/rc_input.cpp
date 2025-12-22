#include "rc_input.h"
#include <cstring>

RCInput::RCInput() : newFrame_(false) {
    memset(&state_, 0, sizeof(state_));
}

void RCInput::begin() {
    // Ensure UART is configured for i-BUS (115200 8N1)
    SBUS_SERIAL.begin(SBUS_BAUD);
    ibus_.begin(SBUS_SERIAL);
}

void RCInput::update() {
    ibus_.loop();
    for (int i = 0; i < 16; ++i) {
        state_.channels[i] = ibus_.readChannel(i);
    }
    // Simple connectivity/failsafe heuristic: many iBUS drivers return 0 when disconnected.
    state_.failsafe = (state_.channels[0] == 0);
    state_.frameLost = state_.failsafe;
    newFrame_ = true;
}

bool RCInput::hasNewFrame() const {
    return newFrame_;
}

RCState RCInput::consumeState() {
    newFrame_ = false;
    return state_;
}

void RCInput::clearNewFlag() {
    newFrame_ = false;
}
