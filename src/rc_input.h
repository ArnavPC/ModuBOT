#pragma once

#include <Arduino.h>
#include "pins.h"

struct RCState {
    int16_t channels[2];  // [0] = throttle, [1] = steering
    bool failsafe;
    bool frameLost;
};

class RCInput {
public:
    RCInput();
    void begin();
    // Call frequently to poll UART data.
    void update();
    bool hasNewFrame() const;
    RCState consumeState();
    void clearNewFlag();

private:
    RCState state_;
    bool newFrame_;
    uint32_t lastPacketTime_;
    
    // Packet parsing state machine
    enum ParseState { WAIT_START, READ_T_HIGH, READ_T_LOW, READ_S_HIGH, READ_S_LOW, WAIT_END };
    ParseState parseState_;
    uint8_t packetBuffer_[4];  // T_H, T_L, S_H, S_L
    uint8_t bufferIndex_;
    
    void resetParser();
    bool parsePacket();
};
