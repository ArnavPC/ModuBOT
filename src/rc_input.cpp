#include "rc_input.h"
#include <cstring>

#define PACKET_TIMEOUT_MS 500  // Failsafe if no packet for 500ms

RCInput::RCInput() : newFrame_(false), lastPacketTime_(0), parseState_(WAIT_START), bufferIndex_(0) {
    memset(&state_, 0, sizeof(state_));
    state_.failsafe = true;  // Start in failsafe
}

void RCInput::begin() {
    // Ensure UART is configured (done in main.cpp: Serial1.begin(115200))
    SBUS_SERIAL.begin(SBUS_BAUD);
    resetParser();
}

void RCInput::resetParser() {
    parseState_ = WAIT_START;
    bufferIndex_ = 0;
}

bool RCInput::parsePacket() {
    // Reconstruct throttle and steering from big-endian bytes
    uint16_t throttle = (packetBuffer_[0] << 8) | packetBuffer_[1];
    uint16_t steering = (packetBuffer_[2] << 8) | packetBuffer_[3];
    
    // Validate range (1000-2000)
    if (throttle < 900 || throttle > 2100 || steering < 900 || steering > 2100) {
        return false;  // Invalid data
    }
    
    state_.channels[0] = throttle;
    state_.channels[1] = steering;
    state_.failsafe = false;
    state_.frameLost = false;
    lastPacketTime_ = millis();
    newFrame_ = true;
    
    return true;
}

void RCInput::update() {
    // Check for timeout failsafe
    if (millis() - lastPacketTime_ > PACKET_TIMEOUT_MS) {
        state_.failsafe = true;
        state_.frameLost = true;
    }
    
    // Parse incoming UART bytes
    while (SBUS_SERIAL.available()) {
        uint8_t byte = SBUS_SERIAL.read();
        
        switch (parseState_) {
            case WAIT_START:
                if (byte == '<') {
                    parseState_ = READ_T_HIGH;
                    bufferIndex_ = 0;
                }
                break;
                
            case READ_T_HIGH:
                packetBuffer_[bufferIndex_++] = byte;
                parseState_ = READ_T_LOW;
                break;
                
            case READ_T_LOW:
                packetBuffer_[bufferIndex_++] = byte;
                parseState_ = READ_S_HIGH;
                break;
                
            case READ_S_HIGH:
                packetBuffer_[bufferIndex_++] = byte;
                parseState_ = READ_S_LOW;
                break;
                
            case READ_S_LOW:
                packetBuffer_[bufferIndex_++] = byte;
                parseState_ = WAIT_END;
                break;
                
            case WAIT_END:
                if (byte == '>') {
                    parsePacket();
                }
                resetParser();
                break;
        }
    }
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
