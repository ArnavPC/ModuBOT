#pragma once

#include <Arduino.h>

struct ChannelCal {
    int16_t min;
    int16_t max;
    int16_t neutral;
    bool initialized;
};

class RCCalibration {
public:
    RCCalibration();
    void reset(uint32_t durationMs);
    bool isActive() const;
    bool isComplete() const;
    void sample(const int16_t *channels, size_t count);
    void finalize();
    int16_t mapToCommand(int16_t value, size_t idx, int deadzone, int16_t outputMax = 255) const;
    const ChannelCal &channel(size_t idx) const;

private:
    ChannelCal data_[16];
    uint32_t endMs_;
    bool active_;
};
