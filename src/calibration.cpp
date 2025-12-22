#include "calibration.h"

RCCalibration::RCCalibration() : endMs_(0), active_(false) {
    reset(0);
}

void RCCalibration::reset(uint32_t durationMs) {
    for (auto &c : data_) {
        c.min = 32767;
        c.max = -32768;
        c.neutral = 0;
        c.initialized = false;
    }
    endMs_ = millis() + durationMs;
    active_ = durationMs > 0;
}

bool RCCalibration::isActive() const {
    return active_ && (static_cast<int32_t>(millis() - endMs_) < 0);
}

bool RCCalibration::isComplete() const {
    return !isActive();
}

void RCCalibration::sample(const int16_t *channels, size_t count) {
    if (!isActive()) return;

    uint32_t now = millis();
    if (static_cast<int32_t>(now - endMs_) >= 0) {
        active_ = false;
        return;
    }

    for (size_t i = 0; i < count && i < 16; ++i) {
        int16_t v = channels[i];
        auto &c = data_[i];
        if (!c.initialized) {
            c.min = c.max = c.neutral = v;
            c.initialized = true;
        } else {
            c.min = min(c.min, v);
            c.max = max(c.max, v);
        }
    }
}

void RCCalibration::finalize() {
    active_ = false;
    for (auto &c : data_) {
        if (!c.initialized) continue;
        if (c.max == c.min) {
            c.max = c.min + 1;
        }
        c.neutral = (c.min + c.max) / 2;
    }
}

int16_t RCCalibration::mapToCommand(int16_t value, size_t idx, int deadzone, int16_t outputMax) const {
    if (idx >= 16) return 0;
    const auto &c = data_[idx];
    if (!c.initialized) return 0;

    int16_t neutral = c.neutral;
    int16_t spanHigh = max<int16_t>(1, c.max - neutral);
    int16_t spanLow = max<int16_t>(1, neutral - c.min);

    int16_t delta = value - neutral;
    if (abs(delta) <= deadzone) return 0;

    if (delta > 0) {
        float scaled = (static_cast<float>(delta - deadzone) / spanHigh) * outputMax;
        return static_cast<int16_t>(constrain(scaled, 0.0f, static_cast<float>(outputMax)));
    } else {
        float scaled = (static_cast<float>(delta + deadzone) / spanLow) * outputMax;
        return static_cast<int16_t>(constrain(scaled, -static_cast<float>(outputMax), 0.0f));
    }
}

const ChannelCal &RCCalibration::channel(size_t idx) const {
    static ChannelCal empty{0, 0, 0, false};
    if (idx >= 16) return empty;
    return data_[idx];
}
