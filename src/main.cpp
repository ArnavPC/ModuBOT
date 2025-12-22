#include <Arduino.h>
#include "pins.h"
#include "motor_driver.h"
#include "rc_input.h"
#include "calibration.h"

// STM32 (ST core) requires explicit HardwareSerial instantiation.
// Nucleo-F303RE: USART1 on PA10 (RX) / PA9 (TX)
HardwareSerial Serial1(PA10, PA9);

MotorDriver motors[NUM_MOTORS] = {
    MotorDriver(MOTOR_PINS[0]),
    MotorDriver(MOTOR_PINS[1]),
    MotorDriver(MOTOR_PINS[2]),
    MotorDriver(MOTOR_PINS[3])
};

RCInput rcInput;
RCCalibration rcCal;
bool calibrationFinalized = false;

void stopAllMotors() {
    for (auto &m : motors) {
        m.stop();
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n=== ModularBOTIO Boot ===");
    Serial.println("Target: STM32 Nucleo-F303RE");
    Serial.println("Protocol: FlySky i-BUS");

    rcInput.begin();
    rcCal.reset(RC_CALIBRATION_MS);
    Serial.print("RC Calibration: ");
    Serial.print(RC_CALIBRATION_MS / 1000);
    Serial.println("s window started.");
    Serial.println("Move all sticks to extremes, then center.\n");

    for (size_t i = 0; i < NUM_MOTORS; ++i) {
        motors[i].begin();
        if (motors[i].isConfigured()) {
            Serial.print("Motor ");
            Serial.print(i);
            Serial.println(": OK");
        }
    }
}

void loop() {
    static uint32_t lastCalPrint = 0;
    rcInput.update();

    if (!calibrationFinalized && !rcCal.isActive()) {
        rcCal.finalize();
        calibrationFinalized = true;
        Serial.println("\n=== Calibration Complete ===");
        Serial.println("Motors enabled. Ready to drive.\n");
    }

    // Print calibration countdown every second
    if (rcCal.isActive() && millis() - lastCalPrint >= 1000) {
        lastCalPrint = millis();
        uint32_t remaining = (rcCal.isActive()) ? ((RC_CALIBRATION_MS - (millis() - (millis() % RC_CALIBRATION_MS))) / 1000) : 0;
        Serial.print("Calibrating... ");
        Serial.print(remaining);
        Serial.println("s");
    }

    if (rcInput.hasNewFrame()) {
        RCState state = rcInput.consumeState();

        if (state.failsafe || state.frameLost) {
            stopAllMotors();
            return;
        }

        if (rcCal.isActive()) {
            rcCal.sample(state.channels, 16);
            stopAllMotors();
            return;
        }

        if (!calibrationFinalized) {
            rcCal.finalize();
            calibrationFinalized = true;
        }

        int16_t throttle = rcCal.mapToCommand(state.channels[RC_CHANNEL_THROTTLE], RC_CHANNEL_THROTTLE, RC_DEADZONE);
        int16_t steering = rcCal.mapToCommand(state.channels[RC_CHANNEL_STEERING], RC_CHANNEL_STEERING, RC_DEADZONE);

        // Simple differential mixing: left motors = throttle + steering, right motors = throttle - steering.
        int16_t leftCmd = constrain(throttle + steering, -255, 255);
        int16_t rightCmd = constrain(throttle - steering, -255, 255);

        // Assign motors: 0/2 left side, 1/3 right side. Adjust mapping to match wiring.
        if (NUM_MOTORS > 0) motors[0].drive(leftCmd);
        if (NUM_MOTORS > 1) motors[1].drive(rightCmd);
        if (NUM_MOTORS > 2) motors[2].drive(leftCmd);
        if (NUM_MOTORS > 3) motors[3].drive(rightCmd);
    }
}
