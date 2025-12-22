#pragma once

#include <Arduino.h>

// Update these pin assignments to match your Nucleo-F303RE wiring.
// Use STM32 pin names (e.g., PA8, PB3) supported by the STM32 Arduino core.
static constexpr uint8_t PIN_UNASSIGNED = 0xFF;

struct MotorPins {
    uint8_t rPwm;
    uint8_t lPwm;
    uint8_t rEn;
    uint8_t lEn;
};

// Four motor slots (expandable). Pins default to PIN_UNASSIGNED and should be
// edited for your wiring. When left unassigned the motor driver keeps outputs idle.
// Choose PWM-capable pins and avoid conflicts:
// - Reserve PA9/PA10 for Serial1 (i-BUS RX/TX)
// - Reserve PB8/PB9 for I2C1 (HuskyLens)
// Verified TIM channels: TIM1(PA8,PA11), TIM3(PA6,PA7,PB0,PB1,PB4,PB5), TIM2(PA15), TIM15(PA2,PA3)
static constexpr MotorPins MOTOR_PINS[] = {
    // Motor 1: TIM1_CH1(PA8), TIM1_CH4(PA11)
    {PA8,  PA11, PC7, PC7},
    // Motor 2: TIM3_CH1(PA6), TIM3_CH2(PA7)
    {PA6,  PA7,  PB6, PB6},
    // Motor 3: TIM3_CH3(PB0), TIM3_CH4(PB1)
    {PB0,  PB1,  PA5, PA5},
    // Motor 4: TIM3_CH1(PB4), TIM3_CH2(PB5)
    {PB4,  PB5,  PC6, PC6}
};

static constexpr size_t NUM_MOTORS = sizeof(MOTOR_PINS) / sizeof(MotorPins);

// ESP32 receiver forwards control data via UART (non-inverted, 115200 baud).
#define SBUS_SERIAL Serial1
static constexpr uint32_t SBUS_BAUD = 115200;

// Calibration window duration in milliseconds.
static constexpr uint32_t RC_CALIBRATION_MS = 10000;

// RC channel indices (0-based) used for driving; adjust as needed.
static constexpr uint8_t RC_CHANNEL_THROTTLE = 0;
static constexpr uint8_t RC_CHANNEL_STEERING = 1;

// Deadzone around neutral to suppress motor twitch.
static constexpr int RC_DEADZONE = 20;
