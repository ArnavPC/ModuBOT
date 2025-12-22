# ModularBOT Firmware Architecture

## Overview
- Target: STM32 Nucleo-F303RE using Arduino framework (PlatformIO `ststm32`).
- Modules: motor driver abstraction, SBUS RC input, calibration helper, main control loop.
- Hardware: 4x BTS7960 H-bridge drivers (R/L PWM + enable), FlySky SBUS receiver, HuskyLens on I2C (logic-only placeholder for now).

## Modules
- `motor_driver.[h/cpp]`: wraps BTS7960 control; exposes `begin`, `stop`, and `drive(int16_t)` with signed PWM magnitude.
- `rc_input.[h/cpp]`: polls FlySky i-BUS via hardware UART (`SBUS_SERIAL`) using IBusBM (`#include <IBusBM.h>`). Stores latest channel values and basic failsafe/frame-lost heuristics.
- `calibration.[h/cpp]`: tracks per-channel min/max/neutral over a timed window.
- `main.cpp`: orchestrates startup, calibration window, RC mapping to four motors.
- `include/pins.h`: central pin/port definitions (SBUS UART, motor pin sets, RC channel indices, deadzone, calibration duration).

## Control Flow
1. `setup`: initialize serial console, RC input, calibration window, and motor drivers (safe stop when pins unassigned).
2. `loop`: poll SBUS frames; during first `RC_CALIBRATION_MS`, collect min/max/neutral while motors stay stopped.
3. When calibration time expires, finalize neutral as midpoint of min/max.
4. After calibration completes, map RC channels through calibrated ranges into signed PWM (-255..255) with a deadzone, then mix throttle/steering into left/right motor commands (0/2 = left, 1/3 = right by default).
5. i-BUS update runs continuously (`ibus_.loop()`), reading channels each loop; no explicit frame flag is provided.

## Extensibility Notes
- Pin mapping isolated in `pins.h`; expand `MOTOR_PINS` array to add motors.
- RC channel roles (throttle/steering) defined in `pins.h` constants.
- Calibration duration and deadzone tunable via `pins.h`.
- Future: add HuskyLens I2C logic in a separate module and integrate into main loop without coupling RC pipeline.
 - SBUS inversion can be toggled with `SBUS_INVERTED` in `pins.h`. Standard receivers output inverted SBUS; some hardware provides de-inverted lines.
