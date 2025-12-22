# ModularBOT Firmware Architecture

## Overview
- **Target**: STM32 Nucleo-F303RE using Arduino framework (PlatformIO `ststm32`).
- **Modules**: motor driver abstraction, custom ESP-NOW radio link, calibration helper, main control loop.
- **Hardware**: 4× BTS7960 H-bridge drivers (R/L PWM + enable), ESP32-based custom radio, HuskyLens on I2C (reserved).

## System Architecture

### Radio Link (ESP-NOW Telemetry)

**Remote (LILYGO T-Display ESP32)**
- Reads 2× analog joysticks (throttle/steering) and 4× buttons.
- Maps joystick ADC values (0-4095) to RC PWM range (1000-2000 µs).
- Transmits control data to receiver via ESP-NOW at ~50 Hz.
- Displays connection status and battery voltage on built-in ST7789 TFT (135×240).
- **Power**: Rechargeable via USB-C with onboard Battery Management System (BMS) for 3.7V 1S LiPo.
- **⚠️ SAFETY**: Remote battery port is for **3.7V (1S) LiPo ONLY**. Never connect robot's 12.6V battery to remote.

**Receiver (ESP32 DevKit V1)**
- Receives control packets wirelessly via ESP-NOW.
- Forwards throttle/steering to STM32 via UART in `< T_H T_L S_H S_L >` format (6 bytes).
- Reads battery voltage (GPIO 34 ADC) and sends telemetry back to remote at ~10 Hz.

**STM32 Nucleo-F303RE**
- Parses incoming UART packets from ESP32 receiver.
- Applies calibration, deadzone, and differential mixing.
- Drives 4× motors via PWM with failsafe on communication timeout.

## STM32 Firmware Modules

- `motor_driver.[h/cpp]`: wraps BTS7960 control; exposes `begin`, `stop`, and `drive(int16_t)` with signed PWM magnitude.
- `rc_input.[h/cpp]`: parses UART packets from ESP32 receiver (`< T_H T_L S_H S_L >`). Stores throttle/steering values and detects communication timeouts.
- `calibration.[h/cpp]`: tracks per-channel min/max/neutral over a timed window during boot.
- `main.cpp`: orchestrates startup, calibration window, RC mapping to four motors, failsafe handling.
- `include/pins.h`: central pin/port definitions (UART port, motor pin sets, RC channel indices, deadzone, calibration duration).

## Control Flow

1. **Setup**: Initialize serial console, UART receiver (ESP32 link), calibration window, and motor drivers (safe stop when pins unassigned).
2. **Calibration** (first `RC_CALIBRATION_MS`): Collect min/max/neutral from incoming packets while motors stay stopped. Operator moves joysticks to extremes on remote, then centers.
3. **Finalization**: When calibration time expires, compute neutral as midpoint of min/max.
4. **Active Loop**: 
   - Parse UART packets continuously.
   - Map throttle/steering through calibrated ranges into signed PWM (-255..255) with deadzone.
   - Mix throttle + steering into left/right motor commands (motors 0/2 = left, 1/3 = right by default).
   - Stop all motors on packet timeout or invalid framing.
5. **Telemetry**: ESP32 receiver sends battery voltage back to remote for live display.

## Packet Format (ESP32 → STM32)

```
< Throttle_High Throttle_Low Steering_High Steering_Low >
```

- **Start byte**: `<` (0x3C)
- **Throttle**: 16-bit unsigned (1000-2000), big-endian
- **Steering**: 16-bit unsigned (1000-2000), big-endian
- **End byte**: `>` (0x3E)

## Extensibility Notes

- Pin mapping isolated in `pins.h`; expand `MOTOR_PINS` array to add motors.
- RC channel roles (throttle/steering) defined in `pins.h` constants.
- Calibration duration and deadzone tunable via `pins.h`.
- ESP32 firmware is modular: swap ESP-NOW for WiFi UDP or Bluetooth Classic with minimal changes.
- Future: integrate HuskyLens I2C vision processing in a separate module without coupling radio pipeline.
