# ModularBOT Firmware – STM32 Nucleo-F303RE

A modular robotics control platform featuring autonomous RC calibration, differential motor mixing, and FlySky i-BUS receiver support.

## Quick Start

### Prerequisites
- PlatformIO IDE or CLI (`python -m pip install platformio`)
- STM32 ST-Link debugger
- Nucleo-F303RE development board

### Build
```bash
pio run
```

### Upload
```bash
pio run -t upload
```

### Monitor Serial Output
```bash
pio device monitor
```

## Hardware Setup

| Component          | Connection                | Note                             |
|--------------------|---------------------------|----------------------------------|
| FlySky FS-iA6B     | PA9 (TX) / PA10 (RX)      | i-BUS @ 115200 baud             |
| Motor 1 PWM        | PA8, PA11 + PC7 (EN)      | TIM1_CH1/CH4                     |
| Motor 2 PWM        | PA6, PA7 + PB6 (EN)       | TIM3_CH1/CH2                     |
| Motor 3 PWM        | PB0, PB1 + PA5 (EN)       | TIM3_CH3/CH4                     |
| Motor 4 PWM        | PB4, PB5 + PC6 (EN)       | TIM3_CH1/CH2                     |
| HuskyLens I2C      | PB8 (SCL) / PB9 (SDA)     | Reserved for future integration  |

## Firmware Behavior

1. **Boot** (~500 ms): Initialize Serial @ 115200, i-BUS receiver, and motors.
2. **Calibration** (10 seconds): Collect RC channel min/max while motors stay stopped. Move all sticks to extremes, then center.
3. **Active** (post-calibration): 
   - Map throttle + steering to motor PWM via calibrated ranges with deadzone.
   - Differential mixing: left motors = throttle + steering, right motors = throttle − steering.
   - Any failsafe or frame loss → all motors stop immediately.

## Project Structure

```
├── include/
│   └── pins.h              # Pin & protocol configuration
├── src/
│   ├── main.cpp            # Control loop & initialization
│   ├── motor_driver.cpp/h  # BTS7960 H-bridge abstraction
│   ├── rc_input.cpp/h      # i-BUS polling & frame handling
│   ├── calibration.cpp/h   # RC min/max/neutral tracking
├── docs/
│   └── architecture.md     # Design & module details
├── platformio.ini          # Build & dependency config
└── README.md               # This file
```

## Configuration

Edit `include/pins.h` to:
- Adjust motor pin assignments (keep timers in mind)
- Change RC channel roles (`RC_CHANNEL_THROTTLE`, `RC_CHANNEL_STEERING`)
- Tune deadzone, calibration duration, or i-BUS UART port

## Troubleshooting

**No i-BUS data?**
- Verify receiver is outputting i-BUS (not PPM or S.BUS).
- Check baud rate is 115200 and serial line is not inverted.
- Confirm PA9/PA10 wiring to receiver TX/RX.

**Motors not responding?**
- Ensure calibration completes (watch Serial output for "Calibration Complete").
- Check pins in `pins.h` match your wiring.
- Verify enable pins (R_EN/L_EN) are HIGH after initialization.

**Build fails?**
- Run `pio pkg update` and retry.
- Delete `.pio/build/` and try again.

## License

MIT

## Author

Arnav – ModularBOT Project
