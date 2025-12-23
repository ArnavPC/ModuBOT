#Troubleshooting and Configuration :D




## Configuration

### STM32 Firmware (`include/pins.h`)
- Adjust motor pin assignments (keep timers in mind)
- Change RC channel roles (`RC_CHANNEL_THROTTLE`, `RC_CHANNEL_STEERING`)
- Tune deadzone, calibration duration, or UART port

### ESP32 Firmware

**Remote** (`esp32_firmware/remote/main.cpp`):
- Set `receiverMAC[]` to your receiver ESP32's MAC address (printed on boot)
- Adjust joystick center calibration if needed in `mapJoystick()`

**Receiver** (`esp32_firmware/receiver/main.cpp`):
- Set `remoteMAC[]` to your remote ESP32's MAC address
- Calibrate `VOLTAGE_DIVIDER_RATIO` based on your battery sensor circuit
- Verify UART pins match STM32 wiring (GPIO 17 TX → PA10 RX)

## Troubleshooting

**No control data from remote?**
- Verify both ESP32s have correct MAC addresses configured.
- Check ESP-NOW channel matches (use channel 0 or explicitly set same channel).
- Ensure remote powers on before receiver and displays "Ready!" on OLED.
- Monitor receiver serial output to confirm packet reception.

**Motors not responding?**
- Ensure calibration completes (watch Serial output for "Calibration Complete").
- Check pins in `pins.h` match your wiring.
- Verify enable pins (R_EN/L_EN) are HIGH after initialization.
- Confirm UART packets are arriving: look for `< T_H T_L S_H S_L >` pattern in debug output.

**Battery voltage incorrect?**
- Calibrate `VOLTAGE_DIVIDER_RATIO` in receiver firmware.
- Verify GPIO 34 wiring and voltage divider resistor values.
- Test ADC reading with multimeter for reference.

**Build fails?**
- Run `pio pkg update` and retry.
- Delete `.pio/build/` and try again.
- For ESP32: ensure `platform = espressif32` and TFT_eSPI library installed.