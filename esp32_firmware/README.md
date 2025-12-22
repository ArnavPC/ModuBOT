# ESP32 Firmware Build Instructions

## Directory Structure

```
esp32_firmware/
├── receiver/
│   ├── platformio.ini
│   └── main.cpp
└── remote/
    ├── platformio.ini
    └── main.cpp
```

## Prerequisites

- PlatformIO Core or PlatformIO IDE extension for VS Code
- ESP32 toolchain (installed automatically by PlatformIO)

## Building and Flashing

### Receiver (ESP32 DevKit V1)

```powershell
cd esp32_firmware\receiver
pio run
pio run -t upload
pio device monitor
```

**First Boot**: Note the MAC address printed to serial — you'll need it for the remote configuration.

### Remote (LILYGO T-Display)

**⚠️ SAFETY FIRST**: The LILYGO T-Display has a built-in 3.7V (1S) LiPo battery connector with USB-C charging and BMS. **NEVER** connect the robot's 12.6V battery to this port—it will destroy the board.

1. **Update MAC Address**: Edit `main.cpp` and set `receiverMAC[]` to your receiver's MAC address.

2. **Build and Flash**:
```powershell
cd esp32_firmware\remote
pio run
pio run -t upload
pio device monitor
```

**First Boot**: Note the MAC address printed to serial — you'll need it for the receiver configuration.

3. **Update Receiver**: Go back to receiver `main.cpp` and set `remoteMAC[]` to the remote's MAC address, then reflash the receiver.

## Configuration

### Receiver (`receiver/main.cpp`)

- **`remoteMAC[]`**: Set to your TTGO T-Display MAC address
- **`VOLTAGE_DIVIDER_RATIO`**: Calibrate based on your battery sensor circuit
- **UART Pins**: GPIO 17 (TX → STM32 PA10), GPIO 16 (RX ← STM32 PA9)

### Remote (`remote/main.cpp`)

- **`receiverMAC[]`**: Set to your ESP32 DevKit MAC address
- **Joystick Calibration**: Adjust `mapJoystick()` if center isn't ~2048 ADC
- **Button Pins**: GPIO 17, 21, 22, 27 (active LOW with pullup)
- **Display**: TFT_eSPI library configured for LILYGO T-Display (ST7789, 135×240)

## Troubleshooting

**Compile Errors for `esp_now.h` or `WiFi.h`**:
- These are ESP32-IDF headers included automatically with `platform = espressif32`
- Ensure you're using PlatformIO, not Arduino IDE directly

**TFT_eSPI Display Issues (Remote)**:
- Build flags in `platformio.ini` configure ST7789 for TTGO T-Display
- If display doesn't work, verify your board revision matches config

**No ESP-NOW Communication**:
- Verify MAC addresses are set correctly in both firmwares
- Both ESP32s must be on the same WiFi channel (default: 0)
- Check serial output for "ESP-NOW Init Failed" or "Add Peer Failed"

**Battery Voltage Incorrect**:
- Measure actual voltage with multimeter
- Adjust `VOLTAGE_DIVIDER_RATIO` = (R1 + R2) / R2
- Test ADC reading: `Serial.println(analogRead(34));`

## Serial Debug Output

**Receiver**:
```
=== ModuBOT Receiver (ESP32) ===
ESP32 MAC Address: XX:XX:XX:XX:XX:XX
Ready. Listening for control data...
RX: T=1500 S=1500 Btn=0x00
```

**Remote**:
```
Initializing...
Ready!
[OLED shows throttle, steering, battery voltage, buttons]
```

## Next Steps

After both ESP32s are configured and communicating:
1. Verify UART packets on STM32 side (connect receiver GPIO 17 → STM32 PA10)
2. Monitor STM32 serial for packet parsing: `< T_H T_L S_H S_L >`
3. Complete calibration on STM32 (10s window, move joysticks to extremes)
4. Test motor response with throttle/steering inputs
