# T-OI_Led

Candle-like flicker effect for a small RGB LED module driven by a LILYGO TTGO T-OI (ESP8266).

The firmware alternates between two modes:

- **Flicker** (30 s): warm orange light with randomly varying intensity and timing.
- **Pulse**: a single red or blue (chosen at random) fade up and down, then back to flicker.

## Hardware

- LILYGO TTGO T-OI (ESP8266, ESP-12E compatible)
- WS2812 / NeoPixel RGB LED module with 7 LEDs
- T-OI pin `GPIO4` → LED module `Data In`

Change `kPinLedStrip` and `kLedsNum` in `src/main.cpp` for a different wiring or LED count.

## Build and flash

The project uses [PlatformIO](https://platformio.org/) (VS Code extension or CLI).

On Windows, install the CH341 USB serial driver first:
https://github.com/LilyGO/LILYGO-T-OI/blob/master/CH341SER_2.EXE

```sh
pio run                  # build
pio run -t upload        # flash
pio device monitor       # serial monitor (115200 baud)
```

## Dependencies

- [FastLED](https://github.com/FastLED/FastLED) (version pinned in `platformio.ini`)
