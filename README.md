# GridGuard

An ESP32 protector for single-phase 230 V loads up to 6 A, built into a modular wall-plate box. It measures voltage and current and opens a relay on the live wire when they go outside the limits you set, then reconnects after a delay. Sister project of [PowerScope](https://github.com/shree-surya/PowerScope), which measures only; the sensing core (ZMPT101B + ACS712-20A into an ESP32) is the same.

| | |
|---|---|
| Protects against | over-voltage, under-voltage, over-current, optional under-current |
| Display | 1.77" colour TFT (ST7735S, 128 x 160, SPI) |
| Web | own WiFi hotspot: settings behind a PIN, trip log, live graph |
| Status | display layout being tested with simulated readings; main firmware not started |

![Display preview](docs/display-preview.png)

## Layout

```
firmware/GridGuard_display_test/   display test: the five screens with simulated readings, no sensors
firmware/GridGuard_hw_test/        hardware test: sensors, button, buzzer, relay, LEDs on one screen
docs/GridGuard-spec.md             limits, states, pins, display and web plan
docs/wiring.md                     complete wiring: pins, dividers, relay, buzzer, button, TFT, power, mains
docs/GridGuard-project-page.html   project page with a working simulator of the trip logic
tools/ttf2gfx.py                   turns a TTF into an Adafruit GFX font (used for Bai Jamjuree)
```

## Run the display test

1. Arduino IDE, board **ESP32 Dev Module**. Library Manager: **Adafruit GFX Library** and **Adafruit ST7735 and ST7789 Library**.
2. Open `firmware/GridGuard_display_test/GridGuard_display_test.ino`. `BaiJamjuree.h` must stay in the same folder (it shows as a second tab).
3. Wiring: TFT VCC 3V3, GND, CS 5, RST 4, DC 27, SDA 23, SCK 18, BLK 3V3.
4. Upload. The screen changes every 5 s while a 100 s story runs: normal, over-voltage trip, reconnect, over-current trip, under-voltage trip. Serial Monitor at 115200 logs each trip.

If the picture is shifted, inverted or red and blue are swapped, see the notes at the top of the sketch.

## Licences

`BaiJamjuree.h` is generated from Bai Jamjuree (SIL Open Font License 1.1, Cadson Demak). No licence has been chosen for the rest of the repo yet.

## Safety

This switches 230 V mains. Relay on the live wire only, normally open, MCB stays as the real protection, every module inside a closed box. It is a learning and demonstration project, not a certified protection device.
