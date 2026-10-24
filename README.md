# GridGuard

An ESP32 protector for single-phase 230 V loads up to 6 A, built into a modular wall-plate box. It measures voltage and current and opens a relay on the live wire when they go outside the limits you set, then reconnects after a delay. Sister project of [PowerScope](https://github.com/shree-surya/PowerScope), which measures only; the sensing core (ZMPT101B + ACS712-20A into an ESP32) is the same.

| | |
|---|---|
| Protects against | voltage, current and power outside their windows: LSL (lower) and USL (upper) for each, any values |
| Display | 1.77" colour TFT (ST7735S, 128 x 160, SPI) |
| Web | own WiFi hotspot "GridGuard" (password `gridguard`), http://192.168.4.1: live readings, 60 s graph, limits, relay ON/OFF, calibration, trip log |
| Status | firmware 0.4.0: protection, display, button, WiFi and web page working in the PC simulation and a headless browser, not yet on hardware |

![Display preview](docs/display-preview.png)

## Layout

```
firmware/GridGuard/                the firmware: sensors, protection, relay, buzzer, button, display, WiFi + web
web/index.html                     the web page (self-contained); tools/embed_web.py turns it into firmware/GridGuard/web_index.h
firmware/GridGuard_display_test/   display test: the five screens with simulated readings, no sensors
firmware/GridGuard_hw_test/        hardware test: sensors, button, buzzer, relay, LEDs on one screen
docs/GridGuard-spec.md             limits, states, pins, display and web plan
docs/wiring.md                     complete wiring: pins, dividers, relay, buzzer, button, TFT, power, mains
docs/GridGuard-project-page.html   project page with a working simulator of the trip logic
tools/ttf2gfx.py                   turns a TTF into an Adafruit GFX font (used for Bai Jamjuree)
tools/embed_web.py                 gzips web/index.html into web_index.h; run it after every edit of the page
```

## Flash the firmware

1. Arduino IDE, board **ESP32 Dev Module**, **Tools > Partition Scheme > Huge APP (3MB No OTA/1MB SPIFFS)**. Library Manager: **Adafruit GFX Library** and **Adafruit ST7735 and ST7789 Library**. From GitHub (Code > Download ZIP, then Sketch > Include Library > Add .ZIP Library): **ESP32Async/ESPAsyncWebServer** and **ESP32Async/AsyncTCP**.
2. Open `firmware/GridGuard/GridGuard.ino` (keep `BaiJamjuree.h` and `web_index.h` beside it) and upload.
3. Phone: join WiFi **GridGuard**, password **gridguard**, open **http://192.168.4.1**. In **Limits**, set LSL and USL for voltage, current and power (any values, LSL below USL, an LSL of 0 never trips) and press Save limits (kept in flash). The relay is on only while every value is inside its window.
4. After a trip:
   - **voltage**: automatic. The relay closes again once the voltage has been back inside its window for 10 s (power-up works the same way).
   - **current or power**: the relay stays off (Reset needed) until you tap **ON** on the page or **hold the button 5 s**. The screen shows RESETTING 5 4 3 2 1; let go early to cancel. Then the relay closes and all limits are checked again.
   - Button: short press changes the screen; with the relay on, hold 2 s to turn it off. Web ON while the voltage is out of range is an override (voltage limits paused until it is back).
5. Calibrate on the web page (Calibration, saved in flash): type the multimeter voltage and press Match; run a kettle, type the clamp meter current and press Match; unplug the load and press Measure zero; Flip direction if power reads wrong. Gain, voltage offset, current trim and zero offset can also be typed in. The same works in the Serial Monitor (115200, "Newline"): `V 231.5`, `I 4.20`, `O -2.5` (voltage offset), `Z`, `F`, `D` (defaults), `?`.

![Firmware screens](docs/firmware-screens.png)

![Web page](docs/web-page.png)

## Run the display test

1. Arduino IDE, board **ESP32 Dev Module**. Library Manager: **Adafruit GFX Library** and **Adafruit ST7735 and ST7789 Library**.
2. Open `firmware/GridGuard_display_test/GridGuard_display_test.ino`. `BaiJamjuree.h` must stay in the same folder (it shows as a second tab).
3. Wiring: TFT VCC 3V3, GND, CS 5, RST 4, DC 27, SDA 19, SCK 23, BLK 3V3.
4. Upload. The screen changes every 5 s while a 100 s story runs: normal, over-voltage trip, reconnect, over-current trip, under-voltage trip. Serial Monitor at 115200 logs each trip.

If the picture is shifted, inverted or red and blue are swapped, see the notes at the top of the sketch.

## Licences

`BaiJamjuree.h` is generated from Bai Jamjuree (SIL Open Font License 1.1, Cadson Demak). No licence has been chosen for the rest of the repo yet.

## Safety

This switches 230 V mains. Relay on the live wire only, normally open, MCB stays as the real protection, every module inside a closed box. It is a learning and demonstration project, not a certified protection device.
