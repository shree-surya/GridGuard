# GridGuard spec

Reference: the working simulator in docs/GridGuard-project-page.html (state machine in its script).

## Settings (set on the web page, saved in flash)
| Setting | Default | Note |
|---|---|---|
| Over-voltage (OVV) | 250 V | |
| Under-voltage (UVV) | 190 V | |
| Over-current (OCV) | 6 A | keep at 7 A or below, the relay is 10 A |
| Under-current (UCV) | 0.5 A, off by default | not below about 0.3 A, sensor noise (see idle-noise-analysis.md; with WiFi on, the floor is higher) |
| Trip delay | 2 s | a fault must last this long |
| Restart delay | 10 s demo default | fridge preset 180 s |
| After a trip | auto reconnect or wait for reset | one global setting |
| Hysteresis | 2 % | a voltage counts as normal again inside OVV x 0.98 and UVV x 1.02 |

Presets (starting points): Fridge (190 to 250 V, 6 A, restart 180 s), Water pump (190 to 250 V, 6 A, under-current 0.5 A, restart 30 s), Charger (190 to 250 V, 3 A, restart 10 s).

## States
Armed -> Tripping (delay counting) -> Tripped (relay open, buzzer, red LED, reason logged) -> Waiting (auto mode, restart delay counting) -> Armed.
- With the relay open the load current cannot be seen. After a current trip the unit simply retries when the restart delay ends and trips again if the fault persists.
- A voltage fault only reconnects once the supply is back inside the hysteresis band.
- After closing the relay, ignore the under-current check for 1.5 s.
- Manual reset refused while the supply is still out of range.
- Relay wired normally open: an unpowered or crashed controller means the load is off.

## Hardware and I/O
| Part | ESP32 pin | Note |
|---|---|---|
| ZMPT101B OUT | GPIO34 | 10k series, 20k + 100nF to GND at the pin (same as PowerScope) |
| ACS712-20A OUT | GPIO35 | same divider |
| 1.77" ST7735S TFT, 128 x 160, SPI | SCK 18, SDA(MOSI) 19, CS 5, RST 4, DC 27 | VCC 3V3, BLK 3V3. Replaces the 1.3" OLED |
| Relay module 5 V, 10 A | GPIO26 | COM/NO on live. Use a high-level-trigger module (H/L jumper on H) and 10k GPIO26 to GND, so boot, crash or no power = relay open |
| Buzzer, 5 V active | GPIO25 | through an NPN (BC547/2N2222), 1k base resistor |
| Push button | GPIO13 | to GND, internal pull-up, debounced in firmware. Replaces the TTP223 touch pad. Short: next screen, 2 s hold: reset a trip |
| Red TRIP LED / green ARMED LED | GPIO33 / GPIO32 | 330 ohm each |

Power: Hi-Link HLK-5M05 straight to VIN (no series diode: a 1N4007-type drop left VIN at 3.6 V and the ESP32 browned out on WiFi start), 470 uF + 100 nF across VIN and GND. Never plug USB in while on mains. Hi-Link and ZMPT101B are tapped after the MCB and before the relay, so the controller keeps measuring after a trip. Check the relay logic polarity with a bulb before relying on it.

## Display (1.77" TFT, portrait)
Five screens, drafted in firmware/GridGuard_display_test (see docs/display-preview.png): Home (status bar ARMED green / TRIP IN amber / TRIPPED red flashing / WAITING blue, voltage and current with meters against the limits, power and relay state), Graph (last 60 s with limit lines), Trips (last trip, total, two earlier ones), Limits (read only), Connect (WiFi name, password, IP). A trip jumps to Home. Font: Bai Jamjuree in 11, 16 and 32 px, made with tools/ttf2gfx.py. Palette as the web pages.

## Web (own hotspot GridGuard-XXXX, WPA2)
Settings page behind a PIN, event log with reason and readings (flash, time from the phone, uptime until a phone connects), live graph with limit lines, manual relay ON/OFF, calibration page shared with PowerScope, trip counter.

## Open questions
Event-log size and format in flash; whether the relay should open on over-power (watts) as a fifth limit; buzzer pattern per fault.
