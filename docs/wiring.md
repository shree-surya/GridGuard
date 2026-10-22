# GridGuard wiring

ESP32 38-pin DevKit (board "ESP32 Dev Module"). All grounds are one common GND.

## Pin map

| ESP32 pin | Goes to | Notes |
|---|---|---|
| VIN | 5 V rail (Hi-Link +Vo) | no series diode |
| GND | GND rail | |
| 3V3 | TFT VCC, TFT BLK | |
| GPIO34 | voltage divider node (ZMPT101B OUT) | input only, ADC1 |
| GPIO35 | voltage divider node (ACS712 OUT) | input only, ADC1 |
| GPIO23 | TFT SCK | SPI clock |
| GPIO19 | TFT SDA | SPI data (MOSI), not I2C |
| GPIO5 | TFT CS | |
| GPIO4 | TFT RST | |
| GPIO27 | TFT DC | |
| GPIO13 | push button (other leg to GND) | internal pull-up, no resistor |
| GPIO25 | buzzer driver (NPN base, through 1k) | |
| GPIO26 | relay module IN (+ 10k to GND) | |
| GPIO32 | green ARMED LED (through 330R) | optional |
| GPIO33 | red TRIP LED (through 330R) | optional |

Free for later: GPIO14, 16, 17, 18, 21, 22. Avoid GPIO0, 2, 12, 15 (boot pins) and 6 to 11 (flash).

## Low-voltage side

```
                                 ESP32 DevKit
                         ┌───────────────────────────┐
  5V RAIL ───────────────┤ VIN                   3V3 ├──┬── TFT VCC
  GND RAIL ──────────────┤ GND                       │  └── TFT BLK
                         │                           │
  ZMPT OUT ──[10k]──●────┤ GPIO34                    │
                    ├─[20k]── GND                    │
                    └─|100nF|─ GND                   │
                         │                           │
  ACS OUT ───[10k]──●────┤ GPIO35                    │
                    ├─[20k]── GND                    │
                    └─|100nF|─ GND                   │
                         │                           │
  TFT SCK ───────────────┤ GPIO23                    │
  TFT SDA ───────────────┤ GPIO19                    │
  TFT CS ────────────────┤ GPIO5                     │
  TFT RST ───────────────┤ GPIO4                     │
  TFT DC ────────────────┤ GPIO27                    │
                         │                           │
  GND ── button ─────────┤ GPIO13                    │
                         │                           │
  Relay IN ─────────●────┤ GPIO26                    │
                    └─[10k]── GND                    │
                         │                           │
           ┌─[1k]────────┤ GPIO25                    │
           │             │                           │
           B  BC547/2N2222 NPN                       │
           E ── GND                                  │
           C ── buzzer (−)    buzzer (+) ── 5V RAIL  │
                         │                           │
  GND ─ LED− LED+ ─[330R]┤ GPIO32  (green, optional) │
  GND ─ LED− LED+ ─[330R]┤ GPIO33  (red, optional)   │
                         └───────────────────────────┘

  5V RAIL  ── ZMPT101B VCC, ACS712 VCC, relay module VCC (DC+)
  GND RAIL ── ZMPT GND, ACS712 GND, relay GND (DC−), TFT GND, both 20k, all 100nF
  VIN–GND  ── 470 µF (long leg to VIN) + 100 nF, right at the ESP32 pins
```

### Module by module

**Voltage sensor ZMPT101B:** VCC → 5V rail, GND → GND, OUT → 10k → node → GPIO34. Node: 20k to GND and 100 nF to GND. Input L/N across live (after the MCB, before the relay) and neutral.

**Current sensor ACS712-20A:** VCC → 5V rail, GND → GND, OUT → 10k → node → GPIO35. Node: 20k to GND and 100 nF to GND. IP+/IP- in series with the live wire after the relay. Optional: 10 µF + 100 nF across its VCC/GND, at the module.

The 10k/20k dividers scale the 5 V sensor outputs to 3.3 V. The 10k goes from the sensor to the node, the 20k from the node to GND. Swapped, the pin can see over 3.3 V.

**TFT 1.77" ST7735S:** VCC → 3V3, GND → GND, SCK → 23, SDA → 19, CS → 5, RST → 4, DC → 27, BLK → 3V3.

**Push button:** one leg → GPIO13, the other → GND. No resistor: the firmware uses the ESP32's internal pull-up (reads LOW while pressed) and debounces it. On a 4-leg tactile switch use two diagonally opposite legs. Optional 100 nF across the button if its wires are long or run near mains. Short press: next screen. Hold 2 s: relay on (resets a trip; out-of-range supply = voltage override) or off.

**Buzzer (5 V active):** + → 5V rail, − → NPN collector. NPN emitter → GND, base → 1k → GPIO25. BC547 legs, flat face towards you: C B E. 2N2222 (TO-92) flat face towards you: E B C.

**Relay module (5 V, 10 A, single channel):** DC+ / VCC → 5V rail, DC− / GND → GND, IN → GPIO26, and 10k from GPIO26 to GND. Set the trigger jumper to **H (high)**. That way the relay stays open while the ESP32 boots, crashes or is unpowered. If the module has no H/L jumper it is usually active-low: do not use it as is (it closes when the pin is low or floating); use a high-trigger module or drive a bare relay with an NPN plus a 1N4007 across the coil.

**LEDs (optional):** GPIO → 330R → LED long leg; short leg (flat side) → GND.

## Power

```
Hi-Link HLK-5M05
  AC L ── 1 A slow-blow fuse ── live after the MCB (before the relay)
  AC N ── neutral
  +Vo  ── 5V rail  (straight to ESP32 VIN, no diode)
  −Vo  ── GND rail
```

- No diode between the Hi-Link and VIN: on the PowerScope bench a 1N4007-type diode dropped VIN to 3.6 V and the ESP32 reset when WiFi started.
- Because of that, **never connect USB while the box is on mains.** Unplug the mains first.
- The relay coil switching dips the rail, so the 470 µF on VIN is required here.
- Budget: ESP32 up to 500 mA peak, relay coil 70 to 80 mA, TFT backlight about 30 mA, sensors and buzzer about 50 mA. Under 700 mA against the Hi-Link's 1 A.

## Mains side

```
plug L ── MCB ──●── relay COM ── relay NO ── ACS712 IP+ ── IP- ── rocker switch ── socket L
                │
                ├── Hi-Link AC L (through its 1 A fuse)
                └── ZMPT101B L
plug N ─────────┬────────────────────────────────────────────────────────────── socket N
                ├── Hi-Link AC N
                └── ZMPT101B N
plug E ──────────────────────────────────────────────────────────────────────── socket E
```

- Live only through the MCB, relay and ACS712. Neutral and earth go straight through.
- Hi-Link and ZMPT101B are taken after the MCB and **before** the relay, so the controller keeps running and keeps measuring the supply after a trip.
- Relay on **NO** (normally open): no power to the coil means the socket is off.
- Test the wall socket for swapped live and neutral first.
- A B6 MCB suits a 6 A socket better than a 10 A one.

## Before the first power-on

Power off, plug out, multimeter:
1. 5V rail to GND, and 3V3 to GND: not a short (a few kΩ or more is normal).
2. Any mains point (plug L, N, relay COM/NO, ACS712 IP+/IP-, ZMPT input, Hi-Link AC pins) to ESP32 GND: open (OL).

Then:
1. USB only, no mains: TFT works, GPIO34 and GPIO35 nodes read about 1.65 V, relay stays off.
2. Unplug USB. Mains through a 60 to 100 W bulb in series with the live: the bulb should glow dim or briefly, then remove it.
3. Check the relay with a bulb load: off at power-up and while the EN button is held, on only when the firmware switches it.
