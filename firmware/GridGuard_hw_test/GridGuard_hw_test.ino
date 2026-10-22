/*
  GridGuard hardware test: checks every part of the wiring, one screen.
  =====================================================================
  What it does
    - Reads the voltage sensor (GPIO34) and current sensor (GPIO35) four times a second and shows
      the midpoint (should be about 1650 mV) and the AC level, plus rough volts and amps.
    - Push button (GPIO13): each press beeps the buzzer (GPIO25) and toggles the relay (GPIO26).
    - LEDs: green (GPIO32) while the relay is on, red (GPIO33) while it is off. Optional.
    - Everything is also printed in the Serial Monitor at 115200.
  The relay starts OFF and only changes when you press the button.

  Libraries (Library Manager): "Adafruit GFX Library", "Adafruit ST7735 and ST7789 Library"
  Board: "ESP32 Dev Module"

  WIRING (see docs/wiring.md)
    TFT       VCC 3V3, GND, CS 5, RST 4, DC 27, SDA 19, SCK 18, BLK 3V3
    ZMPT101B  OUT -> 10k -> GPIO34 node (20k + 100nF to GND)      VCC 5V, GND
    ACS712    OUT -> 10k -> GPIO35 node (20k + 100nF to GND)      VCC 5V, GND
    Button    GPIO13 to GND (internal pull-up)
    Buzzer    GPIO25 -> 1k -> NPN base; NPN collector to buzzer -, buzzer + to 5V
    Relay     IN GPIO26 (+10k to GND), jumper on H (high trigger), VCC 5V, GND
    LEDs      GPIO32 / GPIO33 -> 330R -> LED -> GND (optional)

  TEST ORDER (safe first)
    1. USB only, no mains, nothing on the relay contacts. Check the screen, the button, the beep,
       the relay click and LEDs. Both sensor midpoints should read about 1650 mV ("OK").
    2. Then a bulb on the relay contacts, still on a safe setup, to see ON really switches it.
    3. Mains on the sensors last (unplug USB first). The voltage should read about your mains
       voltage; the current reads only with a load and the relay on. The numbers here are rough:
       calibration comes in the real firmware.
*/

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

// ---------------- pins ----------------
const int TFT_CS = 5, TFT_RST = 4, TFT_DC = 27;
const int TFT_SDA = 19, TFT_SCK = 18;   // SPI data (MOSI) and clock: any free GPIOs work, set them to where your wires go
const int PIN_V = 34, PIN_I = 35;
const int PIN_BTN = 13, PIN_BUZ = 25, PIN_RELAY = 26, PIN_LED_G = 32, PIN_LED_R = 33;

// ---------------- rough scaling (same as PowerScope before calibration) ----------------
const float V_PER_MV = 0.946f;              // mains volts per mV at the pin (ZMPT101B as wired)
const float MV_PER_A = 100.0f * 2.0f / 3.0f; // ACS712-20A: 100 mV/A through the 2/3 divider

Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16     cv(128, 160);

#define rgb(c) ((uint16_t)((((c) >> 8) & 0xF800) | (((c) >> 5) & 0x07E0) | (((c) >> 3) & 0x001F)))
const uint16_t BG = rgb(0x0F1115), CARD = rgb(0x1D212A), LINE = rgb(0x2A303C);
const uint16_t FG = rgb(0xFFFFFF), MUTED = rgb(0x8A93A3);
const uint16_t VOLT = rgb(0x35D0FF), AMP = rgb(0xFF8A3D), GOOD = rgb(0x3DDC84), BAD = rgb(0xFF4D4D), WARN = rgb(0xFFB020);

struct Chan { float midMv, rmsMv; };
const int W_OK_X = 100;   // x of the OK / LO / HI badge
// declared here with its default argument, so the IDE does not add its own declaration
void text(const char *s, int x, int y, uint16_t col, uint8_t size = 1);
Chan     chV = { 0, 0 }, chI = { 0, 0 };
bool     relayOn = false;
int      presses = 0;
bool     btnDown = false;
uint32_t btnChangedAt = 0, beepUntil = 0;

// Reads both sensors for 200 ms (1600 pairs, 10 mains cycles): DC midpoint and AC rms in mV.
void readSensors() {
  const int N = 1600;
  static uint16_t v[1600], c[1600];
  uint32_t t = micros();
  for (int k = 0; k < N; k++) {
    v[k] = analogReadMilliVolts(PIN_V);
    c[k] = analogReadMilliVolts(PIN_I);
    t += 125;
    while ((int32_t)(micros() - t) < 0) { }
  }
  double mv = 0, mi = 0;
  for (int k = 0; k < N; k++) { mv += v[k]; mi += c[k]; }
  mv /= N; mi /= N;
  double sv = 0, si = 0;
  for (int k = 0; k < N; k++) { sv += (v[k] - mv) * (v[k] - mv); si += (c[k] - mi) * (c[k] - mi); }
  chV = { (float)mv, (float)sqrt(sv / N) };
  chI = { (float)mi, (float)sqrt(si / N) };
}

void setRelay(bool on) {
  relayOn = on;
  digitalWrite(PIN_RELAY, on ? HIGH : LOW);
  digitalWrite(PIN_LED_G, on ? HIGH : LOW);
  digitalWrite(PIN_LED_R, on ? LOW : HIGH);
}

void beep(uint32_t ms) { digitalWrite(PIN_BUZ, HIGH); beepUntil = millis() + ms; }

// Debounced button: acts on the press (pin goes LOW), ignores bounces shorter than 40 ms.
void pollButton() {
  bool down = digitalRead(PIN_BTN) == LOW;
  uint32_t now = millis();
  if (down != btnDown && now - btnChangedAt > 40) {
    btnDown = down;
    btnChangedAt = now;
    if (down) {
      presses++;
      setRelay(!relayOn);
      beep(80);
      Serial.printf("[button] press %d -> relay %s\n", presses, relayOn ? "ON" : "OFF");
    }
  }
}

// ---------------- drawing ----------------
void text(const char *s, int x, int y, uint16_t col, uint8_t size) {
  cv.setTextSize(size); cv.setTextColor(col); cv.setCursor(x, y); cv.print(s);
}
void okMark(int x, int y, bool ok, const char *why) {
  cv.fillRoundRect(x, y, 24, 11, 2, ok ? GOOD : BAD);
  text(ok ? "OK" : why, x + 3, y + 2, BG);
}

void sensorBlock(int y, const char *name, const Chan &c, uint16_t col, bool isVolt) {
  char b[32];
  text(name, 4, y, col);
  bool midOk = c.midMv > 1400 && c.midMv < 1900;      // about 1650 mV with nothing flowing
  okMark(W_OK_X, y - 2, midOk, c.midMv < 1400 ? "LO" : "HI");
  snprintf(b, sizeof(b), "mid %4.0f  ac %3.0f mV", c.midMv, c.rmsMv);
  text(b, 4, y + 12, MUTED);
  if (isVolt) snprintf(b, sizeof(b), "%5.1f V", c.rmsMv < 20 ? 0.0f : c.rmsMv * V_PER_MV);
  else        snprintf(b, sizeof(b), "%5.2f A", c.rmsMv / MV_PER_A);
  text(b, 4, y + 24, FG, 2);
}
void draw() {
  char b[32];
  cv.fillScreen(BG);
  cv.fillRect(0, 0, 128, 14, CARD);
  text("GRIDGUARD HW TEST", 4, 4, FG);

  sensorBlock(20, "VOLTAGE  GPIO34", chV, VOLT, true);
  cv.drawFastHLine(0, 62, 128, LINE);
  sensorBlock(68, "CURRENT  GPIO35", chI, AMP, false);
  cv.drawFastHLine(0, 110, 128, LINE);

  // relay, big
  cv.fillRoundRect(4, 116, 120, 22, 4, relayOn ? GOOD : BAD);
  text(relayOn ? "RELAY ON" : "RELAY OFF", relayOn ? 16 : 10, 120, BG, 2);

  snprintf(b, sizeof(b), "Button %s  presses %d", btnDown ? "DOWN" : "up", presses);
  text(b, 4, 142, btnDown ? WARN : MUTED);
  text("Press: relay + beep", 4, 151, MUTED);

  tft.drawRGBBitmap(0, 0, cv.getBuffer(), 128, 160);
}

void setup() {
  Serial.begin(115200);
  // outputs first, relay OFF before anything else
  pinMode(PIN_RELAY, OUTPUT); digitalWrite(PIN_RELAY, LOW);
  pinMode(PIN_BUZ, OUTPUT);   digitalWrite(PIN_BUZ, LOW);
  pinMode(PIN_LED_G, OUTPUT); pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_BTN, INPUT_PULLUP);
  setRelay(false);
  analogSetAttenuation(ADC_11db);
  analogReadResolution(12);

  SPI.begin(TFT_SCK, -1, TFT_SDA, TFT_CS);   // hardware SPI on our pins (the library then keeps them)
  tft.initR(INITR_BLACKTAB);
  tft.setSPISpeed(27000000);
  tft.setRotation(0);
  tft.fillScreen(BG);
  cv.setTextWrap(false);

  beep(150);
  Serial.println("\n=== GridGuard hardware test ===");
  Serial.println("Press the button to toggle the relay. Sensor midpoints should be about 1650 mV.");
}

uint32_t lastDraw = 0, lastLog = 0;

void loop() {
  pollButton();
  if (beepUntil && (int32_t)(millis() - beepUntil) >= 0) { digitalWrite(PIN_BUZ, LOW); beepUntil = 0; }

  if (millis() - lastDraw >= 250) {
    lastDraw = millis();
    readSensors();          // 200 ms; the button is checked again right after
    pollButton();
    draw();
  }
  if (millis() - lastLog >= 1000) {
    lastLog = millis();
    Serial.printf("V: mid %.0f mV ac %.1f mV (~%.1f V) | I: mid %.0f mV ac %.1f mV (~%.2f A) | relay %s | button %s\n",
                  chV.midMv, chV.rmsMv, chV.rmsMv * V_PER_MV, chI.midMv, chI.rmsMv, chI.rmsMv / MV_PER_A,
                  relayOn ? "ON" : "OFF", btnDown ? "DOWN" : "up");
  }
}
