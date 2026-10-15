/*
  GridGuard: ESP32 over/under-voltage and over-current protector with a relay cut-off
  ==================================================================================
  Measures the mains voltage (ZMPT101B) and load current (ACS712-20A) every 200 ms, opens the relay
  when a value stays outside its limit for the trip delay, and closes it again after the restart
  delay once the supply is back in range. Shows everything on a 1.77" colour TFT.

  BUTTON (GPIO13 to GND)
    short press      next screen (the screen otherwise stays where it is)
    hold 2 s         reset a trip now (refused while the supply is still out of range)
  A trip always brings the Home screen up.

  SCREENS   Home, Graph (last 60 s), Trips, Limits, Info

  POWER-UP  The relay stays open for the restart delay (10 s) and closes only if the voltage is in range.
            WiFi and the web page come in a later version.

  WIRING (docs/wiring.md)
    TFT       VCC 3V3, GND, CS 5, RST 4, DC 27, SDA 23, SCK 18, BLK 3V3
    ZMPT101B  OUT -> 10k -> GPIO34 node (20k + 100nF to GND)       VCC 5V
    ACS712    OUT -> 10k -> GPIO35 node (20k + 100nF to GND)       VCC 5V, IP+/IP- in the live after the relay
    Relay     IN GPIO26 (+10k to GND), H trigger, COM/NO in the live, VCC 5V
    Button    GPIO13 to GND        Buzzer GPIO25 via NPN        LEDs GPIO32 green, GPIO33 red (optional)

  ARDUINO IDE
    Board "ESP32 Dev Module". Libraries: "Adafruit GFX Library", "Adafruit ST7735 and ST7789 Library".
    Keep BaiJamjuree.h in this folder (second tab). Serial Monitor 115200, line ending "Newline".

  CALIBRATION (Serial Monitor until the web page exists; saved in flash)
    V 231.5     the multimeter reads 231.5 V: the voltage gain is corrected to match
    I 4.20      the clamp meter reads 4.20 A with a kettle or heater on: the current trim is corrected
    Z           measure the idle current noise (unplug the load first)
    F           flip the current direction (if power reads negative on a kettle)
    D           back to the default calibration
    ?           print the calibration
*/

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <Preferences.h>
#include "BaiJamjuree.h"     // Bai Jamjuree in three sizes (BaiS, BaiM, BaiL)

#define FW_VERSION "0.1.0"

// ---------------- types (kept above every function: the IDE adds its function declarations just
// before the first function, and they must already see these) ----------------
enum State { ARMED, TRIPPING, TRIPPED, WAITING };
enum Reason { R_NONE, R_OV, R_UV, R_OC, R_UC };
enum Page { HOME, GRAPH, TRIPS, LIMITS, INFO, PAGES };
struct Trip { Reason r; float val; uint32_t at; };
// declared here with their default arguments, so the IDE does not generate its own declarations
void text(const char *s, int x, int y, uint16_t col, const GFXfont *f = nullptr);
int  textW(const char *s, const GFXfont *f = nullptr);
void textR(const char *s, int xr, int y, uint16_t col, const GFXfont *f = nullptr);
void textC(const char *s, int y, uint16_t col, const GFXfont *f = nullptr);

// ---------------- pins ----------------
const int TFT_CS = 5, TFT_RST = 4, TFT_DC = 27;
const int PIN_V = 34, PIN_I = 35;
const int PIN_BTN = 13, PIN_BUZ = 25, PIN_RELAY = 26, PIN_LED_G = 32, PIN_LED_R = 33;

// ---------------- protection settings (GridGuard defaults; the web page will make them editable) ----------------
float OVV = 250;              // over-voltage, V
float UVV = 190;              // under-voltage, V
float OCV = 6.0;              // over-current, A (keep at 7 A or below: 10 A relay, 6 A socket)
float UCV = 0.5;              // under-current, A (only if UC_ON)
bool  UC_ON = false;
float TRIP_DELAY_S = 2;       // a fault must last this long
float RESTART_S = 10;         // wait after a trip (and at power-up) before closing the relay
float HYST = 0.02;            // voltage counts as normal again inside OVV x 0.98 and UVV x 1.02
bool  AUTO_RECONNECT = true;  // false: after a trip, wait for a 2 s button hold
const uint32_t UC_IGNORE_MS = 1500;   // after closing the relay, ignore under-current for this long
const uint32_t TRIPPED_SHOW_MS = 3000;

// ---------------- calibration (defaults; the live values are in flash) ----------------
const float V_GAIN_DEF = 0.946f, I_TRIM_DEF = 1.00f, I_NOISE_DEF = 0.195f;
const int   I_SIGN_DEF = -1;
float V_GAIN = V_GAIN_DEF;     // mains volts per mV at the pin
float I_TRIM = I_TRIM_DEF;     // clamp meter A / shown A
float I_NOISE = I_NOISE_DEF;   // idle current noise, A rms (removed in quadrature)
int   I_SIGN = I_SIGN_DEF;     // current direction
const float DIV = 2.0f / 3.0f, SENS_MV_PER_A = 100;   // 10k/20k divider, ACS712-20A
const float GATE_FACTOR = 1.35f, I_FLOOR = 0.08f, V_FLOOR = 20.0f;

// ---------------- sampling ----------------
const int      N = 1600;       // 1600 samples at 8 kHz = 200 ms = 10 mains cycles
const uint32_t TS_US = 125;
uint16_t vr[N], ir[N];

// ---------------- display ----------------
const int W = 128, H = 160;
Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16     cv(W, H);
Preferences     prefs;

#define rgb(c) ((uint16_t)((((c) >> 8) & 0xF800) | (((c) >> 5) & 0x07E0) | (((c) >> 3) & 0x001F)))
const uint16_t BG = rgb(0x0F1115), CARD = rgb(0x1D212A), LINE = rgb(0x2A303C);
const uint16_t FG = rgb(0xFFFFFF), FG2 = rgb(0xA7AFBD), MUTED = rgb(0x8A93A3);
const uint16_t VOLT = rgb(0x35D0FF), AMP = rgb(0xFF8A3D), POW = rgb(0xB6F24A);
const uint16_t OK_GREEN = rgb(0x3DDC84), WARN = rgb(0xFFB020), BAD = rgb(0xFF4D4D), BAD_DIM = rgb(0x8A1F1F), WAIT = rgb(0x4DA3FF);
const uint16_t SAFE_BAND = rgb(0x1E3A2A);

const char *REASON_LONG[]  = { "Power-up", "Over-voltage", "Under-voltage", "Over-current", "Under-current" };
const char *REASON_SHORT[] = { "START", "OVER-V", "UNDER-V", "OVER-I", "UNDER-I" };

// ---------------- live state ----------------
float    V = 0, I = 0, P = 0, PF = 0;     // last good measurement
float    rawI = 0, rawPF = 0;             // before the noise correction (for zeroing)
float    midV = 0, midI = 0;              // DC midpoints in mV (for the Info screen)
uint32_t badWindows = 0, goodWindows = 0;

State    st = WAITING;                    // power-up counts as a wait before the first close
uint32_t stAt = 0;
Reason   pending = R_NONE;
Reason   waitReason = R_NONE;             // what the current wait is for (R_NONE = power-up)
bool     relayOn = false;
uint32_t relayOnAt = 0, relaySwitches = 0;

Trip     trips[3];                        // newest first
int      tripCount = 0;

const int HN = 60;                        // graph: 60 s, one point per second
float    hv[HN], hi[HN];
int      hHead = 0, hCount = 0;

int      page = HOME;
char     toast[32] = "";
uint32_t toastUntil = 0;

// button (polled, also from inside the 200 ms sampling loop so presses are never missed)
bool     btnRaw = false, btnStable = false, btnLongFired = false;
uint32_t btnRawAt = 0, btnPressAt = 0;
bool     evShort = false, evLong = false;

// buzzer pattern: beeps of onMs, gaps of offMs
int      buzLeft = 0;
uint16_t buzOn = 0, buzOff = 0;
bool     buzState = false;
uint32_t buzAt = 0;

// ---------------- small helpers ----------------
void setRelay(bool on) {
  if (on != relayOn) relaySwitches++;
  relayOn = on;
  if (on) relayOnAt = millis();
  digitalWrite(PIN_RELAY, on ? HIGH : LOW);
}

void beep(int count, uint16_t onMs, uint16_t offMs) {
  buzLeft = count; buzOn = onMs; buzOff = offMs;
  buzState = true; buzAt = millis();
  digitalWrite(PIN_BUZ, HIGH);
}
void buzzerTick(uint32_t now) {
  if (!buzLeft) return;
  if (buzState && now - buzAt >= buzOn) {
    digitalWrite(PIN_BUZ, LOW); buzState = false; buzAt = now;
    if (--buzLeft == 0) return;
  } else if (!buzState && now - buzAt >= buzOff) {
    digitalWrite(PIN_BUZ, HIGH); buzState = true; buzAt = now;
  }
}

void showToast(const char *s) {
  strncpy(toast, s, sizeof(toast) - 1);
  toast[sizeof(toast) - 1] = 0;
  toastUntil = millis() + 2500;
}

// Debounced button: a press counts after 25 ms of steady level. Short press acts on release,
// a hold fires once after 2 s.
void buttonTick(uint32_t now) {
  bool raw = digitalRead(PIN_BTN) == LOW;
  if (raw != btnRaw) { btnRaw = raw; btnRawAt = now; }
  if (now - btnRawAt >= 25 && raw != btnStable) {
    btnStable = raw;
    if (raw) { btnPressAt = now; btnLongFired = false; }
    else if (!btnLongFired) evShort = true;
  }
  if (btnStable && !btnLongFired && now - btnPressAt >= 2000) { btnLongFired = true; evLong = true; }
}

void ledsUpdate(uint32_t now) {
  bool g = (st == ARMED);
  bool r = (st == TRIPPED || st == WAITING) || (st == TRIPPING && (now / 200) % 2);
  digitalWrite(PIN_LED_G, g ? HIGH : LOW);
  digitalWrite(PIN_LED_R, r ? HIGH : LOW);
}

// ---------------- measurement (same method as PowerScope) ----------------
bool measure() {
  uint32_t start = micros(), t = start;
  for (int k = 0; k < N; k++) {
    vr[k] = analogReadMilliVolts(PIN_V);
    ir[k] = analogReadMilliVolts(PIN_I);
    if ((k & 15) == 0) { uint32_t now = millis(); buttonTick(now); buzzerTick(now); }
    t += TS_US;
    while ((int32_t)(micros() - t) < 0) { }
  }
  float elapsedUs = (float)(micros() - start);
  if (elapsedUs < 196000.0f || elapsedUs > 204000.0f) { badWindows++; return false; }

  double vm = 0, im = 0;
  for (int k = 0; k < N; k++) { vm += vr[k]; im += ir[k]; }
  vm /= N; im /= N;
  midV = vm; midI = im;

  const double IPM = (1.0 / (DIV * SENS_MV_PER_A)) * I_TRIM;
  double v2 = 0, i2 = 0, pSum = 0;
  for (int k = 0; k < N; k++) {
    double v = (vr[k] - vm) * V_GAIN;
    double i = (ir[k] - im) * IPM;
    v2 += v * v; i2 += i * i; pSum += v * i;
  }
  float Vrms = sqrt(v2 / N);
  float Iraw = sqrt(i2 / N);
  float Praw = I_SIGN * (float)(pSum / N);
  if (Vrms < V_FLOOR) Vrms = 0;
  rawI = Iraw;
  rawPF = (Vrms * Iraw > 1.0f) ? fabs(Praw) / (Vrms * Iraw) : 0;

  // idle noise adds in quadrature; anything close to it is "no load"
  float gate = max(I_FLOOR, I_NOISE * GATE_FACTOR);
  float Irms = 0, Pw = 0;
  if (Iraw >= gate) { Irms = sqrt(max(0.0f, Iraw * Iraw - I_NOISE * I_NOISE)); Pw = Praw; }
  float S = Vrms * Irms;
  float pf = (S > 1.0f) ? constrain(fabs(Pw) / S, 0.0f, 1.0f) : 0;
  if (!isfinite(Vrms + Irms + Pw + pf)) { badWindows++; return false; }
  V = Vrms; I = Irms; P = Pw; PF = pf;
  goodWindows++;
  return true;
}

// ---------------- protection state machine ----------------
void recordTrip(Reason r, float val, uint32_t now) {
  for (int k = 2; k > 0; k--) trips[k] = trips[k - 1];
  trips[0] = { r, val, now };
  tripCount++;
}

bool voltageInBand() { return V <= OVV * (1 - HYST) && V >= UVV * (1 + HYST); }

void updateState(uint32_t now) {
  Reason f = R_NONE;
  float fv = 0;
  if (V > OVV) { f = R_OV; fv = V; }
  else if (V < UVV) { f = R_UV; fv = V; }
  else if (relayOn && I > OCV) { f = R_OC; fv = I; }
  else if (UC_ON && relayOn && now - relayOnAt > UC_IGNORE_MS && I < UCV) { f = R_UC; fv = I; }

  switch (st) {
    case ARMED:
      if (f) { st = TRIPPING; stAt = now; pending = f; }
      break;
    case TRIPPING:
      if (!f) st = ARMED;
      else if (now - stAt >= TRIP_DELAY_S * 1000) {
        setRelay(false);
        st = TRIPPED; stAt = now; waitReason = f;
        recordTrip(f, fv, now);
        page = HOME;                               // a trip always brings Home up
        beep(3, 150, 120);
        Serial.printf("[trip] %s %.2f\n", REASON_LONG[f], fv);
      }
      break;
    case TRIPPED:
      if (AUTO_RECONNECT && now - stAt >= TRIPPED_SHOW_MS) { st = WAITING; stAt = now; }
      break;
    case WAITING:
      if (!voltageInBand()) stAt = now;            // the countdown runs only while the supply is good
      if (now - stAt >= RESTART_S * 1000) {
        setRelay(true);
        st = ARMED;
        beep(1, 60, 0);
        Serial.println("[relay] closed");
      }
      break;
  }
}

void handleButton(uint32_t now) {
  if (evShort) {
    evShort = false;
    page = (page + 1) % PAGES;
    beep(1, 15, 0);
  }
  if (evLong) {
    evLong = false;
    if (st == TRIPPED || st == WAITING) {
      if (voltageInBand()) {
        setRelay(true); st = ARMED;
        beep(1, 60, 0);
        showToast("Reset: relay on");
        Serial.println("[button] reset, relay closed");
      } else {
        beep(2, 60, 60);
        showToast("Supply out of range");
      }
    } else {
      showToast("Hold resets a trip");
    }
  }
}

// ---------------- calibration over Serial ----------------
void saveCal() {
  prefs.putFloat("vgain", V_GAIN); prefs.putFloat("itrim", I_TRIM);
  prefs.putFloat("inoise", I_NOISE); prefs.putInt("isign", I_SIGN);
}
void printCal() {
  Serial.printf("[cal] V_GAIN %.4f  I_TRIM %.4f  I_NOISE %.3f A  I_SIGN %d\n", V_GAIN, I_TRIM, I_NOISE, I_SIGN);
}
void zeroNoise() {
  Serial.println("[zero] measuring idle noise, keep the load unplugged...");
  double sum = 0, sumPF = 0;
  int ok = 0;
  for (int w = 0; w < 40 && ok < 20; w++) if (measure()) { sum += rawI; sumPF += rawPF; ok++; }
  if (ok < 15) { Serial.println("[zero] too few clean windows, try again"); return; }
  float mi = sum / ok, mpf = sumPF / ok;
  if (mi > 0.30f || mpf > 0.35f) { Serial.printf("[zero] a load seems connected (%.3f A), nothing changed\n", mi); return; }
  I_NOISE = mi; saveCal();
  Serial.printf("[zero] idle noise %.3f A saved\n", mi);
}
void serialCommands() {
  static char line[24];
  static int n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c != '\n' && c != '\r') { if (n < (int)sizeof(line) - 1) line[n++] = c; continue; }
    line[n] = 0;
    if (!n) continue;
    n = 0;
    char cmd = toupper(line[0]);
    float x = atof(line + 1);
    if (cmd == 'V') {
      if (V > 100 && x > 150 && x < 300) { V_GAIN *= x / V; saveCal(); printCal(); }
      else Serial.println("[cal] need mains connected and a reading like: V 231.5");
    } else if (cmd == 'I') {
      if (I > 0.5f && x > 0.1f && x < 10) { float k = x / I; I_TRIM *= k; I_NOISE *= k; saveCal(); printCal(); }
      else Serial.println("[cal] need a load of at least 0.5 A and a reading like: I 4.20");
    } else if (cmd == 'Z') zeroNoise();
    else if (cmd == 'F') { I_SIGN = -I_SIGN; saveCal(); printCal(); }
    else if (cmd == 'D') { V_GAIN = V_GAIN_DEF; I_TRIM = I_TRIM_DEF; I_NOISE = I_NOISE_DEF; I_SIGN = I_SIGN_DEF; saveCal(); printCal(); }
    else if (cmd == '?') printCal();
    else Serial.println("[cal] commands: V <volts>, I <amps>, Z, F, D, ?");
  }
}

// ---------------- drawing helpers ----------------
// f = nullptr means the small font, and then y is the top of the capitals; for BaiM and BaiL y is the baseline
void text(const char *s, int x, int y, uint16_t col, const GFXfont *f) {
  if (!f) { f = &BaiS; y += BAIS_CAP; }
  cv.setFont(f); cv.setTextColor(col); cv.setCursor(x, y); cv.print(s);
}
int textW(const char *s, const GFXfont *f) {
  int16_t x1, y1; uint16_t w, h;
  cv.setFont(f ? f : &BaiS); cv.getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
  return x1 + w;
}
void textR(const char *s, int xr, int y, uint16_t col, const GFXfont *f) { text(s, xr - textW(s, f), y, col, f); }
void textC(const char *s, int y, uint16_t col, const GFXfont *f) { text(s, (W - textW(s, f)) / 2, y, col, f); }
void dashedH(int x0, int x1, int y, uint16_t col) { for (int x = x0; x < x1; x += 4) cv.drawFastHLine(x, y, 2, col); }
const char *ago(uint32_t ms, char *b, size_t n) {
  uint32_t s = ms / 1000;
  if (s < 60) snprintf(b, n, "%lu s ago", (unsigned long)s);
  else if (s < 3600) snprintf(b, n, "%lu min ago", (unsigned long)(s / 60));
  else snprintf(b, n, "%lu h ago", (unsigned long)(s / 3600));
  return b;
}
void pageTitle(const char *t) {
  text(t, 4, 4, FG);
  for (int p = 1; p < PAGES; p++) {
    int x = W - 6 - (PAGES - 1 - p) * 7;
    if (p == page) cv.fillCircle(x, 7, 2, FG); else cv.drawCircle(x, 7, 2, MUTED);
  }
  cv.drawFastHLine(0, 15, W, LINE);
}

// ---------------- screen 1: Home ----------------
void statusBar(uint32_t now) {
  uint16_t bg = OK_GREEN;
  char s[24] = "ARMED";
  if (st == TRIPPING) {
    bg = WARN;
    snprintf(s, sizeof(s), "TRIP IN %.1fs", max(0.0f, TRIP_DELAY_S - (now - stAt) / 1000.0f));
  } else if (st == TRIPPED) {
    bg = ((now / 400) % 2) ? BAD : BAD_DIM;
    strcpy(s, "TRIPPED");
  } else if (st == WAITING) {
    bg = WAIT;
    if (voltageInBand()) snprintf(s, sizeof(s), "%s %ds", waitReason ? "WAITING" : "STARTING",
                                  (int)ceil(max(0.0f, RESTART_S - (now - stAt) / 1000.0f)));
    else strcpy(s, "BAD SUPPLY");
  }
  cv.fillRect(0, 0, W, 22, bg);
  textC(s, 16, bg == BAD_DIM ? FG : BG, &BaiM);
}
void meterV(int y) {
  const float lo = UVV - 20, hi = OVV + 20;
  auto X = [&](float v) { return 4 + (int)((constrain(v, lo, hi) - lo) / (hi - lo) * (W - 8)); };
  cv.fillRect(4, y, W - 8, 6, CARD);
  cv.fillRect(X(UVV), y, X(OVV) - X(UVV), 6, SAFE_BAND);
  cv.drawFastVLine(X(UVV), y - 2, 10, BAD);
  cv.drawFastVLine(X(OVV), y - 2, 10, BAD);
  cv.fillRect(X(V) - 1, y - 3, 3, 12, (V > OVV || V < UVV) ? BAD : FG);
  char b[8];
  snprintf(b, sizeof(b), "%.0f", UVV); text(b, X(UVV) - textW(b) / 2, y + 10, MUTED);
  snprintf(b, sizeof(b), "%.0f", OVV); text(b, X(OVV) - textW(b) / 2, y + 10, MUTED);
}
void meterI(int y) {
  const float hi = OCV * 1.25f;
  auto X = [&](float v) { return 4 + (int)(constrain(v, 0.0f, hi) / hi * (W - 8)); };
  cv.fillRect(4, y, W - 8, 6, CARD);
  uint16_t c = I > OCV ? BAD : (I > OCV * 0.9f ? WARN : AMP);
  if (I > 0) cv.fillRect(4, y, X(I) - 4, 6, c);
  cv.drawFastVLine(X(OCV), y - 2, 10, BAD);
  char b[8];
  text("0", 4, y + 9, MUTED);
  snprintf(b, sizeof(b), "%.1f", OCV); text(b, X(OCV) - textW(b) / 2, y + 9, MUTED);
}
void drawHome(uint32_t now) {
  char b[32];
  statusBar(now);
  text("VOLTAGE", 4, 25, MUTED);
  snprintf(b, sizeof(b), "%.1f", V);
  text(b, 4, 60, (V > OVV || V < UVV) ? BAD : VOLT, &BaiL);
  textR("V", W - 4, 60, FG2, &BaiM);
  meterV(65);
  text("CURRENT", 4, 88, MUTED);
  snprintf(b, sizeof(b), "%.2f", I);
  text(b, 4, 122, I > OCV ? BAD : AMP, &BaiL);
  textR("A", W - 4, 122, FG2, &BaiM);
  meterI(127);
  cv.drawFastHLine(0, 148, W, LINE);
  if ((st == TRIPPED || st == WAITING) && waitReason != R_NONE) {
    const Trip &t = trips[0];
    snprintf(b, sizeof(b), (t.r == R_OC || t.r == R_UC) ? "%s %.1fA" : "%s %.0fV", REASON_SHORT[t.r], t.val);
    text(b, 4, 151, BAD);
  } else {
    snprintf(b, sizeof(b), "%.0f W", fabs(P));
    text(b, 4, 151, POW);
  }
  textR(relayOn ? "RELAY ON" : "OFF", W - 4, 151, relayOn ? OK_GREEN : BAD);
}

// ---------------- screen 2: Graph ----------------
void chart(int y0, int h, const float *ring, float lo, float hi, uint16_t col, const float *limits, int nl) {
  cv.drawRect(0, y0, W, h, LINE);
  auto Y = [&](float v) { return y0 + h - 2 - (int)((constrain(v, lo, hi) - lo) / (hi - lo) * (h - 4)); };
  char b[16];
  for (int k = 0; k < nl; k++) {
    dashedH(1, W - 1, Y(limits[k]), BAD);
    snprintf(b, sizeof(b), "%g", limits[k]);
    textR(b, W - 3, Y(limits[k]) + (limits[k] > (lo + hi) / 2 ? 2 : -9), BAD);
  }
  int px = -1, py = 0;
  for (int k = 0; k < hCount; k++) {
    int idx = (hHead - hCount + k + HN) % HN;
    int x = 1 + (int)(k * (W - 3) / (float)(HN - 1));
    int y = Y(ring[idx]);
    if (px >= 0) { cv.drawLine(px, py, x, y, col); cv.drawLine(px, py + 1, x, y + 1, col); }
    px = x; py = y;
  }
}
void drawGraph() {
  char b[24];
  pageTitle("LAST 60 s");
  snprintf(b, sizeof(b), "VOLTAGE  %.1f V", V);
  text(b, 4, 20, VOLT);
  float vl[] = { OVV, UVV };
  chart(30, 54, hv, UVV - 25, OVV + 25, VOLT, vl, 2);
  snprintf(b, sizeof(b), "CURRENT  %.2f A", I);
  text(b, 4, 92, AMP);
  float il[] = { OCV };
  chart(102, 54, hi, 0, OCV * 1.3f, AMP, il, 1);
}

// ---------------- screen 3: Trips ----------------
void drawTrips(uint32_t now) {
  char b[32], a[16];
  pageTitle("TRIPS");
  if (!tripCount) { textC("No trips yet", 70, OK_GREEN, &BaiM); return; }
  const Trip &t = trips[0];
  text("LAST TRIP", 4, 21, MUTED);
  text(REASON_LONG[t.r], 4, 46, BAD, &BaiM);
  snprintf(b, sizeof(b), (t.r == R_OC || t.r == R_UC) ? "%.1f A for %.0f s" : "%.0f V for %.0f s", t.val, TRIP_DELAY_S);
  text(b, 4, 53, FG);
  text(ago(now - t.at, a, sizeof(a)), 4, 65, FG2);
  cv.drawFastHLine(0, 78, W, LINE);
  snprintf(b, sizeof(b), "Trips total: %d", tripCount);
  text(b, 4, 84, FG);
  int y = 100;
  for (int k = 1; k < 3 && k < tripCount; k++, y += 14) {
    const Trip &o = trips[k];
    snprintf(b, sizeof(b), (o.r == R_OC || o.r == R_UC) ? "%s %.1fA" : "%s %.0fV", REASON_SHORT[o.r], o.val);
    text(b, 4, y, FG2);
    uint32_t sec = (now - o.at) / 1000;
    if (sec < 60) snprintf(a, sizeof(a), "%lus", (unsigned long)sec);
    else if (sec < 3600) snprintf(a, sizeof(a), "%lum", (unsigned long)(sec / 60));
    else snprintf(a, sizeof(a), "%luh", (unsigned long)(sec / 3600));
    textR(a, W - 4, y, MUTED);
  }
}

// ---------------- screen 4: Limits ----------------
void row(int y, const char *label, const char *val, uint16_t col) {
  text(label, 4, y, MUTED);
  textR(val, W - 4, y, col);
  cv.drawFastHLine(4, y + 12, W - 8, CARD);
}
void drawLimits() {
  char b[16];
  pageTitle("LIMITS");
  snprintf(b, sizeof(b), "%.0f V", OVV);          row(22, "Over-voltage", b, VOLT);
  snprintf(b, sizeof(b), "%.0f V", UVV);          row(40, "Under-voltage", b, VOLT);
  snprintf(b, sizeof(b), "%.1f A", OCV);          row(58, "Over-current", b, AMP);
  if (UC_ON) snprintf(b, sizeof(b), "%.1f A", UCV); else strcpy(b, "off");
  row(76, "Under-current", b, UC_ON ? AMP : MUTED);
  snprintf(b, sizeof(b), "%.0f s", TRIP_DELAY_S); row(94, "Trip delay", b, FG);
  snprintf(b, sizeof(b), "%.0f s", RESTART_S);    row(112, "Restart delay", b, FG);
  row(130, "After a trip", AUTO_RECONNECT ? "AUTO" : "HOLD", FG);
  textC("Hold button 2 s to reset", 148, MUTED);
}

// ---------------- screen 5: Info ----------------
void drawInfo(uint32_t now) {
  char b[32];
  pageTitle("INFO");
  uint32_t s = now / 1000;
  snprintf(b, sizeof(b), "%luh %02lum %02lus", (unsigned long)(s / 3600), (unsigned long)(s / 60 % 60), (unsigned long)(s % 60));
  row(22, "Up for", b, FG);
  snprintf(b, sizeof(b), "%.2f", PF);              row(40, "Power factor", b, POW);
  snprintf(b, sizeof(b), "%lu", (unsigned long)relaySwitches); row(58, "Relay switches", b, FG);
  snprintf(b, sizeof(b), "%.0f / %.0f", midV, midI); row(76, "Mid mV", b, FG2);
  snprintf(b, sizeof(b), "%.3f", V_GAIN);          row(94, "Voltage gain", b, FG2);
  snprintf(b, sizeof(b), "%.3f", I_TRIM);          row(112, "Current trim", b, FG2);
  row(130, "Firmware", FW_VERSION, FG2);
  textC("WiFi: next version", 148, MUTED);
}

// ---------------- frame ----------------
void render(uint32_t now) {
  cv.fillScreen(BG);
  switch (page) {
    case HOME:   drawHome(now);  break;
    case GRAPH:  drawGraph();    break;
    case TRIPS:  drawTrips(now); break;
    case LIMITS: drawLimits();   break;
    default:     drawInfo(now);  break;
  }
  if (toastUntil && (int32_t)(toastUntil - now) > 0) {
    cv.fillRoundRect(4, 132, W - 8, 24, 4, CARD);
    cv.drawRoundRect(4, 132, W - 8, 24, 4, FG2);
    textC(toast, 140, FG);
  }
  tft.drawRGBBitmap(0, 0, cv.getBuffer(), W, H);
}

// ---------------- setup / loop ----------------
uint32_t lastHist = 0, lastLog = 0;

void setup() {
  // relay open before anything else
  pinMode(PIN_RELAY, OUTPUT); digitalWrite(PIN_RELAY, LOW);
  pinMode(PIN_BUZ, OUTPUT);   digitalWrite(PIN_BUZ, LOW);
  pinMode(PIN_LED_G, OUTPUT); pinMode(PIN_LED_R, OUTPUT);
  pinMode(PIN_BTN, INPUT_PULLUP);
  Serial.begin(115200);
  analogSetAttenuation(ADC_11db);
  analogReadResolution(12);

  prefs.begin("gridguard", false);
  V_GAIN = prefs.getFloat("vgain", V_GAIN_DEF);
  I_TRIM = prefs.getFloat("itrim", I_TRIM_DEF);
  I_NOISE = prefs.getFloat("inoise", I_NOISE_DEF);
  I_SIGN = prefs.getInt("isign", I_SIGN_DEF) < 0 ? -1 : 1;

  tft.initR(INITR_BLACKTAB);
  tft.setSPISpeed(27000000);
  tft.setRotation(0);
  tft.fillScreen(BG);
  cv.setTextWrap(false);

  st = WAITING; stAt = millis(); waitReason = R_NONE;
  beep(1, 80, 0);
  Serial.printf("\n=== GridGuard v%s ===\n", FW_VERSION);
  Serial.printf("Limits: OVV %.0f V, UVV %.0f V, OCV %.1f A, trip delay %.0f s, restart %.0f s\n", OVV, UVV, OCV, TRIP_DELAY_S, RESTART_S);
  printCal();
  Serial.println("Calibration commands: V <volts>, I <amps>, Z, F, D, ?");
}

void loop() {
  serialCommands();
  measure();                       // 200 ms; the last good values stay if a window is skipped
  uint32_t now = millis();
  buttonTick(now);
  buzzerTick(now);
  updateState(now);
  handleButton(now);
  ledsUpdate(now);

  if (now - lastHist >= 1000) {
    lastHist = now;
    hv[hHead] = V; hi[hHead] = I;
    hHead = (hHead + 1) % HN;
    if (hCount < HN) hCount++;
  }
  render(now);

  if (now - lastLog >= 1000) {
    lastLog = now;
    const char *sn[] = { "ARMED", "TRIPPING", "TRIPPED", "WAITING" };
    Serial.printf("V %.1f | I %.3f (raw %.3f) | P %.0f W | PF %.2f | %s | relay %s\n",
                  V, I, rawI, P, PF, sn[st], relayOn ? "ON" : "OFF");
  }
}
