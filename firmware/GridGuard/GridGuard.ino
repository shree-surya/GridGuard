/*
  GridGuard: ESP32 over/under-voltage, over-current and over-power protector with a relay cut-off
  ===============================================================================================
  Measures the mains voltage (ZMPT101B) and load current (ACS712-20A) every 200 ms, opens the relay
  when a value stays outside its limit for the trip delay, and closes it again after the restart
  delay once the supply is back in range. Shows everything on a 1.77" colour TFT and on its own
  WiFi hotspot:

      WiFi  "GridGuard"   password  "gridguard"      ->  http://192.168.4.1

  WEB PAGE   live voltage, current, power, PF and relay state; a 60 s graph; the trip log;
             the limits (over-voltage, under-voltage, over-current, over-power), saved in flash;
             a relay ON / OFF button; calibration (gain, offset, zero, direction), saved in flash.
             Protection stays active while the relay is on.

  RELAY ON WHILE THE SUPPLY IS OUT OF RANGE   allowed by hand (web or button): this is an OVERRIDE.
             Over- and under-voltage are ignored until the voltage comes back into range, then
             normal protection resumes by itself. Over-current and over-power stay active.

  BUTTON (GPIO13 to GND)
    short press      next screen (the screen otherwise stays where it is)
    hold 2 s         relay on (resets a trip; out-of-range supply = override, see above),
                     or relay off if it is on (manual OFF, stays off until turned on again)
  A trip always brings the Home screen up.

  POWER-UP   The relay stays open for the restart delay (10 s) and closes only if the voltage is in range.

  WIRING (docs/wiring.md)
    TFT       VCC 3V3, GND, CS 5, RST 4, DC 27, SDA 19, SCK 23, BLK 3V3   (pins set just below)
    ZMPT101B  OUT -> 10k -> GPIO34 node (20k + 100nF to GND)       VCC 5V
    ACS712    OUT -> 10k -> GPIO35 node (20k + 100nF to GND)       VCC 5V, IP+/IP- in the live after the relay
    Relay     IN GPIO26 (+10k to GND), H trigger, COM/NO in the live, VCC 5V
    Button    GPIO13 to GND        Buzzer GPIO25 via NPN        LEDs GPIO32 green, GPIO33 red (optional)

  ARDUINO IDE
    Board "ESP32 Dev Module", Partition Scheme "Huge APP (3MB No OTA/1MB SPIFFS)".
    Libraries: "Adafruit GFX Library", "Adafruit ST7735 and ST7789 Library" (Library Manager),
               ESPAsyncWebServer and AsyncTCP (ESP32Async, GitHub -> Download ZIP -> Add .ZIP Library).
    Keep BaiJamjuree.h and web_index.h in this folder. Serial Monitor 115200, line ending "Newline".
    The page is web/index.html; after editing it run  python tools/embed_web.py  to rebuild web_index.h.

  CALIBRATION (web page "Calibration", or the Serial Monitor; saved in flash)
    V 231.5     the multimeter reads 231.5 V            I 4.20    the clamp meter reads 4.20 A (kettle on)
    O -2.5      voltage offset in volts                  Z         zero offset (idle current, nothing plugged in)
    F           flip the current direction               D         default calibration      ?   print it
*/

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <Preferences.h>
#include "BaiJamjuree.h"     // Bai Jamjuree in three sizes (BaiS, BaiM, BaiL)
#include "web_index.h"       // the web page, gzip, made by tools/embed_web.py

#define FW_VERSION "0.3.0"

// ---------------- types (kept above every function: the IDE adds its function declarations just
// before the first function, and they must already see these) ----------------
enum State { ARMED, TRIPPING, TRIPPED, WAITING, MANUAL_OFF };
enum Reason { R_NONE, R_OV, R_UV, R_OC, R_OP };
enum Page { HOME, GRAPH, TRIPS, LIMITS, CONNECT, PAGES };
struct Trip { Reason r; float val; uint32_t at; };
struct Limits { float ovv, uvv, ocv, opp; };
struct Cal { float vgain, voff, itrim, inoise; int isign; };
// declared here with their default arguments, so the IDE does not generate its own declarations
void text(const char *s, int x, int y, uint16_t col, const GFXfont *f = nullptr);
int  textW(const char *s, const GFXfont *f = nullptr);
void textR(const char *s, int xr, int y, uint16_t col, const GFXfont *f = nullptr);
void textC(const char *s, int y, uint16_t col, const GFXfont *f = nullptr);

// ---------------- pins ----------------
const int TFT_CS = 5, TFT_RST = 4, TFT_DC = 27;
const int TFT_SDA = 19, TFT_SCK = 23;   // SPI data (MOSI) and clock: any free GPIOs work, set them to where your wires go
const int PIN_V = 34, PIN_I = 35;
const int PIN_BTN = 13, PIN_BUZ = 25, PIN_RELAY = 26, PIN_LED_G = 32, PIN_LED_R = 33;

// ---------------- WiFi ----------------
const char *WIFI_SSID = "GridGuard";
const char *WIFI_PASS = "gridguard";    // WPA2: at least 8 characters

// ---------------- protection settings (limits set from the web page, saved in flash) ----------------
const Limits LIM_DEF = { 250, 190, 6.0, 1300 };
const Limits LIM_MIN = { 200, 150, 0.5, 50 };
const Limits LIM_MAX = { 280, 220, 7.0, 1600 };   // 7 A / 1600 W: 10 A relay, 6 A socket
Limits LIM = LIM_DEF;
float TRIP_DELAY_S = 2;       // a fault must last this long
float RESTART_S = 10;         // wait after a trip (and at power-up) before closing the relay
float HYST = 0.02;            // voltage counts as normal again inside OVV x 0.98 and UVV x 1.02
const uint32_t TRIPPED_SHOW_MS = 3000;

// ---------------- calibration (defaults; the live values are in flash) ----------------
const float V_GAIN_DEF = 0.946f, V_OFF_DEF = 0, I_TRIM_DEF = 1.00f, I_NOISE_DEF = 0.195f;
const int   I_SIGN_DEF = -1;
const Cal   CAL_DEF = { V_GAIN_DEF, V_OFF_DEF, I_TRIM_DEF, I_NOISE_DEF, I_SIGN_DEF };
float V_GAIN = V_GAIN_DEF;     // mains volts per mV at the pin
float V_OFF = V_OFF_DEF;       // volts added to the reading (after the gain)
float I_TRIM = I_TRIM_DEF;     // clamp meter A / shown A
float I_NOISE = I_NOISE_DEF;   // idle current noise, A rms (removed in quadrature)
int   I_SIGN = I_SIGN_DEF;     // current direction
const float DIV = 2.0f / 3.0f, SENS_MV_PER_A = 100;   // 10k/20k divider, ACS712-20A
const float GATE_FACTOR = 1.35f, I_FLOOR = 0.08f, V_FLOOR = 20.0f;

// ---------------- sampling ----------------
const int      N = 1600;       // 1600 samples at 8 kHz = 200 ms = 10 mains cycles
const uint32_t TS_US = 125;
uint16_t vr[N], ir[N];

// ---------------- display, web, flash ----------------
const int W = 128, H = 160;
Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);
GFXcanvas16     cv(W, H);
Preferences     prefs;
AsyncWebServer  server(80);
DNSServer       dns;

// GridGuard palette (the web page uses the same): amber brand, periwinkle volts, rose amps, mint watts
#define rgb(c) ((uint16_t)((((c) >> 8) & 0xF800) | (((c) >> 5) & 0x07E0) | (((c) >> 3) & 0x001F)))
const uint16_t BG = rgb(0x0D0E11), CARD = rgb(0x1F2128), LINE = rgb(0x2C2F38);
const uint16_t FG = rgb(0xFFFFFF), FG2 = rgb(0xB4B8C2), MUTED = rgb(0x8C909B);
const uint16_t BRAND = rgb(0xFFC233);
const uint16_t VOLT = rgb(0x8FA8FF), AMP = rgb(0xFF8FA3), POW = rgb(0x47E5BC);
const uint16_t OK_GREEN = rgb(0x3DDC84), WARN = rgb(0xFF9F1C), BAD = rgb(0xFF4D4D), BAD_DIM = rgb(0x8A1F1F), WAIT = rgb(0x4DA3FF), OFFGREY = rgb(0x6B7080);
const uint16_t SAFE_BAND = rgb(0x1E3A2A);

const char *REASON_LONG[]  = { "Power-up", "Over-voltage", "Under-voltage", "Over-current", "Over-power" };
const char *REASON_SHORT[] = { "START", "OVER-V", "UNDER-V", "OVER-I", "OVER-P" };
const char *STATE_NAME[]   = { "ARMED", "TRIPPING", "TRIPPED", "WAITING", "OFF" };

// ---------------- live state ----------------
float    V = 0, I = 0, P = 0, PF = 0;     // last good measurement
float    rawI = 0, rawPF = 0;             // before the noise correction (for zeroing)
uint32_t badWindows = 0, goodWindows = 0;

State    st = WAITING;                    // power-up counts as a wait before the first close
uint32_t stAt = 0;
Reason   pending = R_NONE;                // fault being timed while TRIPPING
Reason   waitReason = R_NONE;             // what the current wait is for (R_NONE = power-up)
bool     relayOn = false;
uint32_t relayOnAt = 0, relaySwitches = 0;

const int TRIPS_KEPT = 10;
Trip     trips[TRIPS_KEPT];               // newest first, RAM only
int      tripCount = 0;

const int HN = 60;                        // 60 s history, one point per second
float    hv[HN], hi[HN], hp[HN];
int      hHead = 0, hCount = 0;

int      page = HOME;
char     toast[32] = "";
uint32_t toastUntil = 0;

// requests from the web page, applied in loop()
volatile bool limPending = false;
Limits        limNext;
volatile int  relayCmd = -1;              // -1 none, 0 off, 1 on
volatile bool calPending = false;
Cal           calNext;
volatile bool zeroReq = false;

bool     vOverride = false;               // relay turned on by hand while the voltage was out of range

// zero offset measurement (idle current noise), runs over 20 windows in loop()
int      zeroState = 0;                   // 0 never, 1 running, 2 done, 3 refused
char     zeroMsg[64] = "";
int      zeroOk = 0, zeroTries = 0;
double   zeroSum = 0, zeroSumPF = 0;

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
  else vOverride = false;
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
  // a WiFi interrupt can stretch the window; skip it rather than act on a wrong value
  if (elapsedUs < 196000.0f || elapsedUs > 204000.0f) { badWindows++; return false; }

  double vm = 0, im = 0;
  for (int k = 0; k < N; k++) { vm += vr[k]; im += ir[k]; }
  vm /= N; im /= N;

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
  else if (V_OFF != 0) { float k = max(0.0f, Vrms + V_OFF) / Vrms; Vrms *= k; Praw *= k; }
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
  for (int k = TRIPS_KEPT - 1; k > 0; k--) trips[k] = trips[k - 1];
  trips[0] = { r, val, now };
  tripCount++;
}

bool voltageInBand() { return V <= LIM.ovv * (1 - HYST) && V >= LIM.uvv * (1 + HYST); }

void updateState(uint32_t now) {
  if (vOverride && voltageInBand()) {
    vOverride = false;
    Serial.println("[relay] supply back in range, voltage protection active again");
  }
  Reason f = R_NONE;
  float fv = 0;
  bool vCheck = !vOverride;                        // an override ignores the voltage limits
  if (vCheck && V > LIM.ovv) { f = R_OV; fv = V; }
  else if (vCheck && V < LIM.uvv) { f = R_UV; fv = V; }
  else if (relayOn && I > LIM.ocv) { f = R_OC; fv = I; }
  else if (relayOn && fabs(P) > LIM.opp) { f = R_OP; fv = fabs(P); }

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
      if (now - stAt >= TRIPPED_SHOW_MS) { st = WAITING; stAt = now; }
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
    case MANUAL_OFF:
      break;                                       // stays off until turned on by hand
  }
}

// Turns the relay on by hand (button hold or web page). With the supply out of range this is an
// override: the voltage limits are ignored until the voltage is back in range.
void manualOn(bool fromWeb) {
  if (relayOn) return;
  bool out = !voltageInBand();
  setRelay(true);
  st = ARMED;
  vOverride = out;
  beep(out ? 2 : 1, 60, 60);
  showToast(out ? "ON: voltage override" : (fromWeb ? "Relay on (web)" : "Relay on"));
  Serial.printf("[relay] on by hand%s\n", out ? ", supply out of range: OVERRIDE (voltage limits paused)" : "");
}
void manualOff() {
  setRelay(false);
  st = MANUAL_OFF;
  beep(1, 60, 0);
  Serial.println("[relay] off by hand");
}

void handleButton() {
  if (evShort) {
    evShort = false;
    page = (page + 1) % PAGES;
    beep(1, 15, 0);
  }
  if (evLong) {
    evLong = false;
    if (relayOn) { manualOff(); showToast("Relay off (manual)"); }
    else manualOn(false);
  }
}

// Requests from the web page, applied here between two measuring windows.
void handleWeb() {
  if (limPending) {
    limPending = false;
    LIM = limNext;
    prefs.putBytes("limits", &LIM, sizeof(LIM));
    Serial.printf("[limits] OVV %.0f V  UVV %.0f V  OCV %.2f A  OPP %.0f W, saved\n", LIM.ovv, LIM.uvv, LIM.ocv, LIM.opp);
    showToast("Limits saved");
  }
  int cmd = relayCmd;
  if (cmd >= 0) {
    relayCmd = -1;
    if (cmd == 1) manualOn(true);
    else { manualOff(); showToast("Relay off (web)"); }
  }
  if (calPending) {
    calPending = false;
    applyCal(calNext);
    saveCal();
    printCal();
    showToast("Calibration saved");
  }
  if (zeroReq) { zeroReq = false; zeroStart(); }
}

// Checks limits typed on the page. Returns nullptr if they are fine, otherwise the message to show.
const char *checkLimits(const Limits &l) {
  static char e[64];
  if (!(l.ovv >= LIM_MIN.ovv && l.ovv <= LIM_MAX.ovv)) { snprintf(e, sizeof(e), "Over-voltage must be %.0f to %.0f V", LIM_MIN.ovv, LIM_MAX.ovv); return e; }
  if (!(l.uvv >= LIM_MIN.uvv && l.uvv <= LIM_MAX.uvv)) { snprintf(e, sizeof(e), "Under-voltage must be %.0f to %.0f V", LIM_MIN.uvv, LIM_MAX.uvv); return e; }
  if (l.ovv - l.uvv < 20) return "Over-voltage must be at least 20 V above under-voltage";
  if (!(l.ocv >= LIM_MIN.ocv && l.ocv <= LIM_MAX.ocv)) { snprintf(e, sizeof(e), "Over-current must be %.1f to %.1f A", LIM_MIN.ocv, LIM_MAX.ocv); return e; }
  if (!(l.opp >= LIM_MIN.opp && l.opp <= LIM_MAX.opp)) { snprintf(e, sizeof(e), "Over-power must be %.0f to %.0f W", LIM_MIN.opp, LIM_MAX.opp); return e; }
  return nullptr;
}

// ---------------- calibration (web page and Serial) ----------------
Cal calNow() {
  return { V_GAIN, V_OFF, I_TRIM, I_NOISE, I_SIGN };
}
void applyCal(const Cal &c) {
  V_GAIN = c.vgain; V_OFF = c.voff; I_TRIM = c.itrim; I_NOISE = c.inoise; I_SIGN = c.isign;
}
void saveCal() {
  prefs.putFloat("vgain", V_GAIN); prefs.putFloat("voff", V_OFF); prefs.putFloat("itrim", I_TRIM);
  prefs.putFloat("inoise", I_NOISE); prefs.putInt("isign", I_SIGN);
}
void printCal() {
  Serial.printf("[cal] V_GAIN %.4f  V_OFF %+.1f V  I_TRIM %.4f  I_ZERO %.3f A  I_SIGN %d\n", V_GAIN, V_OFF, I_TRIM, I_NOISE, I_SIGN);
}
// Returns nullptr if the values are usable, otherwise the message to show.
const char *checkCal(const Cal &c) {
  if (!(c.vgain >= 0.5f && c.vgain <= 1.5f)) return "Voltage gain must be 0.5 to 1.5";
  if (!(c.voff >= -30 && c.voff <= 30)) return "Voltage offset must be -30 to 30 V";
  if (!(c.itrim >= 0.5f && c.itrim <= 2.0f)) return "Current trim must be 0.5 to 2.0";
  if (!(c.inoise >= 0 && c.inoise <= 1.0f)) return "Zero offset must be 0 to 1.0 A";
  if (c.isign != 1 && c.isign != -1) return "Direction must be 1 or -1";
  return nullptr;
}
// Gain so the shown voltage matches the multimeter (the offset stays as it is).
const char *calFromVolts(Cal &c, float meter) {
  if (!(V > 100)) return "Needs mains connected (reading above 100 V)";
  if (!(meter >= 150 && meter <= 300)) return "Multimeter reading must be 150 to 300 V";
  if (V - c.voff < 1) return "Reading too low for this offset";
  c.vgain *= (meter - c.voff) / (V - c.voff);
  return nullptr;
}
// Trim so the shown current matches the clamp meter; the zero offset was in amps, so it scales too.
const char *calFromAmps(Cal &c, float meter) {
  if (!(I >= 0.5f)) return "Needs a load drawing at least 0.5 A (kettle or heater)";
  if (!(meter >= 0.1f && meter <= 10)) return "Clamp meter reading must be 0.1 to 10 A";
  float k = meter / I;
  c.itrim *= k; c.inoise *= k;
  return nullptr;
}
// Zero offset: the current the sensor shows with nothing flowing, measured over 20 clean windows.
void zeroStart() {
  zeroState = 1; zeroOk = 0; zeroTries = 0; zeroSum = 0; zeroSumPF = 0;
  strcpy(zeroMsg, "Measuring, keep the load unplugged");
  Serial.println("[zero] measuring the zero offset, keep the load unplugged...");
}
void zeroTick(bool good) {
  if (zeroState != 1 || zeroReq) return;       // zeroReq: a new run is about to start, its counters are not reset yet
  zeroTries++;
  if (good) { zeroSum += rawI; zeroSumPF += rawPF; zeroOk++; }
  if (zeroOk < 20 && zeroTries < 40) return;
  if (zeroOk < 15) { zeroState = 3; strcpy(zeroMsg, "Too few clean readings, try again"); }
  else {
    float mi = zeroSum / zeroOk, mpf = zeroSumPF / zeroOk;
    if (mi > 0.30f || mpf > 0.35f) {
      zeroState = 3;
      snprintf(zeroMsg, sizeof(zeroMsg), "A load seems connected (%.2f A), nothing changed", mi);
    } else {
      I_NOISE = mi; saveCal();
      zeroState = 2;
      snprintf(zeroMsg, sizeof(zeroMsg), "Zero offset %.3f A saved", mi);
      showToast("Zero offset saved");
    }
  }
  Serial.printf("[zero] %s\n", zeroMsg);
}
void serialCommands() {
  static char line[24];
  static int n = 0;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch != '\n' && ch != '\r') { if (n < (int)sizeof(line) - 1) line[n++] = ch; continue; }
    line[n] = 0;
    if (!n) continue;
    n = 0;
    char cmd = toupper(line[0]);
    float x = atof(line + 1);
    Cal c = calNow();
    const char *e = nullptr;
    if (cmd == 'V') e = calFromVolts(c, x);
    else if (cmd == 'I') e = calFromAmps(c, x);
    else if (cmd == 'O') c.voff = x;
    else if (cmd == 'F') c.isign = -c.isign;
    else if (cmd == 'D') c = CAL_DEF;
    else if (cmd == 'Z') { zeroStart(); continue; }
    else if (cmd == '?') { printCal(); continue; }
    else { Serial.println("[cal] commands: V <volts>, I <amps>, O <volts>, Z, F, D, ?"); continue; }
    if (!e) e = checkCal(c);
    if (e) { Serial.printf("[cal] %s\n", e); continue; }
    applyCal(c); saveCal(); printCal();
  }
}

// ---------------- web server ----------------
static void sendJson(AsyncWebServerRequest *req, const char *b) {
  AsyncWebServerResponse *r = req->beginResponse(200, "application/json", b);
  r->addHeader("Cache-Control", "no-store");
  req->send(r);
}
static void sendErr(AsyncWebServerRequest *req, const char *msg) {
  char b[160];
  snprintf(b, sizeof(b), "{\"ok\":0,\"err\":\"%s\"}", msg);
  sendJson(req, b);
}
static float param(AsyncWebServerRequest *req, const char *k, float def) {
  return req->hasParam(k) ? req->getParam(k)->value().toFloat() : def;
}
// seconds left on the current countdown, -1 if none
int countdownLeft(uint32_t now) {
  if (st == TRIPPING) return (int)ceil(max(0.0f, TRIP_DELAY_S - (now - stAt) / 1000.0f));
  if (st == WAITING && voltageInBand()) return (int)ceil(max(0.0f, RESTART_S - (now - stAt) / 1000.0f));
  return -1;
}

void dnsTask(void *) {
  for (;;) { dns.processNextRequest(); vTaskDelay(pdMS_TO_TICKS(5)); }
}

void setupWeb() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(WIFI_SSID, WIFI_PASS, 6, 0, 4);
  WiFi.setSleep(false);
  // phones probe for internet; answer every name with our address and the probes with "online"
  dns.setErrorReplyCode(DNSReplyCode::NoError);
  dns.start(53, "*", WiFi.softAPIP());
  xTaskCreatePinnedToCore(dnsTask, "dns", 3072, nullptr, 1, nullptr, 0);
  Serial.printf("[WiFi] AP '%s' (password %s) up -> open http://%s\n", WIFI_SSID, WIFI_PASS, WiFi.softAPIP().toString().c_str());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *req) {
    if (req->hasHeader("If-None-Match") && req->header("If-None-Match") == INDEX_HTML_ETAG) {
      AsyncWebServerResponse *r = req->beginResponse(304);
      r->addHeader("ETag", INDEX_HTML_ETAG);
      req->send(r);
      return;
    }
    AsyncWebServerResponse *r = req->beginResponse(200, "text/html", INDEX_HTML_GZ, INDEX_HTML_GZ_LEN);
    r->addHeader("Content-Encoding", "gzip");
    r->addHeader("Cache-Control", "no-cache");
    r->addHeader("ETag", INDEX_HTML_ETAG);
    req->send(r);
  });

  server.on("/data", HTTP_GET, [](AsyncWebServerRequest *req) {
    uint32_t now = millis();
    char b[512];
    snprintf(b, sizeof(b),
             "{\"v\":%.1f,\"i\":%.3f,\"p\":%.1f,\"pf\":%.3f,\"st\":\"%s\",\"relay\":%d,\"left\":%d,\"why\":\"%s\","
             "\"ovv\":%.0f,\"uvv\":%.0f,\"ocv\":%.2f,\"opp\":%.0f,\"td\":%.0f,\"rd\":%.0f,"
             "\"trips\":%d,\"up\":%lu,\"fw\":\"%s\",\"ovr\":%d}",
             V, I, fabs(P), PF, STATE_NAME[st], relayOn ? 1 : 0, countdownLeft(now),
             (st == TRIPPED || st == WAITING || st == TRIPPING) ? REASON_LONG[st == TRIPPING ? pending : waitReason] : "",
             LIM.ovv, LIM.uvv, LIM.ocv, LIM.opp, TRIP_DELAY_S, RESTART_S,
             tripCount, (unsigned long)(now / 1000), FW_VERSION, vOverride ? 1 : 0);
    sendJson(req, b);
  });

  server.on("/history", HTTP_GET, [](AsyncWebServerRequest *req) {
    AsyncResponseStream *r = req->beginResponseStream("application/json");
    r->addHeader("Cache-Control", "no-store");
    const float *rings[] = { hv, hi, hp };
    const char *names[] = { "v", "i", "p" };
    r->print("{\"dt\":1");
    for (int s = 0; s < 3; s++) {
      r->printf(",\"%s\":[", names[s]);
      for (int k = 0; k < hCount; k++) {
        int idx = (hHead - hCount + k + HN) % HN;
        if (k) r->print(',');
        r->printf(s == 1 ? "%.3f" : "%.1f", rings[s][idx]);
      }
      r->print("]");
    }
    r->print("}");
    req->send(r);
  });

  server.on("/trips", HTTP_GET, [](AsyncWebServerRequest *req) {
    uint32_t now = millis();
    AsyncResponseStream *r = req->beginResponseStream("application/json");
    r->addHeader("Cache-Control", "no-store");
    r->printf("{\"total\":%d,\"list\":[", tripCount);
    int n = min(tripCount, TRIPS_KEPT);
    for (int k = 0; k < n; k++) {
      const Trip &t = trips[k];
      bool amps = t.r == R_OC, watts = t.r == R_OP;
      if (k) r->print(',');
      r->printf("{\"why\":\"%s\",\"val\":%.2f,\"unit\":\"%s\",\"ago\":%lu}", REASON_LONG[t.r], t.val,
                amps ? "A" : (watts ? "W" : "V"), (unsigned long)((now - t.at) / 1000));
    }
    r->print("]}");
    req->send(r);
  });

  server.on("/limits", HTTP_POST, [](AsyncWebServerRequest *req) {
    Limits l = { param(req, "ovv", LIM.ovv), param(req, "uvv", LIM.uvv), param(req, "ocv", LIM.ocv), param(req, "opp", LIM.opp) };
    const char *e = checkLimits(l);
    if (e) { sendErr(req, e); return; }
    limNext = l;
    limPending = true;      // applied and saved in loop()
    sendJson(req, "{\"ok\":1}");
  });

  server.on("/relay", HTTP_POST, [](AsyncWebServerRequest *req) {
    int on = (int)param(req, "on", -1);
    if (on != 0 && on != 1) { sendErr(req, "on must be 0 or 1"); return; }
    relayCmd = on;          // applied in loop()
    sendJson(req, "{\"ok\":1}");
  });

  server.on("/cal", HTTP_GET, [](AsyncWebServerRequest *req) {
    char b[400];
    snprintf(b, sizeof(b),
             "{\"vgain\":%.4f,\"voff\":%.1f,\"itrim\":%.4f,\"inoise\":%.3f,\"isign\":%d,\"v\":%.1f,\"i\":%.3f,\"iraw\":%.3f,"
             "\"zero\":%d,\"zmsg\":\"%s\",\"def\":{\"vgain\":%.4f,\"voff\":%.1f,\"itrim\":%.4f,\"inoise\":%.3f,\"isign\":%d}}",
             V_GAIN, V_OFF, I_TRIM, I_NOISE, I_SIGN, V, I, rawI, zeroState, zeroMsg,
             CAL_DEF.vgain, CAL_DEF.voff, CAL_DEF.itrim, CAL_DEF.inoise, CAL_DEF.isign);
    sendJson(req, b);
  });

  // one change per call: vref= (multimeter V), iref= (clamp A), vgain=, voff=, itrim=, inoise=, isign=, defaults=1
  server.on("/cal", HTTP_POST, [](AsyncWebServerRequest *req) {
    Cal c = calNow();
    const char *e = nullptr;
    if (req->hasParam("defaults")) c = CAL_DEF;
    else if (req->hasParam("vref")) e = calFromVolts(c, param(req, "vref", 0));
    else if (req->hasParam("iref")) e = calFromAmps(c, param(req, "iref", 0));
    else {
      float oldTrim = c.itrim;
      c.vgain = param(req, "vgain", c.vgain);
      c.voff = param(req, "voff", c.voff);
      c.itrim = param(req, "itrim", c.itrim);
      c.inoise = param(req, "inoise", c.inoise);
      c.isign = (int)param(req, "isign", (float)c.isign);
      // the zero offset is in amps: a new trim rescales it, unless it was typed in too
      if (!req->hasParam("inoise") && c.itrim > 0 && oldTrim > 0) c.inoise *= c.itrim / oldTrim;
    }
    if (!e) e = checkCal(c);
    if (e) { sendErr(req, e); return; }
    calNext = c;
    calPending = true;      // applied and saved in loop()
    char b[200];
    snprintf(b, sizeof(b), "{\"ok\":1,\"vgain\":%.4f,\"voff\":%.1f,\"itrim\":%.4f,\"inoise\":%.3f,\"isign\":%d}",
             c.vgain, c.voff, c.itrim, c.inoise, c.isign);
    sendJson(req, b);
  });

  server.on("/zero", HTTP_POST, [](AsyncWebServerRequest *req) {
    if (zeroState == 1) { sendErr(req, "Already measuring"); return; }
    zeroState = 1;
    strcpy(zeroMsg, "Measuring, keep the load unplugged");
    zeroReq = true;         // runs in loop() over about 4 s
    sendJson(req, "{\"ok\":1}");
  });

  // captive-portal probes: say "internet OK" so phones stay on this WiFi
  auto online204 = [](AsyncWebServerRequest *req) { req->send(204); };
  server.on("/generate_204", HTTP_GET, online204);
  server.on("/gen_204", HTTP_GET, online204);
  server.on("/hotspot-detect.html", HTTP_GET, [](AsyncWebServerRequest *req) {
    req->send(200, "text/html", "<HTML><HEAD><TITLE>Success</TITLE></HEAD><BODY>Success</BODY></HTML>");
  });
  server.on("/connecttest.txt", HTTP_GET, [](AsyncWebServerRequest *req) { req->send(200, "text/plain", "Microsoft Connect Test"); });
  server.on("/ncsi.txt", HTTP_GET, [](AsyncWebServerRequest *req) { req->send(200, "text/plain", "Microsoft NCSI"); });
  server.onNotFound([](AsyncWebServerRequest *req) { req->redirect("http://192.168.4.1/"); });
  server.begin();
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
const char *tripValue(const Trip &t, char *b, size_t n) {
  if (t.r == R_OC) snprintf(b, n, "%s %.1fA", REASON_SHORT[t.r], t.val);
  else if (t.r == R_OP) snprintf(b, n, "%s %.0fW", REASON_SHORT[t.r], t.val);
  else snprintf(b, n, "%s %.0fV", REASON_SHORT[t.r], t.val);
  return b;
}
void pageTitle(const char *t) {
  text(t, 4, 4, BRAND);
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
  int left = countdownLeft(now);
  if (st == ARMED && vOverride) {
    bg = WARN;
    strcpy(s, "OVERRIDE");
  } else if (st == TRIPPING) {
    bg = WARN;
    snprintf(s, sizeof(s), "TRIP IN %.1fs", max(0.0f, TRIP_DELAY_S - (now - stAt) / 1000.0f));
  } else if (st == TRIPPED) {
    bg = ((now / 400) % 2) ? BAD : BAD_DIM;
    strcpy(s, "TRIPPED");
  } else if (st == WAITING) {
    bg = WAIT;
    if (left >= 0) snprintf(s, sizeof(s), "%s %ds", waitReason ? "WAITING" : "STARTING", left);
    else strcpy(s, "BAD SUPPLY");
  } else if (st == MANUAL_OFF) {
    bg = OFFGREY;
    strcpy(s, "OFF (MANUAL)");
  }
  cv.fillRect(0, 0, W, 22, bg);
  textC(s, 16, (bg == BAD_DIM || bg == OFFGREY) ? FG : BG, &BaiM);
}
void meterV(int y) {
  const float lo = LIM.uvv - 20, hi = LIM.ovv + 20;
  auto X = [&](float v) { return 4 + (int)((constrain(v, lo, hi) - lo) / (hi - lo) * (W - 8)); };
  cv.fillRect(4, y, W - 8, 6, CARD);
  cv.fillRect(X(LIM.uvv), y, X(LIM.ovv) - X(LIM.uvv), 6, SAFE_BAND);
  cv.drawFastVLine(X(LIM.uvv), y - 2, 10, BAD);
  cv.drawFastVLine(X(LIM.ovv), y - 2, 10, BAD);
  cv.fillRect(X(V) - 1, y - 3, 3, 12, (V > LIM.ovv || V < LIM.uvv) ? BAD : FG);
  char b[8];
  snprintf(b, sizeof(b), "%.0f", LIM.uvv); text(b, X(LIM.uvv) - textW(b) / 2, y + 10, MUTED);
  snprintf(b, sizeof(b), "%.0f", LIM.ovv); text(b, X(LIM.ovv) - textW(b) / 2, y + 10, MUTED);
}
void meterI(int y) {
  const float hi = LIM.ocv * 1.25f;
  auto X = [&](float v) { return 4 + (int)(constrain(v, 0.0f, hi) / hi * (W - 8)); };
  cv.fillRect(4, y, W - 8, 6, CARD);
  uint16_t c = I > LIM.ocv ? BAD : (I > LIM.ocv * 0.9f ? WARN : AMP);
  if (I > 0) cv.fillRect(4, y, X(I) - 4, 6, c);
  cv.drawFastVLine(X(LIM.ocv), y - 2, 10, BAD);
  char b[8];
  text("0", 4, y + 9, MUTED);
  snprintf(b, sizeof(b), "%.1f", LIM.ocv); text(b, X(LIM.ocv) - textW(b) / 2, y + 9, MUTED);
}
void drawHome(uint32_t now) {
  char b[32];
  statusBar(now);
  text("VOLTAGE", 4, 25, MUTED);
  snprintf(b, sizeof(b), "%.1f", V);
  text(b, 4, 60, (V > LIM.ovv || V < LIM.uvv) ? BAD : VOLT, &BaiL);
  textR("V", W - 4, 60, FG2, &BaiM);
  meterV(65);
  text("CURRENT", 4, 88, MUTED);
  snprintf(b, sizeof(b), "%.2f", I);
  text(b, 4, 122, I > LIM.ocv ? BAD : AMP, &BaiL);
  textR("A", W - 4, 122, FG2, &BaiM);
  meterI(127);
  cv.drawFastHLine(0, 148, W, LINE);
  if ((st == TRIPPED || st == WAITING) && waitReason != R_NONE && tripCount) {
    text(tripValue(trips[0], b, sizeof(b)), 4, 151, BAD);
  } else {
    snprintf(b, sizeof(b), "%.0f W", fabs(P));
    text(b, 4, 151, fabs(P) > LIM.opp ? BAD : POW);
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
  float vl[] = { LIM.ovv, LIM.uvv };
  chart(30, 54, hv, LIM.uvv - 25, LIM.ovv + 25, VOLT, vl, 2);
  snprintf(b, sizeof(b), "CURRENT  %.2f A", I);
  text(b, 4, 92, AMP);
  float il[] = { LIM.ocv };
  chart(102, 54, hi, 0, LIM.ocv * 1.3f, AMP, il, 1);
}

// ---------------- screen 3: Trips ----------------
void drawTrips(uint32_t now) {
  char b[32], a[16];
  pageTitle("TRIPS");
  if (!tripCount) { textC("No trips yet", 70, OK_GREEN, &BaiM); return; }
  const Trip &t = trips[0];
  text("LAST TRIP", 4, 21, MUTED);
  text(REASON_LONG[t.r], 4, 46, BAD, &BaiM);
  if (t.r == R_OC) snprintf(b, sizeof(b), "%.1f A for %.0f s", t.val, TRIP_DELAY_S);
  else if (t.r == R_OP) snprintf(b, sizeof(b), "%.0f W for %.0f s", t.val, TRIP_DELAY_S);
  else snprintf(b, sizeof(b), "%.0f V for %.0f s", t.val, TRIP_DELAY_S);
  text(b, 4, 53, FG);
  text(ago(now - t.at, a, sizeof(a)), 4, 65, FG2);
  cv.drawFastHLine(0, 78, W, LINE);
  snprintf(b, sizeof(b), "Trips total: %d", tripCount);
  text(b, 4, 84, FG);
  int y = 100;
  for (int k = 1; k < 4 && k < tripCount && k < TRIPS_KEPT; k++, y += 14) {
    text(tripValue(trips[k], b, sizeof(b)), 4, y, FG2);
    uint32_t sec = (now - trips[k].at) / 1000;
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
  snprintf(b, sizeof(b), "%.0f V", LIM.ovv);      row(22, "Over-voltage", b, VOLT);
  snprintf(b, sizeof(b), "%.0f V", LIM.uvv);      row(40, "Under-voltage", b, VOLT);
  snprintf(b, sizeof(b), "%.1f A", LIM.ocv);      row(58, "Over-current", b, AMP);
  snprintf(b, sizeof(b), "%.0f W", LIM.opp);      row(76, "Over-power", b, POW);
  snprintf(b, sizeof(b), "%.0f s", TRIP_DELAY_S); row(94, "Trip delay", b, FG);
  snprintf(b, sizeof(b), "%.0f s", RESTART_S);    row(112, "Restart delay", b, FG);
  row(130, "After a trip", "AUTO", FG);
  textC("Edit on the web page", 148, MUTED);
}

// ---------------- screen 5: Connect ----------------
void drawConnect() {
  char b[32];
  pageTitle("CONNECT");
  text("WiFi", 4, 22, MUTED);
  text(WIFI_SSID, 4, 44, FG, &BaiM);
  text("Password", 4, 52, MUTED);
  text(WIFI_PASS, 4, 74, FG, &BaiM);
  text("Then open", 4, 82, MUTED);
  text("192.168.4.1", 4, 104, BRAND, &BaiM);
  cv.drawFastHLine(0, 116, W, LINE);
  snprintf(b, sizeof(b), "Phones connected: %d", WiFi.softAPgetStationNum());
  text(b, 4, 122, FG2);
  snprintf(b, sizeof(b), "Firmware %s", FW_VERSION);
  text(b, 4, 136, MUTED);
  text("Hold 2 s: relay on / off", 4, 149, MUTED);
}

// ---------------- frame ----------------
void render(uint32_t now) {
  cv.fillScreen(BG);
  switch (page) {
    case HOME:   drawHome(now);  break;
    case GRAPH:  drawGraph();    break;
    case TRIPS:  drawTrips(now); break;
    case LIMITS: drawLimits();   break;
    default:     drawConnect();  break;
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
  V_OFF = prefs.getFloat("voff", V_OFF_DEF);
  I_TRIM = prefs.getFloat("itrim", I_TRIM_DEF);
  I_NOISE = prefs.getFloat("inoise", I_NOISE_DEF);
  I_SIGN = prefs.getInt("isign", I_SIGN_DEF) < 0 ? -1 : 1;
  Limits saved;
  if (prefs.getBytesLength("limits") == sizeof(saved) && prefs.getBytes("limits", &saved, sizeof(saved)) == sizeof(saved)
      && !checkLimits(saved)) LIM = saved;

  SPI.begin(TFT_SCK, -1, TFT_SDA, TFT_CS);   // hardware SPI on our pins (the library then keeps them)
  tft.initR(INITR_BLACKTAB);
  tft.setSPISpeed(27000000);
  tft.setRotation(0);
  tft.fillScreen(BG);
  cv.setTextWrap(false);
  cv.fillScreen(BG);
  textC("GridGuard", 80, BRAND, &BaiM);
  textC("starting WiFi...", 96, FG2);
  tft.drawRGBBitmap(0, 0, cv.getBuffer(), W, H);

  Serial.printf("\n=== GridGuard v%s ===\n", FW_VERSION);
  setupWeb();
  st = WAITING; stAt = millis(); waitReason = R_NONE;
  beep(1, 80, 0);
  Serial.printf("Limits: OVV %.0f V, UVV %.0f V, OCV %.2f A, OPP %.0f W, trip delay %.0f s, restart %.0f s\n",
                LIM.ovv, LIM.uvv, LIM.ocv, LIM.opp, TRIP_DELAY_S, RESTART_S);
  printCal();
  Serial.println("Calibration commands: V <volts>, I <amps>, O <volts>, Z, F, D, ?");
}

void loop() {
  serialCommands();
  bool good = measure();           // 200 ms; the last good values stay if a window is skipped
  zeroTick(good);
  uint32_t now = millis();
  buttonTick(now);
  buzzerTick(now);
  handleWeb();
  updateState(now);
  handleButton();
  ledsUpdate(now);

  if (now - lastHist >= 1000) {
    lastHist = now;
    hv[hHead] = V; hi[hHead] = I; hp[hHead] = fabs(P);
    hHead = (hHead + 1) % HN;
    if (hCount < HN) hCount++;
  }
  render(now);

  if (now - lastLog >= 1000) {
    lastLog = now;
    Serial.printf("V %.1f | I %.3f (raw %.3f) | P %.0f W | PF %.2f | %s | relay %s | clients %d\n",
                  V, I, rawI, P, PF, STATE_NAME[st], relayOn ? "ON" : "OFF", WiFi.softAPgetStationNum());
  }
}
