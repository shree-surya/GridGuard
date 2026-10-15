/*
  GridGuard display test: 1.77" ST7735S colour TFT (128 x 160), simulated readings, no sensors.
  ==============================================================================================
  Shows the five GridGuard screens with made-up voltage and current that run through a 100 s
  story: normal, over-voltage trip, reconnect, over-current trip, reconnect, under-voltage trip,
  reconnect. The trip logic is the real GridGuard state machine (trip delay, restart delay,
  voltage hysteresis), so the status bar, countdowns and trip log behave as they will for real.
  The screen changes every 5 s; when a trip happens it jumps to Home.

  WIRING                               ARDUINO IDE
    TFT VCC  -> 3V3                      Board: "ESP32 Dev Module"
    TFT GND  -> GND                      Libraries (Library Manager):
    TFT CS   -> GPIO5                      "Adafruit GFX Library"
    TFT RST  -> GPIO4                      "Adafruit ST7735 and ST7789 Library"
    TFT DC   -> GPIO27                   Serial Monitor: 115200
    TFT SDA  -> GPIO23  (SPI data, not I2C)
    TFT SCK  -> GPIO18  (SPI clock)
    TFT BLK  -> 3V3     (backlight, if your module has the pin)

  FILES: keep BaiJamjuree.h in the same folder as this .ino (the IDE shows it as a second tab).

  IF THE PICTURE LOOKS WRONG
    Shifted by a few pixels or a noisy line at one edge:  change INITR_BLACKTAB to INITR_GREENTAB
    Colours inverted (black background shows white):      set INVERT to true
    Red and blue swapped (the ARMED bar is not green):     change INITR_BLACKTAB to INITR_REDTAB
*/

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include "BaiJamjuree.h"     // Bai Jamjuree in three sizes (BaiS, BaiM, BaiL); keep this file beside the .ino

// ---------------- types (kept above every function: the Arduino IDE adds its function
// declarations just before the first function, and they must already see these) ----------------
enum State { ARMED, TRIPPING, TRIPPED, WAITING };
enum Reason { R_NONE, R_OV, R_UV, R_OC };
enum Page { HOME, GRAPH, TRIPS, LIMITS, CONNECT, PAGES };
struct Trip { Reason r; float val; uint32_t at; };
// declared here with their default arguments, so the IDE does not generate its own declarations for them
void text(const char *s, int x, int y, uint16_t col, const GFXfont *f = nullptr);
int  textW(const char *s, const GFXfont *f = nullptr);

// ---------------- pins and panel ----------------
const int  TFT_CS = 5, TFT_RST = 4, TFT_DC = 27;
const bool INVERT = false;
const int  W = 128, H = 160;

Adafruit_ST7735 tft(TFT_CS, TFT_DC, TFT_RST);   // hardware SPI: SCK 18, MOSI 23
GFXcanvas16     cv(W, H);                       // draw here, then send the whole frame (no flicker)

// ---------------- colours (same palette as the web pages) ----------------
#define rgb(c) ((uint16_t)((((c) >> 8) & 0xF800) | (((c) >> 5) & 0x07E0) | (((c) >> 3) & 0x001F)))
const uint16_t BG = rgb(0x0F1115), CARD = rgb(0x1D212A), LINE = rgb(0x2A303C);
const uint16_t FG = rgb(0xFFFFFF), FG2 = rgb(0xA7AFBD), MUTED = rgb(0x8A93A3);
const uint16_t VOLT = rgb(0x35D0FF), AMP = rgb(0xFF8A3D), POW = rgb(0xB6F24A);
const uint16_t OK_GREEN = rgb(0x3DDC84), WARN = rgb(0xFFB020), BAD = rgb(0xFF4D4D), BAD_DIM = rgb(0x8A1F1F), WAIT = rgb(0x4DA3FF);
const uint16_t SAFE_BAND = rgb(0x1E3A2A);

// ---------------- limits (GridGuard defaults) ----------------
float OVV = 250, UVV = 190, OCV = 6.0, TRIP_DELAY_S = 2, RESTART_S = 10, HYST = 0.02f;
const uint32_t TRIPPED_SHOW_MS = 3000;   // how long "TRIPPED" stays before the reconnect countdown

// ---------------- state ----------------
const char *REASON_LONG[]  = { "", "Over-voltage", "Under-voltage", "Over-current" };
const char *REASON_SHORT[] = { "", "OVER-V", "UNDER-V", "OVER-I" };

State    st = ARMED;
uint32_t stAt = 0;            // when the current state (or countdown) started
Reason   pending = R_NONE;    // fault being timed while TRIPPING
bool     relayOn = true;
float    V = 0, I = 0, P = 0;

Trip     trips[3];            // newest first
int      tripCount = 0;

const int HN = 60;            // 60 s of history for the graph screen
float    hv[HN], hi[HN];
int      hHead = 0, hCount = 0;

int      page = HOME;
uint32_t pageAt = 0;
const uint32_t PAGE_MS = 5000;

// ---------------- simulated sensors: a 100 s story ----------------
float noise(float a) { return a * ((random(2001) - 1000) / 1000.0f); }

void simulate(uint32_t now) {
  float t = fmod(now / 1000.0, 100.0);
  float v = 231 + 1.5f * sin(t * 0.7f) + noise(0.4f);
  if (t >= 20 && t < 30) v = 231 + min(1.0f, (t - 20) / 2.0f) * (262 - 231) + noise(0.6f);   // over-voltage
  if (t >= 75 && t < 80) v = 182 + noise(0.6f);                                              // under-voltage
  float load = 2.4f + 0.3f * sin(t * 0.3f) + noise(0.03f);
  if (t >= 55 && t < 62) load = 7.2f + noise(0.05f);                                         // over-current
  V = v;
  I = relayOn ? load : 0;     // with the relay open no current flows
  P = V * I * 0.95f;
}

// ---------------- the GridGuard state machine ----------------
void recordTrip(Reason r, float val, uint32_t now) {
  for (int k = 2; k > 0; k--) trips[k] = trips[k - 1];
  trips[0] = { r, val, now };
  tripCount++;
}

void updateState(uint32_t now) {
  Reason f = R_NONE;
  float fv = 0;
  if (V > OVV) { f = R_OV; fv = V; }
  else if (V < UVV) { f = R_UV; fv = V; }
  else if (relayOn && I > OCV) { f = R_OC; fv = I; }

  switch (st) {
    case ARMED:
      if (f) { st = TRIPPING; stAt = now; pending = f; }
      break;
    case TRIPPING:
      if (!f) st = ARMED;                                   // fault went away in time
      else if (now - stAt >= TRIP_DELAY_S * 1000) {
        st = TRIPPED; stAt = now; relayOn = false;
        recordTrip(f, fv, now);
        page = HOME; pageAt = now;                          // a trip always brings Home up
        Serial.printf("[trip] %s %.1f\n", REASON_LONG[f], fv);
      }
      break;
    case TRIPPED:
      if (now - stAt >= TRIPPED_SHOW_MS) { st = WAITING; stAt = now; }
      break;
    case WAITING: {
      bool vOk = V <= OVV * (1 - HYST) && V >= UVV * (1 + HYST);
      if (!vOk) stAt = now;                                 // countdown starts only once the supply is back in band
      if (now - stAt >= RESTART_S * 1000) { st = ARMED; relayOn = true; Serial.println("[relay] reconnected"); }
      break;
    }
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
void textR(const char *s, int xr, int y, uint16_t col, const GFXfont *f = nullptr) { text(s, xr - textW(s, f), y, col, f); }
void textC(const char *s, int y, uint16_t col, const GFXfont *f = nullptr) { text(s, (W - textW(s, f)) / 2, y, col, f); }
// big font if it fits the width, otherwise the small one (so long names never wrap)
void textFit(const char *s, int x, int y, uint16_t col) {
  if (textW(s, &BaiM) <= W - x - 2) text(s, x, y, col, &BaiM);
  else text(s, x, y - BAIS_CAP, col);
}
void dashedH(int x0, int x1, int y, uint16_t col) { for (int x = x0; x < x1; x += 4) cv.drawFastHLine(x, y, 2, col); }
const char *ago(uint32_t ms, char *b, size_t n) {
  uint32_t s = ms / 1000;
  if (s < 60) snprintf(b, n, "%lu s ago", (unsigned long)s);
  else if (s < 3600) snprintf(b, n, "%lu min ago", (unsigned long)(s / 60));
  else snprintf(b, n, "%lu h ago", (unsigned long)(s / 3600));
  return b;
}
void pageTitle(const char *t, uint16_t col) {
  text(t, 4, 4, col);
  for (int p = 1; p < PAGES; p++) {                       // dots for screens 2 to 5
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
    float left = max(0.0f, TRIP_DELAY_S - (now - stAt) / 1000.0f);
    snprintf(s, sizeof(s), "TRIP IN %.1fs", left);
  } else if (st == TRIPPED) {
    bg = ((now / 400) % 2) ? BAD : BAD_DIM;                 // flashes
    strcpy(s, "TRIPPED");
  } else if (st == WAITING) {
    bg = WAIT;
    float left = max(0.0f, RESTART_S - (now - stAt) / 1000.0f);
    snprintf(s, sizeof(s), "WAITING %ds", (int)ceil(left));
  }
  cv.fillRect(0, 0, W, 22, bg);
  textC(s, 16, bg == BAD_DIM ? FG : BG, &BaiM);
}

// a horizontal meter: track, optional safe band, fill or marker, end labels
void meterV(int y) {
  const float lo = UVV - 20, hi = OVV + 20;
  auto X = [&](float v) { return 4 + (int)((constrain(v, lo, hi) - lo) / (hi - lo) * (W - 8)); };
  cv.fillRect(4, y, W - 8, 6, CARD);
  cv.fillRect(X(UVV), y, X(OVV) - X(UVV), 6, SAFE_BAND);
  cv.drawFastVLine(X(UVV), y - 2, 10, BAD);
  cv.drawFastVLine(X(OVV), y - 2, 10, BAD);
  bool out = V > OVV || V < UVV;
  cv.fillRect(X(V) - 1, y - 3, 3, 12, out ? BAD : FG);
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

  text("VOLTAGE", 4, 27, MUTED);
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
  if (st == TRIPPED || st == WAITING) {
    const Trip &t = trips[0];
    snprintf(b, sizeof(b), t.r == R_OC ? "%s %.1fA" : "%s %.0fV", REASON_SHORT[t.r], t.val);
    text(b, 4, 151, BAD);
  } else {
    snprintf(b, sizeof(b), "%.0f W", P);
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
  pageTitle("LAST 60 s", FG);
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
  pageTitle("TRIPS", FG);
  if (!tripCount) {
    textC("No trips yet", 70, OK_GREEN, &BaiM);
    return;
  }
  const Trip &t = trips[0];
  text("LAST TRIP", 4, 21, MUTED);
  text(REASON_LONG[t.r], 4, 46, BAD, &BaiM);
  snprintf(b, sizeof(b), t.r == R_OC ? "%.1f A for %.0f s" : "%.0f V for %.0f s", t.val, TRIP_DELAY_S);
  text(b, 4, 53, FG);
  text(ago(now - t.at, a, sizeof(a)), 4, 65, FG2);

  cv.drawFastHLine(0, 78, W, LINE);
  snprintf(b, sizeof(b), "Trips total: %d", tripCount);
  text(b, 4, 84, FG);
  int y = 100;
  for (int k = 1; k < 3 && k < tripCount; k++, y += 14) {
    const Trip &o = trips[k];
    snprintf(b, sizeof(b), o.r == R_OC ? "%s %.1fA" : "%s %.0fV", REASON_SHORT[o.r], o.val);
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
  pageTitle("LIMITS", FG);
  snprintf(b, sizeof(b), "%.0f V", OVV);       row(22, "Over-voltage", b, VOLT);
  snprintf(b, sizeof(b), "%.0f V", UVV);       row(40, "Under-voltage", b, VOLT);
  snprintf(b, sizeof(b), "%.1f A", OCV);       row(58, "Over-current", b, AMP);
  row(76, "Under-current", "off", MUTED);
  snprintf(b, sizeof(b), "%.0f s", TRIP_DELAY_S); row(94, "Trip delay", b, FG);
  snprintf(b, sizeof(b), "%.0f s", RESTART_S); row(112, "Restart delay", b, FG);
  row(130, "After a trip", "AUTO", FG);
  textC("Set from the phone", 148, MUTED);
}

// ---------------- screen 5: Connect ----------------
void drawConnect() {
  pageTitle("CONNECT", FG);
  text("WiFi", 4, 24, MUTED);
  textFit("GridGuard-A1B2", 4, 44, FG);
  text("Password", 4, 58, MUTED);
  textFit("gridguard", 4, 78, FG);
  text("Then open", 4, 92, MUTED);
  text("192.168.4.1", 4, 112, VOLT, &BaiM);
  cv.drawFastHLine(0, 126, W, LINE);
  text("Phones connected: 0", 4, 132, FG2);
  text("Settings, trip log", 4, 146, MUTED);
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
  tft.drawRGBBitmap(0, 0, cv.getBuffer(), W, H);
}

// ---------------- setup / loop ----------------
uint32_t lastTick = 0, lastHist = 0;

void setup() {
  Serial.begin(115200);
  tft.initR(INITR_BLACKTAB);
  tft.setSPISpeed(27000000);
  tft.setRotation(0);          // portrait, 128 wide x 160 tall; 2 turns it upside down
  tft.invertDisplay(INVERT);
  tft.fillScreen(BG);
  cv.setTextWrap(false);       // text that does not fit is cut, never wrapped onto the next line
  Serial.println("\n=== GridGuard display test (simulated readings) ===");
}

void loop() {
  uint32_t now = millis();
  if (now - lastTick < 250) return;         // 4 frames per second
  lastTick = now;

  simulate(now);
  updateState(now);
  if (now - lastHist >= 1000) {              // one graph point per second
    lastHist = now;
    hv[hHead] = V; hi[hHead] = I;
    hHead = (hHead + 1) % HN;
    if (hCount < HN) hCount++;
  }
  if (now - pageAt >= PAGE_MS) { pageAt = now; page = (page + 1) % PAGES; }
  render(now);
}
