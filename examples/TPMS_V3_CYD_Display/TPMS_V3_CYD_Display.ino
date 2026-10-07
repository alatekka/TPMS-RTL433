// =============================================================================
//  TPMS V3 — وحدة الشاشة (CYD: ESP32-2432S028، ST7789 2.8" + لمس XPT2046)
//  "العقل": معرّفات الإطارات، الحدود، الإنذارات، الواجهة الرسومية، الواي فاي.
//  تستقبل الحزم من وحدة الراديو (TPMS_V3_Receiver) عبر UART بروتوكول TPMS-LINK v2.
//  الواي فاي هنا وليس على وحدة الراديو: قياسياً رفع الواي فاي القريب من CC1101
//  أرضية الضجيج 4 dB، أما هنا فهو على لوحة منفصلة فلا يمس الاستقبال إطلاقاً.
// =============================================================================
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include "esp_task_wdt.h"
#include "cyd_lgfx.h"

#define FW_VERSION "V3-UI 3.0"

// ---------------- العتاد ----------------
#define LINK_RX   22          // CN1 في لوحة CYD
#define LINK_TX   27
#define LINK_BAUD 115200
#define PIN_SPK   26          // مخرج السماعة
#define PIN_LED_R 4           // لمبة RGB (فعّالة عند LOW)
#define PIN_LED_G 16
#define PIN_LED_B 17
#define PIN_LDR   34          // حساس الضوء

HardwareSerial Link(2);
LGFX_CYD* lcd = nullptr;
LGFX_Sprite band;             // نصف شاشة 320x120 بلون 16 بت: رسم بلا وميض
static const int SCR_W = 320, SCR_H = 240, BAND_H = 120;
static const int TOP_H = 22, NAV_Y = 206, NAV_H = 34;

// ---------------- الألوان ----------------
#define C_BG      0x0861   // #0b0d12 تقريباً
#define C_PANEL   0x10C4
#define C_PANEL2  0x18E6
#define C_LINE    0x2947
#define C_TEXT    0xEF7D
#define C_MUTED   0x8C71
#define C_ACCENT  0x2E9F   // سماوي
#define C_OK      0x2E69   // أخضر
#define C_WARN    0xFD20   // برتقالي
#define C_DANGER  0xF206   // أحمر
#define C_BLUE    0x3B7F
#define C_CAR     0x3A2D
#define C_GLASS   0x1A8E

// ---------------- الإعدادات والبيانات ----------------
enum : uint8_t { AL_LOW = 1, AL_HIGH = 2, AL_HOT = 4, AL_BATT = 8, AL_LEAK = 16 };
#define AL_DANGER (AL_LOW | AL_HOT | AL_LEAK)
static const char* TIRE_LBL[5]  = {"FL", "FR", "RL", "RR", "SP"};
static const char* TIRE_NAME[5] = {"FRONT LEFT", "FRONT RIGHT", "REAR LEFT", "REAR RIGHT", "SPARE"};
static const char* ORIG_IDS[5]  = {"D78D8483", "D78D8C0A", "D78D8CD9", "D78D8C17", "D78D8409"};

struct TireCfg { char id[12]; float pMin, pMax, tMax; };
struct TireState {
  bool has; float bar, temp; int8_t batt; int rssi; uint32_t lastMs, count; uint8_t alarms; bool hwLeak;
  float hb[32]; uint32_t ht[32]; uint8_t hh, hn;           // تاريخ قصير لكشف التسريب
  float trend[60]; uint32_t trendT; uint8_t th, tn;         // منحنى (عينة كل دقيقة)
};
struct Scan { char id[12]; int8_t p, tire, batt; float bar, temp; int rssi, best; uint32_t first, last, count; };
struct Radio {
  bool cc; int err; float f; int foff, bw, d, act, n, th, cur; uint32_t cap, dec, unk, lost, pk, rej, rst, up; int heap; char fw[20];
};
struct TechCap { bool valid; char id[12]; int8_t p, tire, batt; float bar, temp; int rssi, best; uint32_t lastMs, count; };

TireCfg   tcfg[5];
TireState ts[5];
#define MAX_SCAN 40
Scan      scan[MAX_SCAN];
int       scanN = 0;
Radio     radio = {};
TechCap   tech = {}, techHist[10];
int       techHistN = 0;
char      decName[25][28], decShort[25][12];
bool      decKnown = false;

// تفضيلات الواجهة (NVS)
int  brightness = 200;      bool autoBright = true;
int  unitP = 0;             // 0 bar، 1 psi، 2 kPa
bool soundOn = true, wifiOn = true;
bool panelInvert = true, panelBgr = false;
int  techThr = -70;
uint16_t touchCal[8];       bool touchCalValid = false;
#define STALE_MS (15UL * 60UL * 1000UL)

SemaphoreHandle_t dataMux;
Preferences prefs;
volatile uint32_t linkLastMs = 0, linkBad = 0, linkGood = 0;
int liveRssi = -120, livePeak = -120;
int rssiHist[120]; int rssiHistN = 0, rssiHistHead = 0;

// ---------------- الواجهة ----------------
enum Page : uint8_t { PG_CAR, PG_SCAN, PG_RADIO, PG_TECH, PG_SETUP, PG_COUNT };
volatile uint8_t page = PG_CAR;
int setupTab = 0, limitsTire = 0, scanScroll = 0, popupScan = -1;
volatile bool uiDirty = true;
volatile bool alarmActive = false, alarmAcked = false;
bool calibrateRequest = false;
volatile bool shotRequest = false;
volatile int tapX = -1, tapY = -1;

struct Btn { int16_t x, y, w, h; uint16_t id; };
Btn btns[64]; int btnN = 0;
bool recordBtns = false;
void addBtn(int x, int y, int w, int h, uint16_t id) {
  if (!recordBtns || btnN >= 64) return;
  for (int i = 0; i < btnN; i++) if (btns[i].id == id && btns[i].x == x && btns[i].y == y) return;  // عنصر يعبر الشريطين
  btns[btnN++] = {(int16_t)x, (int16_t)y, (int16_t)w, (int16_t)h, id};
}

// ---------------- أدوات ----------------
static inline float pU(float bar) { return unitP == 1 ? bar * 14.5038f : unitP == 2 ? bar * 100.0f : bar; }
static inline const char* pUnit() { return unitP == 1 ? "PSI" : unitP == 2 ? "kPa" : "BAR"; }
static inline int pDec() { return unitP == 0 ? 2 : unitP == 1 ? 1 : 0; }
void fmtAge(uint32_t ms, char* o, size_t n) {
  uint32_t s = ms / 1000;
  if (s < 60) snprintf(o, n, "%lus", (unsigned long)s);
  else if (s < 3600) snprintf(o, n, "%lum", (unsigned long)(s / 60));
  else snprintf(o, n, "%luh", (unsigned long)(s / 3600));
}
bool idMatch(const char* full, const char* cfg) {
  size_t fl = strlen(full), cl = strlen(cfg);
  if (!cl || cl > fl) return false;
  return !strcmp(full + fl - cl, cfg);
}
int findTire(const char* id) { for (int i = 0; i < 5; i++) if (tcfg[i].id[0] && idMatch(id, tcfg[i].id)) return i; return -1; }
const char* protoShort(int p) { return (p >= 0 && p < 25 && decKnown) ? decShort[p] : (p >= 0 ? "TPMS" : "?"); }
bool stale(const TireState& t) { return t.has && millis() - t.lastMs > STALE_MS; }
uint16_t tireColor(const TireState& t) {
  if (!t.has || stale(t)) return C_MUTED;
  if (t.alarms & AL_DANGER) return C_DANGER;
  if (t.alarms) return C_WARN;
  return C_OK;
}
const char* alarmText(uint8_t a) {
  if (a & AL_LEAK) return "FAST LEAK";
  if (a & AL_LOW) return "LOW PRESSURE";
  if (a & AL_HOT) return "TOO HOT";
  if (a & AL_HIGH) return "HIGH PRESSURE";
  if (a & AL_BATT) return "BATTERY LOW";
  return "OK";
}

// ---------------- الإعدادات الدائمة ----------------
void loadPrefs() {
  prefs.begin("v3ui", true);
  for (int i = 0; i < 5; i++) {
    char k[12];
    snprintf(k, sizeof(k), "id%d", i);   String v = prefs.getString(k, ORIG_IDS[i]); strlcpy(tcfg[i].id, v.c_str(), sizeof(tcfg[i].id));
    snprintf(k, sizeof(k), "pmin%d", i); tcfg[i].pMin = prefs.getFloat(k, 1.80f);
    snprintf(k, sizeof(k), "pmax%d", i); tcfg[i].pMax = prefs.getFloat(k, i == 4 ? 4.50f : 3.00f);
    snprintf(k, sizeof(k), "tmax%d", i); tcfg[i].tMax = prefs.getFloat(k, 75.0f);
  }
  brightness  = prefs.getInt("bright", 200);
  autoBright  = prefs.getBool("autob", true);
  unitP       = prefs.getInt("unit", 0);
  soundOn     = prefs.getBool("sound", true);
  wifiOn      = prefs.getBool("wifi", true);
  panelInvert = prefs.getBool("inv", true);
  panelBgr    = prefs.getBool("bgr", false);
  techThr     = prefs.getInt("tthr", -70);
  touchCalValid = prefs.getBytes("tcal", touchCal, sizeof(touchCal)) == sizeof(touchCal);
  prefs.end();
}
void savePrefs() {
  prefs.begin("v3ui", false);
  for (int i = 0; i < 5; i++) {
    char k[12];
    snprintf(k, sizeof(k), "id%d", i);   prefs.putString(k, tcfg[i].id);
    snprintf(k, sizeof(k), "pmin%d", i); prefs.putFloat(k, tcfg[i].pMin);
    snprintf(k, sizeof(k), "pmax%d", i); prefs.putFloat(k, tcfg[i].pMax);
    snprintf(k, sizeof(k), "tmax%d", i); prefs.putFloat(k, tcfg[i].tMax);
  }
  prefs.putInt("bright", brightness);
  prefs.putBool("autob", autoBright);
  prefs.putInt("unit", unitP);
  prefs.putBool("sound", soundOn);
  prefs.putBool("wifi", wifiOn);
  prefs.putBool("inv", panelInvert);
  prefs.putBool("bgr", panelBgr);
  prefs.putInt("tthr", techThr);
  if (touchCalValid) prefs.putBytes("tcal", touchCal, sizeof(touchCal));
  prefs.end();
}

// ---------------- الربط مع وحدة الراديو ----------------
SemaphoreHandle_t linkTxMux;
void linkSend(JsonDocument& d) {
  char buf[256];
  size_t n = serializeJson(d, buf, sizeof(buf) - 6);
  uint8_t x = 0;
  for (size_t i = 0; i < n; i++) x ^= (uint8_t)buf[i];
  if (xSemaphoreTake(linkTxMux, pdMS_TO_TICKS(100)) == pdTRUE) {
    Link.write('$'); Link.write((const uint8_t*)buf, n); Link.printf("*%02X\n", x);
    xSemaphoreGive(linkTxMux);
  }
}
void sendCfg(const char* key, float v) { JsonDocument d; d["c"] = "cfg"; d[key] = v; linkSend(d); }
void sendCmd(const char* c, int on = -1) { JsonDocument d; d["c"] = c; if (on >= 0) d["on"] = on; linkSend(d); }

// ---- كشف التسريب والإنذارات (نفس منطق R3) ----
bool leakCheck(TireState& t, float bar, uint32_t now) {
  if (isnan(bar)) return false;
  uint8_t last = (t.hh + 31) % 32;
  if (t.hn == 0 || now - t.ht[last] >= 20000) { t.hb[t.hh] = bar; t.ht[t.hh] = now; t.hh = (t.hh + 1) % 32; if (t.hn < 32) t.hn++; }
  float mx = bar;
  for (uint8_t i = 0; i < t.hn; i++) { uint8_t k = (t.hh + 31 - i) % 32; if (now - t.ht[k] > 600000UL) break; mx = max(mx, t.hb[k]); }
  return mx - bar >= 0.20f;
}
uint8_t computeAlarms(int i, float bar, float temp, int8_t batt, bool leak, uint8_t prev) {
  const TireCfg& c = tcfg[i];
  uint8_t a = 0;
  if (!isnan(bar)) {
    if ((prev & AL_LOW) ? bar < c.pMin + 0.05f : bar < c.pMin) a |= AL_LOW;
    if ((prev & AL_HIGH) ? bar > c.pMax - 0.05f : bar > c.pMax) a |= AL_HIGH;
  }
  if (!isnan(temp) && ((prev & AL_HOT) ? temp > c.tMax - 2 : temp > c.tMax)) a |= AL_HOT;
  if (batt == 0) a |= AL_BATT;
  if (leak) a |= AL_LEAK;
  return a;
}

void techConsider(const char* id, int p, int tire, float bar, float temp, int8_t batt, int rssi, uint32_t now) {
  bool same = tech.valid && !strcmp(tech.id, id);
  if (!same && tech.valid) {
    if (now - tech.lastMs < 10000 && rssi < tech.rssi + 3) return;
    for (int i = min(techHistN, 9); i > 0; i--) techHist[i] = techHist[i - 1];
    techHist[0] = tech; if (techHistN < 10) techHistN++;
  }
  if (!same) { memset(&tech, 0, sizeof(tech)); tech.valid = true; strlcpy(tech.id, id, sizeof(tech.id)); tech.best = rssi; }
  tech.p = p; tech.tire = tire; tech.bar = bar; tech.temp = temp; tech.batt = batt; tech.rssi = rssi;
  if (rssi > tech.best) tech.best = rssi;
  tech.lastMs = now; tech.count++;
}

void beep(int count, int onMs, int freq);
void handlePacket(JsonDocument& d) {
  const char* id = d["id"] | "";
  if (!id[0]) return;
  int p = d["p"] | -1;
  float bar = d["k"].isNull() ? NAN : d["k"].as<float>() / 100.0f;
  float temp = d["c"].isNull() ? NAN : d["c"].as<float>();
  int8_t batt = d["b"] | -1;
  int rssi = d["r"] | -120;
  bool hwLeak = (d["lk"] | 0) != 0;
  uint32_t now = millis();
  xSemaphoreTake(dataMux, portMAX_DELAY);
  int tire = findTire(id);
  if (tire >= 0) {
    TireState& t = ts[tire];
    bool leak = leakCheck(t, bar, now) || hwLeak;
    uint8_t before = t.alarms;
    t.alarms = computeAlarms(tire, bar, temp, batt, leak, t.alarms);
    t.has = true; t.bar = bar; t.temp = temp; t.batt = batt; t.rssi = rssi; t.lastMs = now; t.count++; t.hwLeak = hwLeak;
    if (!isnan(bar) && (t.tn == 0 || now - t.trendT >= 60000)) { t.trend[t.th] = bar; t.th = (t.th + 1) % 60; if (t.tn < 60) t.tn++; t.trendT = now; }
    (void)before;
  }
  int k = -1;
  for (int i = 0; i < scanN; i++) if (!strcmp(scan[i].id, id)) { k = i; break; }
  if (k < 0) {
    if (scanN < MAX_SCAN) k = scanN++;
    else { k = 0; for (int i = 1; i < MAX_SCAN; i++) if ((int32_t)(scan[i].last - scan[k].last) < 0) k = i; }
    memset(&scan[k], 0, sizeof(Scan)); strlcpy(scan[k].id, id, sizeof(scan[k].id)); scan[k].first = now; scan[k].best = rssi;
  }
  Scan& s = scan[k];
  s.p = p; s.tire = tire; s.batt = batt; s.bar = bar; s.temp = temp; s.rssi = rssi; s.last = now; s.count++;
  if (rssi > s.best) s.best = rssi;
  if (page == PG_TECH && rssi >= techThr) {
    bool wasNew = !tech.valid || strcmp(tech.id, id);
    techConsider(id, p, tire, bar, temp, batt, rssi, now);
    if (wasNew && tech.valid && !strcmp(tech.id, id)) beep(1, 60, 2700);
  }
  xSemaphoreGive(dataMux);
  uiDirty = true;
}

void handleLinkJson(JsonDocument& d) {
  const char* t = d["t"] | "";
  linkLastMs = millis();
  if (!strcmp(t, "P")) handlePacket(d);
  else if (!strcmp(t, "S")) {
    xSemaphoreTake(dataMux, portMAX_DELAY);
    radio.cc = (d["cc"] | 0) != 0; radio.err = d["err"] | 0; radio.f = d["f"] | 433.92f; radio.foff = d["foff"] | 0;
    radio.bw = d["bw"] | 0; radio.d = d["d"] | 0; radio.act = d["act"] | 0; radio.n = d["n"] | 0; radio.th = d["th"] | 0;
    radio.cur = d["cur"] | 0; radio.cap = d["cap"] | 0; radio.dec = d["dec"] | 0; radio.unk = d["unk"] | 0;
    radio.lost = d["lost"] | 0; radio.pk = d["pk"] | 0; radio.rej = d["rej"] | 0; radio.rst = d["rst"] | 0;
    radio.up = d["up"] | 0; radio.heap = d["heap"] | 0; strlcpy(radio.fw, d["fw"] | "", sizeof(radio.fw));
    xSemaphoreGive(dataMux);
    if (!decKnown) sendCmd("dec");
    if (page == PG_RADIO || page == PG_TECH) sendCmd("live", 1);
  } else if (!strcmp(t, "L")) {
    liveRssi = d["cur"] | -120; livePeak = d["pk"] | -120;
    rssiHist[rssiHistHead] = livePeak; rssiHistHead = (rssiHistHead + 1) % 120; if (rssiHistN < 120) rssiHistN++;
    radio.n = d["n"] | radio.n; radio.th = d["th"] | radio.th;
    if (page == PG_RADIO || page == PG_TECH) uiDirty = true;
  } else if (!strcmp(t, "D")) {
    int i = d["i"] | -1;
    if (i >= 0 && i < 25) {
      strlcpy(decName[i], d["n"] | "", sizeof(decName[i]));
      strlcpy(decShort[i], d["s"] | "", sizeof(decShort[i]));
      if (i == 24) decKnown = true;
    }
  }
}

void linkTask(void*) {
  esp_task_wdt_add(NULL);
  static char line[700];
  size_t len = 0;
  for (;;) {
    esp_task_wdt_reset();
    while (Link.available()) {
      char ch = (char)Link.read();
      if (ch == '\n' || ch == '\r') {
        if (len > 4 && line[0] == '$') {
          line[len] = 0;
          char* star = strrchr(line, '*');
          bool ok = false;
          if (star) {
            *star = 0;
            uint8_t x = 0;
            for (char* q = line + 1; *q; q++) x ^= (uint8_t)*q;
            ok = x == (uint8_t)strtoul(star + 1, nullptr, 16);
          }
          if (ok) {
            JsonDocument d;
            if (deserializeJson(d, line + 1) == DeserializationError::Ok) { linkGood++; handleLinkJson(d); } else linkBad++;
          } else linkBad++;
        }
        len = 0;
      } else if (len < sizeof(line) - 1) line[len++] = ch;
      else len = 0;
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// =============================================================================
//  الرسم — كل الدوال ترسم في الشريط band مع إزاحة رأسية oy
// =============================================================================
#define Y(v) ((v) - oy)

void drawSignalBars(LGFX_Sprite& s, int oy, int x, int y, int rssi, uint16_t col) {
  int lvl = rssi >= -60 ? 4 : rssi >= -72 ? 3 : rssi >= -84 ? 2 : rssi >= -96 ? 1 : 0;
  for (int i = 0; i < 4; i++) {
    int h = 3 + i * 3;
    if (i < lvl) s.fillRect(x + i * 5, Y(y + 12 - h), 3, h, col);
    else s.drawRect(x + i * 5, Y(y + 12 - h), 3, h, C_LINE);
  }
}

void drawTopBar(LGFX_Sprite& s, int oy) {
  if (oy > TOP_H) return;
  bool link = millis() - linkLastMs < 4000;
  s.fillRect(0, Y(0), SCR_W, TOP_H, alarmActive && !alarmAcked && (millis() / 500) % 2 ? 0x6000 : C_PANEL);
  s.drawFastHLine(0, Y(TOP_H - 1), SCR_W, C_LINE);
  s.setFont(&fonts::Font2);
  s.setTextDatum(middle_left);
  if (alarmActive) {
    s.setTextColor(C_DANGER);
    int bad = -1;
    for (int i = 0; i < 5; i++) if (ts[i].has && !stale(ts[i]) && (ts[i].alarms & AL_DANGER)) { bad = i; break; }
    char b[40];
    snprintf(b, sizeof(b), "! %s %s", bad >= 0 ? TIRE_LBL[bad] : "", bad >= 0 ? alarmText(ts[bad].alarms) : "ALARM");
    s.drawString(b, 6, Y(11));
    addBtn(0, 0, 200, TOP_H, 900);   // لمس الشريط = إسكات الإنذار
  } else {
    s.setTextColor(C_TEXT);
    s.drawString("TPMS PRO", 6, Y(11));
    s.setTextColor(C_ACCENT);
    s.drawString("V3", 78, Y(11));
  }
  // حالة الربط والراديو والواي فاي
  s.setTextDatum(middle_right);
  int x = SCR_W - 6;
  s.setTextColor(wifiOn ? C_ACCENT : C_MUTED);
  s.drawString(wifiOn ? "WiFi" : "----", x, Y(11)); x -= 38;
  s.fillCircle(x, Y(11), 4, link && radio.cc ? C_OK : link ? C_WARN : C_DANGER);
  s.setTextColor(C_MUTED);
  s.drawString(link ? (radio.cc ? "RF" : "RF!") : "LINK", x - 7, Y(11)); x -= 46;
  char b[16];
  snprintf(b, sizeof(b), "%lu", (unsigned long)radio.pk);
  s.drawString(b, x, Y(11));
}

void drawNav(LGFX_Sprite& s, int oy) {
  if (oy + BAND_H < NAV_Y) return;
  static const char* lbl[PG_COUNT] = {"CAR", "SCAN", "RADIO", "TECH", "SETUP"};
  s.fillRect(0, Y(NAV_Y), SCR_W, NAV_H, C_PANEL);
  s.drawFastHLine(0, Y(NAV_Y), SCR_W, C_LINE);
  int w = SCR_W / PG_COUNT;
  s.setFont(&fonts::Font2);
  s.setTextDatum(middle_center);
  for (int i = 0; i < PG_COUNT; i++) {
    bool on = page == i;
    if (on) { s.fillRoundRect(i * w + 3, Y(NAV_Y + 4), w - 6, NAV_H - 8, 6, 0x1A4F); s.drawRoundRect(i * w + 3, Y(NAV_Y + 4), w - 6, NAV_H - 8, 6, C_ACCENT); }
    s.setTextColor(on ? C_TEXT : C_MUTED);
    s.drawString(lbl[i], i * w + w / 2, Y(NAV_Y + NAV_H / 2));
    addBtn(i * w, NAV_Y, w, NAV_H, i);
  }
}

void drawCarBody(LGFX_Sprite& s, int oy, int cx, int cy) {
  s.fillRoundRect(cx - 26, Y(cy - 72), 52, 144, 18, 0x0841);
  s.fillRoundRect(cx - 22, Y(cy - 68), 44, 136, 16, C_CAR);
  s.drawRoundRect(cx - 22, Y(cy - 68), 44, 136, 16, 0x8C71);
  s.fillRoundRect(cx - 17, Y(cy - 40), 34, 26, 7, C_GLASS);
  s.fillRoundRect(cx - 16, Y(cy + 22), 32, 20, 6, C_GLASS);
  s.fillRoundRect(cx - 19, Y(cy - 66), 9, 4, 2, 0xFFF0);
  s.fillRoundRect(cx + 10, Y(cy - 66), 9, 4, 2, 0xFFF0);
  s.fillRoundRect(cx - 19, Y(cy + 63), 9, 3, 1, C_DANGER);
  s.fillRoundRect(cx + 10, Y(cy + 63), 9, 3, 1, C_DANGER);
}

void drawWheel(LGFX_Sprite& s, int oy, int x, int y, uint16_t col) {
  s.fillRoundRect(x, Y(y), 10, 28, 4, 0x18C3);
  s.drawRoundRect(x - 1, Y(y - 1), 12, 30, 5, col);
  s.fillRect(x + 3, Y(y + 5), 4, 18, col);
}

void drawTireCard(LGFX_Sprite& s, int oy, int x, int y, int w, int h, int i) {
  if (y > oy + BAND_H || y + h < oy) return;
  const TireState& t = ts[i];
  uint16_t col = tireColor(t);
  bool danger = t.has && !stale(t) && (t.alarms & AL_DANGER);
  bool blink = danger && (millis() / 500) % 2;
  s.fillRoundRect(x, Y(y), w, h, 8, blink ? 0x3000 : C_PANEL);
  s.drawRoundRect(x, Y(y), w, h, 8, col);
  if (danger) s.drawRoundRect(x + 1, Y(y + 1), w - 2, h - 2, 7, col);
  s.setFont(&fonts::Font0);
  s.setTextDatum(top_left);
  s.setTextColor(C_MUTED);
  s.drawString(TIRE_NAME[i], x + 7, Y(y + 6));
  if (t.has) drawSignalBars(s, oy, x + w - 25, y + 4, t.rssi, col);

  s.setTextDatum(baseline_left);
  s.setFont(&fonts::FreeSansBold18pt7b);
  s.setTextColor(t.has ? col : C_MUTED);
  char b[24];
  if (t.has && !isnan(t.bar)) snprintf(b, sizeof(b), "%.*f", pDec(), pU(t.bar)); else strcpy(b, "--");
  s.drawString(b, x + 7, Y(y + 44));
  int tw = s.textWidth(b);
  s.setFont(&fonts::Font2);
  s.setTextColor(C_MUTED);
  s.drawString(pUnit(), x + 11 + tw, Y(y + 44));

  s.setFont(&fonts::Font2);
  s.setTextDatum(top_left);
  if (!t.has) {
    s.setTextColor(C_MUTED);
    s.drawString(tcfg[i].id[0] ? "WAITING..." : "NOT ASSIGNED", x + 7, Y(y + h - 20));
  } else {
    char age[8];
    fmtAge(millis() - t.lastMs, age, sizeof(age));
    if (stale(t)) { s.setTextColor(C_MUTED); snprintf(b, sizeof(b), "NO SIGNAL %s", age); }
    else if (t.alarms) { s.setTextColor(col); snprintf(b, sizeof(b), "%s", alarmText(t.alarms)); }
    else { s.setTextColor(C_TEXT); snprintf(b, sizeof(b), "%s%dC  %s", isnan(t.temp) ? "--" : "", isnan(t.temp) ? 0 : (int)lroundf(t.temp), age); if (isnan(t.temp)) snprintf(b, sizeof(b), "--C  %s", age); }
    s.drawString(b, x + 7, Y(y + h - 20));
  }
}

void pageCar(LGFX_Sprite& s, int oy) {
  const int cw = 118, chh = 78;
  drawTireCard(s, oy, 4, 26, cw, chh, 0);
  drawTireCard(s, oy, SCR_W - cw - 4, 26, cw, chh, 1);
  drawTireCard(s, oy, 4, 26 + chh + 4, cw, chh, 2);
  drawTireCard(s, oy, SCR_W - cw - 4, 26 + chh + 4, cw, chh, 3);
  int cx = SCR_W / 2, cy = 106;
  drawCarBody(s, oy, cx, cy);
  drawWheel(s, oy, cx - 33, cy - 52, tireColor(ts[0]));
  drawWheel(s, oy, cx + 23, cy - 52, tireColor(ts[1]));
  drawWheel(s, oy, cx - 33, cy + 22, tireColor(ts[2]));
  drawWheel(s, oy, cx + 23, cy + 22, tireColor(ts[3]));
  // الإطار الاحتياطي
  const TireState& sp = ts[4];
  int y = 26 + 2 * chh + 5;
  if (y + 18 >= oy && y < oy + BAND_H) {
    s.setFont(&fonts::Font2);
    s.setTextDatum(middle_center);
    char b[48];
    if (sp.has && !isnan(sp.bar)) {
      char age[8]; fmtAge(millis() - sp.lastMs, age, sizeof(age));
      snprintf(b, sizeof(b), "SPARE  %.*f %s  %dC  %s", pDec(), pU(sp.bar), pUnit(), isnan(sp.temp) ? 0 : (int)sp.temp, age);
      s.setTextColor(tireColor(sp));
    } else { strcpy(b, "SPARE: STANDBY"); s.setTextColor(C_MUTED); }
    s.drawString(b, SCR_W / 2, Y(y + 9));
  }
}

int scanOrder[MAX_SCAN];
int buildScanOrder() {
  int n = scanN;
  for (int i = 0; i < n; i++) scanOrder[i] = i;
  for (int i = 1; i < n; i++) { int k = scanOrder[i], j = i - 1; while (j >= 0 && (int32_t)(scan[scanOrder[j]].last - scan[k].last) < 0) { scanOrder[j + 1] = scanOrder[j]; j--; } scanOrder[j + 1] = k; }
  return n;
}

void pageScan(LGFX_Sprite& s, int oy) {
  int n = buildScanOrder();
  const int rowH = 25, y0 = 44, rows = 6;
  if (scanScroll > max(0, n - rows)) scanScroll = max(0, n - rows);
  s.setFont(&fonts::Font0);
  s.setTextDatum(middle_left);
  s.setTextColor(C_ACCENT);
  s.fillRect(0, Y(TOP_H + 2), SCR_W, 16, C_PANEL2);
  s.drawString("ID", 6, Y(32)); s.drawString("MODEL", 80, Y(32)); s.drawString(pUnit(), 136, Y(32));
  s.drawString("TEMP", 182, Y(32)); s.drawString("SIGNAL", 220, Y(32)); s.drawString("AGE", 270, Y(32));
  if (n == 0) { s.setFont(&fonts::Font2); s.setTextColor(C_MUTED); s.setTextDatum(middle_center); s.drawString("Listening for sensors...", SCR_W / 2, Y(120)); return; }
  for (int r = 0; r < rows && scanScroll + r < n; r++) {
    const Scan& c = scan[scanOrder[scanScroll + r]];
    int y = y0 + r * rowH;
    s.fillRoundRect(2, Y(y), SCR_W - 30, rowH - 3, 4, r % 2 ? C_BG : C_PANEL);
    uint16_t col = c.tire >= 0 ? C_OK : (c.p == 0 || c.p == 1) ? C_WARN : C_TEXT;
    s.setFont(&fonts::Font2);
    s.setTextColor(col);
    s.drawString(c.id, 6, Y(y + 11));
    s.setFont(&fonts::Font0);
    char mdl[9]; strlcpy(mdl, c.tire >= 0 ? TIRE_LBL[c.tire] : protoShort(c.p), sizeof(mdl));
    s.drawString(mdl, 80, Y(y + 11));
    char b[16];
    if (!isnan(c.bar)) snprintf(b, sizeof(b), "%.*f", pDec(), pU(c.bar)); else strcpy(b, "--");
    s.setFont(&fonts::Font2); s.drawString(b, 134, Y(y + 11));
    if (!isnan(c.temp)) snprintf(b, sizeof(b), "%dC", (int)c.temp); else strcpy(b, "--");
    s.drawString(b, 182, Y(y + 11));
    drawSignalBars(s, oy, 222, y + 4, c.rssi, col);
    char age[8]; fmtAge(millis() - c.last, age, sizeof(age));
    s.setFont(&fonts::Font0); s.setTextColor(C_MUTED); s.drawString(age, 270, Y(y + 11));
    addBtn(2, y, SCR_W - 30, rowH - 3, 100 + scanOrder[scanScroll + r]);
  }
  // شريط التمرير
  s.fillRoundRect(SCR_W - 24, Y(y0), 20, 70, 5, C_PANEL2); s.drawString("^", SCR_W - 17, Y(y0 + 35));
  s.fillRoundRect(SCR_W - 24, Y(y0 + 76), 20, 70, 5, C_PANEL2); s.drawString("v", SCR_W - 17, Y(y0 + 111));
  addBtn(SCR_W - 26, y0, 24, 72, 150); addBtn(SCR_W - 26, y0 + 76, 24, 72, 151);
  char pos[24]; snprintf(pos, sizeof(pos), "%d-%d / %d", scanScroll + 1, min(n, scanScroll + rows), n);
  s.setTextColor(C_MUTED); s.drawString(pos, 6, Y(NAV_Y - 6));
}

void drawPopup(LGFX_Sprite& s, int oy) {
  if (popupScan < 0 || popupScan >= scanN) return;
  const Scan& c = scan[popupScan];
  int x = 20, y = 34, w = SCR_W - 40, h = 166;
  s.fillRoundRect(x, Y(y), w, h, 10, C_PANEL2);
  s.drawRoundRect(x, Y(y), w, h, 10, C_ACCENT);
  s.setTextDatum(top_left);
  s.setFont(&fonts::FreeSansBold12pt7b); s.setTextColor(C_TEXT);
  s.drawString(c.id, x + 12, Y(y + 10));
  s.setFont(&fonts::Font2); s.setTextColor(C_MUTED);
  char b[64];
  snprintf(b, sizeof(b), "%s  |  best %d dBm  |  %lu pkts", protoShort(c.p), c.best, (unsigned long)c.count);
  s.drawString(b, x + 12, Y(y + 38));
  if (!isnan(c.bar)) snprintf(b, sizeof(b), "%.*f %s   %dC   battery %s", pDec(), pU(c.bar), pUnit(), isnan(c.temp) ? 0 : (int)c.temp, c.batt == 0 ? "LOW" : c.batt == 1 ? "OK" : "n/a");
  else strcpy(b, "no pressure data");
  s.setTextColor(C_TEXT); s.drawString(b, x + 12, Y(y + 58));
  s.setTextColor(C_ACCENT); s.drawString("Assign this sensor to:", x + 12, Y(y + 82));
  for (int i = 0; i < 5; i++) {
    int bx = x + 10 + i * 54, by = y + 102;
    bool cur = c.tire == i;
    s.fillRoundRect(bx, Y(by), 48, 28, 6, cur ? C_OK : 0x2108);
    s.drawRoundRect(bx, Y(by), 48, 28, 6, C_ACCENT);
    s.setTextDatum(middle_center); s.setTextColor(cur ? C_BG : C_TEXT);
    s.drawString(TIRE_LBL[i], bx + 24, Y(by + 14));
    addBtn(bx, by, 48, 28, 200 + i);
  }
  s.fillRoundRect(x + w - 70, Y(y + h - 30), 60, 22, 6, C_LINE);
  s.setTextColor(C_TEXT); s.drawString("CLOSE", x + w - 40, Y(y + h - 19));
  addBtn(x + w - 70, y + h - 30, 60, 22, 210);
}

void drawStepper(LGFX_Sprite& s, int oy, int x, int y, const char* label, const char* value, uint16_t idDn, uint16_t idUp) {
  s.setFont(&fonts::Font0); s.setTextDatum(top_left); s.setTextColor(C_MUTED);
  s.drawString(label, x, Y(y));
  s.fillRoundRect(x, Y(y + 10), 30, 26, 6, C_PANEL2); s.drawRoundRect(x, Y(y + 10), 30, 26, 6, C_LINE);
  s.fillRoundRect(x + 72, Y(y + 10), 30, 26, 6, C_PANEL2); s.drawRoundRect(x + 72, Y(y + 10), 30, 26, 6, C_LINE);
  s.setFont(&fonts::Font2); s.setTextDatum(middle_center); s.setTextColor(C_TEXT);
  s.drawString("-", x + 15, Y(y + 23)); s.drawString("+", x + 87, Y(y + 23));
  s.setTextColor(C_ACCENT); s.drawString(value, x + 51, Y(y + 23));
  addBtn(x, y + 10, 30, 26, idDn); addBtn(x + 72, y + 10, 30, 26, idUp);
}

void drawMeter(LGFX_Sprite& s, int oy, int x, int y, int w, int h, int val, int noise, int thr) {
  auto X = [&](int dbm) { return x + (int)((long)(constrain(dbm, -110, -30) + 110) * w / 80); };
  s.fillRoundRect(x, Y(y), w, h, 4, C_PANEL);
  int px = X(val);
  for (int xx = x + 1; xx < px; xx += 3) {   // تدرج مقطّع (مثل أجهزة القياس)
    float f = (float)(xx - x) / w;
    uint16_t col = f < 0.35f ? C_BLUE : f < 0.7f ? C_OK : C_WARN;
    s.fillRect(xx, Y(y + 2), 2, h - 4, col);
  }
  s.drawFastVLine(X(noise), Y(y - 3), h + 6, C_MUTED);
  s.drawFastVLine(X(thr), Y(y - 3), h + 6, C_DANGER);
  s.drawRoundRect(x, Y(y), w, h, 4, C_LINE);
}

void pageRadio(LGFX_Sprite& s, int oy) {
  s.setTextDatum(baseline_left);
  s.setFont(&fonts::FreeSansBold24pt7b);
  s.setTextColor(livePeak > radio.th ? C_OK : C_TEXT);
  char b[48];
  snprintf(b, sizeof(b), "%d", livePeak);
  s.drawString(b, 8, Y(74));
  int vw = s.textWidth(b);
  s.setFont(&fonts::Font2); s.setTextColor(C_MUTED);
  s.drawString("dBm", 14 + vw, Y(74));
  s.setTextDatum(baseline_right);
  s.setTextColor(livePeak > radio.th ? C_OK : C_MUTED);
  s.drawString(livePeak > radio.th ? "SIGNAL" : "LISTENING", SCR_W - 8, Y(52));
  s.setTextColor(C_MUTED);
  snprintf(b, sizeof(b), "margin %+d dB", livePeak - radio.n);
  s.drawString(b, SCR_W - 8, Y(72));
  s.setTextDatum(baseline_left);
  drawMeter(s, oy, 8, 86, 304, 16, livePeak, radio.n, radio.th);
  // منحنى RSSI لآخر ~18 ثانية
  int gx = 8, gy = 108, gw = 150, gh = 46;
  s.drawRoundRect(gx, Y(gy), gw, gh, 4, C_LINE);
  for (int i = 1; i < rssiHistN; i++) {
    int a = (rssiHistHead - rssiHistN + i - 1 + 240) % 120, c = (rssiHistHead - rssiHistN + i + 240) % 120;
    int x0 = gx + (i - 1) * gw / 120, x1 = gx + i * gw / 120;
    int y0 = gy + gh - 2 - (constrain(rssiHist[a], -110, -30) + 110) * (gh - 4) / 80;
    int y1 = gy + gh - 2 - (constrain(rssiHist[c], -110, -30) + 110) * (gh - 4) / 80;
    s.drawLine(x0, Y(y0), x1, Y(y1), C_ACCENT);
  }
  int thY = gy + gh - 2 - (constrain(radio.th, -110, -30) + 110) * (gh - 4) / 80;
  s.drawFastHLine(gx + 1, Y(thY), gw - 2, 0x6000);
  s.setFont(&fonts::Font0); s.setTextDatum(top_left); s.setTextColor(C_TEXT);
  snprintf(b, sizeof(b), "NOISE %d  THR %d dBm", radio.n, radio.th);            s.drawString(b, 166, Y(108));
  snprintf(b, sizeof(b), "%.3f MHz  BW %d", radio.f, radio.bw);                   s.drawString(b, 166, Y(120));
  snprintf(b, sizeof(b), "DECODED %lu  PKTS %lu", (unsigned long)radio.dec, (unsigned long)radio.pk); s.drawString(b, 166, Y(132));
  snprintf(b, sizeof(b), "LOST %lu  REJ %lu  RST %lu", (unsigned long)radio.lost, (unsigned long)radio.rej, (unsigned long)radio.rst); s.drawString(b, 166, Y(144));
  char v[16];
  snprintf(v, sizeof(v), "%d", radio.bw);                       drawStepper(s, oy, 8, 158, "BW kHz", v, 300, 301);
  snprintf(v, sizeof(v), "%+d", radio.foff);                     drawStepper(s, oy, 112, 158, "FREQ OFS kHz", v, 302, 303);
  snprintf(v, sizeof(v), "%d dB", radio.d);                      drawStepper(s, oy, 216, 158, "SENS DELTA", v, 304, 305);
}

const char* techVerdict(const TechCap& c, uint16_t* col) {
  *col = C_OK;
  if (c.batt == 0) { *col = C_DANGER; return "BATTERY LOW"; }
  if (isnan(c.bar)) { *col = C_WARN; return "NO PRESSURE DATA"; }
  if (c.bar < 0.3f) { *col = C_WARN; return "0 BAR: NOT MOUNTED?"; }
  if (!isnan(c.temp) && c.temp > 90) { *col = C_DANGER; return "OVERHEAT"; }
  return "SENSOR OK";
}

void pageTech(LGFX_Sprite& s, int oy) {
  char b[64];
  if (!tech.valid) {
    s.setTextDatum(middle_center);
    s.setFont(&fonts::FreeSansBold12pt7b); s.setTextColor(C_TEXT);
    s.drawString("Bring the device", SCR_W / 2, Y(52));
    s.drawString("close to the tire", SCR_W / 2, Y(78));
    s.setFont(&fonts::Font2); s.setTextColor(C_MUTED);
    s.drawString("deflate a little, rotate or use an LF tool", SCR_W / 2, Y(102));
    drawMeter(s, oy, 10, 118, 300, 16, livePeak, radio.n, techThr);
    snprintf(b, sizeof(b), "LIVE %d dBm   capture >= %d dBm", livePeak, techThr);
    s.setTextColor(C_TEXT); s.drawString(b, SCR_W / 2, Y(146));
  } else {
    uint16_t col; const char* v = techVerdict(tech, &col);
    bool flash = millis() - (tech.lastMs - 0) < 1500 && tech.count == 1;
    if (flash) s.drawRoundRect(1, Y(TOP_H + 1), SCR_W - 2, NAV_Y - TOP_H - 2, 6, C_OK);
    s.setTextDatum(top_left);
    s.setFont(&fonts::Font0); s.setTextColor(C_MUTED); s.drawString("SENSOR ID", 10, Y(28));
    s.setTextColor(C_ACCENT); s.drawString(protoShort(tech.p), 200, Y(28));
    if (tech.tire >= 0) { s.setTextColor(C_OK); s.drawString(String("MY ") + TIRE_LBL[tech.tire], 270, Y(28)); }
    s.setFont(&fonts::FreeSansBold18pt7b); s.setTextColor(C_TEXT); s.drawString(tech.id, 10, Y(38));
    s.setFont(&fonts::FreeSansBold24pt7b); s.setTextColor(col);
    if (!isnan(tech.bar)) snprintf(b, sizeof(b), "%.*f", pDec(), pU(tech.bar)); else strcpy(b, "--");
    s.drawString(b, 10, Y(74));
    int tw = s.textWidth(b);
    s.setFont(&fonts::Font2); s.setTextColor(C_MUTED); s.drawString(pUnit(), 16 + tw, Y(96));
    snprintf(b, sizeof(b), "%dC", isnan(tech.temp) ? 0 : (int)tech.temp);
    s.setFont(&fonts::FreeSansBold12pt7b); s.setTextColor(C_TEXT); s.drawString(b, 230, Y(82));
    s.setFont(&fonts::Font0);
    snprintf(b, sizeof(b), "SIGNAL %d (BEST %d)  BATT %s  PKTS %lu", tech.rssi, tech.best,
             tech.batt == 1 ? "OK" : tech.batt == 0 ? "LOW" : "N/A", (unsigned long)tech.count);
    s.setTextColor(C_TEXT); s.drawString(b, 10, Y(122));
    s.fillRoundRect(10, Y(136), 300, 28, 8, col == C_OK ? 0x0320 : col == C_DANGER ? 0x5000 : 0x4A00);
    s.drawRoundRect(10, Y(136), 300, 28, 8, col);
    s.setFont(&fonts::FreeSansBold9pt7b); s.setTextDatum(middle_center); s.setTextColor(col);
    s.drawString(v, SCR_W / 2, Y(150));
    if (techHistN) { s.setFont(&fonts::Font0); s.setTextColor(C_MUTED); s.setTextDatum(top_left);
      snprintf(b, sizeof(b), "PREV %s  %s", techHist[0].id, isnan(techHist[0].bar) ? "--" : String(pU(techHist[0].bar), pDec()).c_str());
      s.drawString(b, 10, Y(168)); }
  }
  char v[16]; snprintf(v, sizeof(v), "%d", techThr);
  drawStepper(s, oy, 10, 168 + (tech.valid ? 8 : 0) - 8, "NEAR dBm", v, 400, 401);
  s.fillRoundRect(200, Y(178), 110, 26, 6, C_PANEL2); s.drawRoundRect(200, Y(178), 110, 26, 6, C_ACCENT);
  s.setFont(&fonts::Font2); s.setTextDatum(middle_center); s.setTextColor(C_TEXT); s.drawString("NEW CAPTURE", 255, Y(191));
  addBtn(200, 178, 110, 26, 402);
}

void drawToggle(LGFX_Sprite& s, int oy, int x, int y, int w, const char* label, const char* value, bool on, uint16_t id) {
  s.fillRoundRect(x, Y(y), w, 30, 6, C_PANEL);
  s.drawRoundRect(x, Y(y), w, 30, 6, on ? C_ACCENT : C_LINE);
  s.setFont(&fonts::Font0); s.setTextDatum(top_left); s.setTextColor(C_MUTED); s.drawString(label, x + 6, Y(y + 4));
  s.setFont(&fonts::Font2); s.setTextColor(on ? C_ACCENT : C_TEXT); s.drawString(value, x + 6, Y(y + 14));
  addBtn(x, y, w, 30, id);
}

void pageSetup(LGFX_Sprite& s, int oy) {
  static const char* tabs[4] = {"DISPLAY", "TIRES", "LIMITS", "SYSTEM"};
  for (int i = 0; i < 4; i++) {
    int x = 4 + i * 79;
    s.fillRoundRect(x, Y(26), 75, 22, 5, setupTab == i ? 0x1A4F : C_PANEL);
    s.drawRoundRect(x, Y(26), 75, 22, 5, setupTab == i ? C_ACCENT : C_LINE);
    s.setFont(&fonts::Font2); s.setTextDatum(middle_center); s.setTextColor(setupTab == i ? C_TEXT : C_MUTED);
    s.drawString(tabs[i], x + 37, Y(37));
    addBtn(x, 26, 75, 22, 500 + i);
  }
  char v[40];
  if (setupTab == 0) {
    snprintf(v, sizeof(v), "%d%%", brightness * 100 / 255);
    drawStepper(s, oy, 8, 54, "BRIGHTNESS", v, 510, 511);
    drawToggle(s, oy, 120, 64, 92, "AUTO (LIGHT)", autoBright ? "ON" : "OFF", autoBright, 512);
    drawToggle(s, oy, 220, 64, 92, "UNITS", pUnit(), true, 513);
    drawToggle(s, oy, 8, 104, 100, "ALARM SOUND", soundOn ? "ON" : "OFF", soundOn, 514);
    drawToggle(s, oy, 116, 104, 96, "COLOR INVERT", panelInvert ? "ON" : "OFF", panelInvert, 515);
    drawToggle(s, oy, 220, 104, 92, "RGB ORDER", panelBgr ? "BGR" : "RGB", true, 516);
    drawToggle(s, oy, 8, 144, 150, "TOUCH", "CALIBRATE", false, 517);
    drawToggle(s, oy, 164, 144, 148, "TEST", "SOUND / LED", false, 518);
  } else if (setupTab == 1) {
    for (int i = 0; i < 5; i++) {
      int y = 54 + i * 24;
      bool orig = !strcmp(tcfg[i].id, ORIG_IDS[i]) || idMatch(ORIG_IDS[i], tcfg[i].id);
      s.setFont(&fonts::Font2); s.setTextDatum(middle_left);
      s.setTextColor(C_MUTED); s.drawString(TIRE_LBL[i], 10, Y(y + 10));
      s.setTextColor(orig ? C_OK : C_WARN); s.drawString(tcfg[i].id[0] ? tcfg[i].id : "--------", 44, Y(y + 10));
      s.setTextColor(C_MUTED); s.setFont(&fonts::Font0); s.drawString(orig ? "original" : "changed", 140, Y(y + 10));
    }
    drawToggle(s, oy, 200, 54, 112, "RESTORE", "ORIGINAL IDs", false, 520);
    s.setFont(&fonts::Font0); s.setTextColor(C_MUTED); s.setTextDatum(top_left);
    s.drawString("To change: SCAN page,", 200, Y(94)); s.drawString("tap sensor, pick tire", 200, Y(106));
  } else if (setupTab == 2) {
    for (int i = 0; i < 5; i++) {
      int x = 4 + i * 63;
      s.fillRoundRect(x, Y(52), 59, 20, 5, limitsTire == i ? 0x1A4F : C_PANEL);
      s.drawRoundRect(x, Y(52), 59, 20, 5, limitsTire == i ? C_ACCENT : C_LINE);
      s.setFont(&fonts::Font2); s.setTextDatum(middle_center); s.setTextColor(C_TEXT); s.drawString(TIRE_LBL[i], x + 29, Y(62));
      addBtn(x, 52, 59, 20, 530 + i);
    }
    TireCfg& c = tcfg[limitsTire];
    snprintf(v, sizeof(v), "%.*f", pDec(), pU(c.pMin)); drawStepper(s, oy, 8, 80, "MIN PRESSURE", v, 540, 541);
    snprintf(v, sizeof(v), "%.*f", pDec(), pU(c.pMax)); drawStepper(s, oy, 112, 80, "MAX PRESSURE", v, 542, 543);
    snprintf(v, sizeof(v), "%.0fC", c.tMax);           drawStepper(s, oy, 216, 80, "MAX TEMP", v, 544, 545);
    s.setFont(&fonts::Font0); s.setTextColor(C_MUTED); s.setTextDatum(top_left);
    s.drawString("Fast-leak alarm: 0.2 bar drop within 10 min.", 8, Y(130));
  } else {
    drawToggle(s, oy, 8, 54, 150, "WIFI ACCESS POINT", wifiOn ? "ON  TPMS-V3" : "OFF", wifiOn, 550);
    s.setFont(&fonts::Font0); s.setTextDatum(top_left); s.setTextColor(C_TEXT);
    char b[64];
    snprintf(b, sizeof(b), "IP 192.168.4.1  clients %d", wifiOn ? WiFi.softAPgetStationNum() : 0); s.drawString(b, 166, Y(58));
    snprintf(b, sizeof(b), "UI %s  RX %s", FW_VERSION, radio.fw[0] ? radio.fw : "-");              s.drawString(b, 8, Y(94));
    snprintf(b, sizeof(b), "LINK ok %lu bad %lu  heap %u KB", (unsigned long)linkGood, (unsigned long)linkBad, ESP.getFreeHeap() / 1024); s.drawString(b, 8, Y(106));
    snprintf(b, sizeof(b), "RX up %lus heap %d KB  decoders %d", (unsigned long)radio.up, radio.heap, radio.act); s.drawString(b, 8, Y(118));
    drawToggle(s, oy, 8, 136, 150, "RECEIVER", "REBOOT", false, 551);
    drawToggle(s, oy, 164, 136, 148, "DISPLAY", "REBOOT", false, 552);
  }
}

void renderBand(LGFX_Sprite& s, int oy) {
  s.fillScreen(C_BG);
  switch (page) {
    case PG_CAR:   pageCar(s, oy); break;
    case PG_SCAN:  pageScan(s, oy); drawPopup(s, oy); break;
    case PG_RADIO: pageRadio(s, oy); break;
    case PG_TECH:  pageTech(s, oy); break;
    case PG_SETUP: pageSetup(s, oy); break;
  }
  drawTopBar(s, oy);
  drawNav(s, oy);
}

void renderFrame() {
  xSemaphoreTake(dataMux, portMAX_DELAY);
  btnN = 0;
  recordBtns = true;
  for (int b = 0; b < 2; b++) {
    renderBand(band, b * BAND_H);
    band.pushSprite(lcd, 0, b * BAND_H);
  }
  recordBtns = false;
  xSemaphoreGive(dataMux);
}

// ---------------- تصوير الشاشة عبر المنفذ التسلسلي ----------------
void dumpScreen() {
  static const char b64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  Serial.printf("[SHOT-BEGIN %dx%d RGB565LE]\n", SCR_W, SCR_H);
  char line[80]; int n = 0; uint8_t trip[3]; int tn = 0;
  for (int b = 0; b < 2; b++) {
    xSemaphoreTake(dataMux, portMAX_DELAY);
    renderBand(band, b * BAND_H);
    xSemaphoreGive(dataMux);
    for (int y = 0; y < BAND_H; y++) for (int x = 0; x < SCR_W; x++) {
      uint16_t c = band.readPixel(x, y);
      uint8_t bytes[2] = {(uint8_t)(c & 0xFF), (uint8_t)(c >> 8)};
      for (int k = 0; k < 2; k++) {
        trip[tn++] = bytes[k];
        if (tn == 3) {
          uint32_t v = (trip[0] << 16) | (trip[1] << 8) | trip[2];
          line[n++] = b64[(v >> 18) & 63]; line[n++] = b64[(v >> 12) & 63]; line[n++] = b64[(v >> 6) & 63]; line[n++] = b64[v & 63];
          tn = 0;
          if (n >= 76) { line[n] = 0; Serial.println(line); n = 0; }
        }
      }
    }
  }
  if (tn) { uint32_t v = trip[0] << 16 | (tn > 1 ? trip[1] << 8 : 0); line[n++] = b64[(v >> 18) & 63]; line[n++] = b64[(v >> 12) & 63]; line[n++] = tn > 1 ? b64[(v >> 6) & 63] : '='; line[n++] = '='; }
  if (n) { line[n] = 0; Serial.println(line); }
  Serial.println("[SHOT-END]");
}

// =============================================================================
//  الصوت واللمبة والسطوع
// =============================================================================
static int beepLeft = 0, beepOn = 0, beepFreq = 2400; static uint32_t beepNext = 0; static bool beepState = false;
void beep(int count, int onMs, int freq) { if (!soundOn) return; beepLeft = count; beepOn = onMs; beepFreq = freq; beepNext = millis(); beepState = false; }
void serviceBeep() {
  uint32_t now = millis();
  if ((int32_t)(now - beepNext) < 0) return;
  if (beepState) { ledcWriteTone(PIN_SPK, 0); beepState = false; beepNext = now + beepOn; }
  else if (beepLeft > 0) { ledcWriteTone(PIN_SPK, beepFreq); beepState = true; beepLeft--; beepNext = now + beepOn; }
}
void setLed(bool r, bool g, bool b) { digitalWrite(PIN_LED_R, !r); digitalWrite(PIN_LED_G, !g); digitalWrite(PIN_LED_B, !b); }

void serviceAlarmAndLight() {
  static uint32_t lastCheck = 0, lastBeep = 0, lastLdr = 0; static bool prev = false; static float ldrAvg = -1;
  uint32_t now = millis();
  if (now - lastCheck >= 500) {
    lastCheck = now;
    bool active = false;
    xSemaphoreTake(dataMux, portMAX_DELAY);
    for (int i = 0; i < 5; i++) if (ts[i].has && !stale(ts[i]) && (ts[i].alarms & AL_DANGER)) active = true;
    xSemaphoreGive(dataMux);
    if (active && !prev) { alarmAcked = false; beep(3, 300, 2200); lastBeep = now; }
    else if (active && !alarmAcked && now - lastBeep > 60000) { beep(3, 300, 2200); lastBeep = now; }
    if (!active) alarmAcked = false;
    prev = active; alarmActive = active;
    bool link = now - linkLastMs < 4000;
    if (active) setLed((now / 500) % 2 || alarmAcked, false, false);
    else if (!link) setLed((now / 1000) % 2, (now / 1000) % 2, false);   // أصفر وامض: الربط مفقود
    else setLed(false, true, false);
  }
  if (autoBright && now - lastLdr >= 300) {
    lastLdr = now;
    int raw = analogRead(PIN_LDR);           // في CYD: القيمة ترتفع مع الظلام
    ldrAvg = ldrAvg < 0 ? raw : ldrAvg * 0.85f + raw * 0.15f;
    int target = constrain(map((int)ldrAvg, 0, 1500, 255, 25), 25, 255);
    static int cur = -1;
    if (cur < 0 || abs(target - cur) > 6) { cur = target; lcd->setBrightness(cur); }
  }
}

// =============================================================================
//  اللمس والإجراءات
// =============================================================================
void applyAction(uint16_t id) {
  if (id < PG_COUNT) {
    if (page != id) {
      page = id; popupScan = -1;
      sendCmd("live", (page == PG_RADIO || page == PG_TECH) ? 1 : 0);
    }
  } else if (id >= 100 && id < 100 + MAX_SCAN) popupScan = id - 100;
  else if (id == 150) { if (scanScroll > 0) scanScroll--; }
  else if (id == 151) scanScroll++;
  else if (id >= 200 && id < 205 && popupScan >= 0) {
    int t = id - 200;
    xSemaphoreTake(dataMux, portMAX_DELAY);
    for (int i = 0; i < 5; i++) if (i != t && !strcmp(tcfg[i].id, scan[popupScan].id)) tcfg[i].id[0] = 0;
    strlcpy(tcfg[t].id, scan[popupScan].id, sizeof(tcfg[t].id));
    memset(&ts[t], 0, sizeof(TireState));
    for (int i = 0; i < scanN; i++) scan[i].tire = findTire(scan[i].id);
    xSemaphoreGive(dataMux);
    savePrefs(); beep(2, 60, 2700); popupScan = -1;
  } else if (id == 210) popupScan = -1;
  else if (id >= 300 && id <= 305) {
    static const int bws[] = {58, 68, 81, 102, 116, 135, 162, 203, 232, 270, 325};
    int k = 0; for (int i = 0; i < 11; i++) if (bws[i] == radio.bw) k = i;
    if (id == 300) sendCfg("bw", bws[max(0, k - 1)]);
    if (id == 301) sendCfg("bw", bws[min(10, k + 1)]);
    if (id == 302) sendCfg("foff", radio.foff - 5);
    if (id == 303) sendCfg("foff", radio.foff + 5);
    if (id == 304) sendCfg("d", radio.d - 1);
    if (id == 305) sendCfg("d", radio.d + 1);
  }
  else if (id == 400) { techThr = max(-100, techThr - 5); savePrefs(); }
  else if (id == 401) { techThr = min(-20, techThr + 5); savePrefs(); }
  else if (id == 402) { xSemaphoreTake(dataMux, portMAX_DELAY); if (tech.valid) { for (int i = min(techHistN, 9); i > 0; i--) techHist[i] = techHist[i - 1]; techHist[0] = tech; if (techHistN < 10) techHistN++; } tech.valid = false; xSemaphoreGive(dataMux); }
  else if (id >= 500 && id < 504) setupTab = id - 500;
  else if (id == 510) { autoBright = false; brightness = max(20, brightness - 25); lcd->setBrightness(brightness); savePrefs(); }
  else if (id == 511) { autoBright = false; brightness = min(255, brightness + 25); lcd->setBrightness(brightness); savePrefs(); }
  else if (id == 512) { autoBright = !autoBright; if (!autoBright) lcd->setBrightness(brightness); savePrefs(); }
  else if (id == 513) { unitP = (unitP + 1) % 3; savePrefs(); }
  else if (id == 514) { soundOn = !soundOn; savePrefs(); }
  else if (id == 515) { panelInvert = !panelInvert; lcd->invertDisplay(panelInvert); savePrefs(); }
  else if (id == 516) { panelBgr = !panelBgr; savePrefs(); Serial.println("[UI] RGB order change applies after restart"); }
  else if (id == 517) calibrateRequest = true;
  else if (id == 518) { bool s = soundOn; soundOn = true; beep(2, 120, 2400); soundOn = s; setLed(true, false, false); delay(150); setLed(false, true, false); delay(150); setLed(false, false, true); delay(150); }
  else if (id == 520) { xSemaphoreTake(dataMux, portMAX_DELAY); for (int i = 0; i < 5; i++) { strlcpy(tcfg[i].id, ORIG_IDS[i], sizeof(tcfg[i].id)); memset(&ts[i], 0, sizeof(TireState)); } for (int i = 0; i < scanN; i++) scan[i].tire = findTire(scan[i].id); xSemaphoreGive(dataMux); savePrefs(); beep(2, 60, 2700); }
  else if (id >= 530 && id < 535) limitsTire = id - 530;
  else if (id >= 540 && id <= 545) {
    TireCfg& c = tcfg[limitsTire];
    if (id == 540) c.pMin = max(0.5f, c.pMin - 0.05f);
    if (id == 541) c.pMin = min(c.pMax - 0.1f, c.pMin + 0.05f);
    if (id == 542) c.pMax = max(c.pMin + 0.1f, c.pMax - 0.05f);
    if (id == 543) c.pMax = min(8.0f, c.pMax + 0.05f);
    if (id == 544) c.tMax = max(30.0f, c.tMax - 1);
    if (id == 545) c.tMax = min(130.0f, c.tMax + 1);
    savePrefs();
  }
  else if (id == 550) { wifiOn = !wifiOn; savePrefs(); extern void wifiApply(); wifiApply(); }
  else if (id == 551) sendCmd("reboot");
  else if (id == 552) { savePrefs(); delay(100); ESP.restart(); }
  else if (id == 900) { alarmAcked = true; beepLeft = 0; ledcWriteTone(PIN_SPK, 0); }
  uiDirty = true;
}

void handleTouchAt(int x, int y) {
  for (int i = btnN - 1; i >= 0; i--) {
    const Btn& b = btns[i];
    if (b.w && x >= b.x && x < b.x + b.w && y >= b.y && y < b.y + b.h) {
      if (soundOn) { ledcWriteTone(PIN_SPK, 3200); delay(12); ledcWriteTone(PIN_SPK, 0); }
      applyAction(b.id);
      return;
    }
  }
  if (popupScan >= 0) { popupScan = -1; uiDirty = true; }
}

void uiTask(void*) {
  esp_task_wdt_add(NULL);
  uint32_t lastRender = 0;
  bool wasTouched = false;
  for (;;) {
    esp_task_wdt_reset();
    if (calibrateRequest) {
      calibrateRequest = false;
      esp_task_wdt_delete(NULL);
      lcd->fillScreen(TFT_BLACK);
      lcd->setTextColor(TFT_WHITE); lcd->setFont(&fonts::Font2); lcd->setTextDatum(middle_center);
      lcd->drawString("Touch the arrow tips", SCR_W / 2, SCR_H / 2);
      lcd->calibrateTouch(touchCal, TFT_WHITE, TFT_BLACK, 20);
      touchCalValid = true; savePrefs();
      esp_task_wdt_add(NULL);
      uiDirty = true;
    }
    if (shotRequest) { shotRequest = false; dumpScreen(); }
    if (tapX >= 0) { handleTouchAt(tapX, tapY); tapX = -1; }
    int32_t tx, ty;
    bool touched = lcd->getTouch(&tx, &ty);
    if (touched && !wasTouched) handleTouchAt(tx, ty);
    wasTouched = touched;

    uint32_t now = millis();
    uint32_t interval = (page == PG_RADIO || page == PG_TECH) ? 120 : 250;
    if (uiDirty || now - lastRender >= interval) {
      uiDirty = false;
      lastRender = now;
      renderFrame();
    }
    serviceBeep();
    serviceAlarmAndLight();
    vTaskDelay(pdMS_TO_TICKS(8));
  }
}

// =============================================================================
//  الواي فاي وصفحة الويب
// =============================================================================
WebServer server(80);
DNSServer dns;
bool wifiRunning = false;

#include "web_page.h"

void apiHandler() {
  JsonDocument d;
  uint32_t now = millis();
  xSemaphoreTake(dataMux, portMAX_DELAY);
  d["link"] = now - linkLastMs < 4000;
  d["unit"] = pUnit();
  d["live"] = livePeak;
  JsonObject r = d["radio"].to<JsonObject>();
  r["f"] = serialized(String(radio.f, 3)); r["foff"] = radio.foff; r["bw"] = radio.bw; r["d"] = radio.d; r["n"] = radio.n; r["th"] = radio.th;
  r["dec"] = radio.dec; r["pk"] = radio.pk; r["lost"] = radio.lost; r["rej"] = radio.rej; r["rst"] = radio.rst; r["fw"] = radio.fw; r["up"] = radio.up;
  JsonArray ta = d["tires"].to<JsonArray>();
  for (int i = 0; i < 5; i++) {
    JsonObject o = ta.add<JsonObject>();
    o["id"] = tcfg[i].id; o["has"] = ts[i].has;
    if (ts[i].has) {
      o["p"] = isnan(ts[i].bar) ? String("--") : String(pU(ts[i].bar), pDec());
      o["t"] = isnan(ts[i].temp) ? 0 : (int)ts[i].temp; o["r"] = ts[i].rssi; o["age"] = (now - ts[i].lastMs) / 1000;
      o["al"] = ts[i].alarms; o["alt"] = stale(ts[i]) ? "لا إشارة حديثة" : ts[i].alarms ? alarmText(ts[i].alarms) : "سليم";
    }
  }
  int n = buildScanOrder();
  JsonArray sa = d["scan"].to<JsonArray>();
  for (int i = 0; i < n; i++) {
    const Scan& s = scan[scanOrder[i]];
    JsonObject o = sa.add<JsonObject>();
    o["id"] = s.id; o["m"] = protoShort(s.p); o["p"] = isnan(s.bar) ? String("--") : String(pU(s.bar), pDec());
    o["t"] = isnan(s.temp) ? 0 : (int)s.temp; o["r"] = s.rssi; o["age"] = (now - s.last) / 1000; o["tire"] = s.tire;
  }
  JsonArray ca = d["cfg"].to<JsonArray>();
  for (int i = 0; i < 5; i++) {
    JsonObject o = ca.add<JsonObject>();
    o["id"] = tcfg[i].id; o["mn"] = String(tcfg[i].pMin, 2); o["mx"] = String(tcfg[i].pMax, 2); o["tx"] = (int)tcfg[i].tMax;
  }
  xSemaphoreGive(dataMux);
  String out; serializeJson(d, out);
  server.send(200, "application/json", out);
}

void cmdHandler() {
  if (server.hasArg("bw"))   sendCfg("bw", server.arg("bw").toFloat());
  if (server.hasArg("foff")) sendCfg("foff", server.arg("foff").toInt());
  if (server.hasArg("d"))    sendCfg("d", server.arg("d").toInt());
  if (server.hasArg("restore")) applyAction(520);
  if (server.hasArg("unit")) applyAction(513);
  if (server.hasArg("assign") && server.hasArg("id")) {
    int t = server.arg("assign").toInt();
    String id = server.arg("id");
    if (t >= 0 && t < 5 && id.length() && id.length() < 12) {
      xSemaphoreTake(dataMux, portMAX_DELAY);
      for (int i = 0; i < 5; i++) if (i != t && !strcmp(tcfg[i].id, id.c_str())) tcfg[i].id[0] = 0;
      strlcpy(tcfg[t].id, id.c_str(), sizeof(tcfg[t].id));
      memset(&ts[t], 0, sizeof(TireState));
      for (int i = 0; i < scanN; i++) scan[i].tire = findTire(scan[i].id);
      xSemaphoreGive(dataMux);
      savePrefs();
    }
  }
  server.send(200, "text/plain", "OK");
  if (server.hasArg("wifi")) { wifiOn = server.arg("wifi") == "1"; savePrefs(); delay(200); extern void wifiApply(); wifiApply(); }
  uiDirty = true;
}

void wifiApply() {
  if (wifiOn && !wifiRunning) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP("TPMS-V3", "12345678");
    WiFi.setTxPower(WIFI_POWER_11dBm);
    dns.start(53, "*", WiFi.softAPIP());
    server.begin();
    wifiRunning = true;
    Serial.println("[WIFI] AP TPMS-V3 @ 192.168.4.1");
  } else if (!wifiOn && wifiRunning) {
    dns.stop(); server.stop();
    WiFi.softAPdisconnect(true); WiFi.mode(WIFI_OFF);
    wifiRunning = false;
    Serial.println("[WIFI] off");
  }
  uiDirty = true;
}

// =============================================================================
//  الإعداد والحلقة
// =============================================================================
void handleUsb() {
  static char line[64]; static size_t len = 0;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch != '\n' && ch != '\r') { if (len < sizeof(line) - 1) line[len++] = ch; continue; }
    if (!len) continue;
    line[len] = 0; len = 0;
    if (!strcmp(line, "shot")) shotRequest = true;
    else if (!strncmp(line, "tap ", 4)) { int x = 0, y = 0; sscanf(line + 4, "%d %d", &x, &y); tapY = y; tapX = x; Serial.printf("[CMD] tap %d %d\n", x, y); }
    else if (!strncmp(line, "page ", 5)) { int pg = atoi(line + 5); if (pg >= 0 && pg < PG_COUNT) { tapY = NAV_Y + 10; tapX = pg * (SCR_W / PG_COUNT) + 10; } Serial.printf("[CMD] page %d\n", page); }
    else if (!strcmp(line, "stat")) Serial.printf("[STAT] up %lus page %d | link %s good %lu bad %lu | rx pk %lu cc %d %.3f MHz bw %d | heap %u KB (min %u) | wifi %d clients %d\n",
        millis() / 1000, page, millis() - linkLastMs < 4000 ? "OK" : "LOST", (unsigned long)linkGood, (unsigned long)linkBad,
        (unsigned long)radio.pk, radio.cc, radio.f, radio.bw, ESP.getFreeHeap() / 1024, ESP.getMinFreeHeap() / 1024, wifiRunning, wifiRunning ? WiFi.softAPgetStationNum() : 0);
    else if (!strcmp(line, "tires")) for (int i = 0; i < 5; i++) Serial.printf("[TIRE] %s %s has %d bar %.2f alarms %u\n", TIRE_LBL[i], tcfg[i].id, ts[i].has, ts[i].bar, ts[i].alarms);
    else if (!strcmp(line, "reboot")) ESP.restart();
    else Serial.printf("[CMD] unknown: %s (shot | tap X Y | page N | stat | tires | reboot)\n", line);
  }
}

void setup() {
  Serial.begin(115200);
  delay(50);
  Serial.printf("\n[SYSTEM] TPMS %s\n", FW_VERSION);
  esp_task_wdt_config_t wdt = {};
  wdt.timeout_ms = 20000; wdt.trigger_panic = true;
  if (esp_task_wdt_init(&wdt) == ESP_ERR_INVALID_STATE) esp_task_wdt_reconfigure(&wdt);

  pinMode(PIN_LED_R, OUTPUT); pinMode(PIN_LED_G, OUTPUT); pinMode(PIN_LED_B, OUTPUT);
  setLed(false, false, true);
  ledcAttach(PIN_SPK, 2000, 8);
  ledcWriteTone(PIN_SPK, 0);
  analogSetPinAttenuation(PIN_LDR, ADC_0db);

  dataMux = xSemaphoreCreateMutex();
  linkTxMux = xSemaphoreCreateMutex();
  loadPrefs();

  lcd = new LGFX_CYD(panelInvert, panelBgr);
  lcd->init();
  lcd->setRotation(1);
  lcd->setBrightness(brightness);
  if (touchCalValid) lcd->setTouchCalibrate(touchCal);
  band.setColorDepth(16);
  if (!band.createSprite(SCR_W, BAND_H)) Serial.println("[ERR] sprite alloc failed");

  Link.setRxBufferSize(4096);
  Link.begin(LINK_BAUD, SERIAL_8N1, LINK_RX, LINK_TX);

  server.on("/", []() { server.send_P(200, "text/html", PAGE); });
  server.on("/api", apiHandler);
  server.on("/cmd", cmdHandler);
  auto portal = []() { server.sendHeader("Location", "http://192.168.4.1/", true); server.send(302, "text/plain", ""); };
  server.on("/generate_204", portal); server.on("/hotspot-detect.html", portal); server.on("/connecttest.txt", portal);
  server.onNotFound(portal);
  wifiApply();

  xTaskCreatePinnedToCore(linkTask, "link", 6144, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(uiTask, "ui", 10240, NULL, 2, NULL, 1);
  sendCmd("dec");
}

void loop() {
  if (wifiRunning) { dns.processNextRequest(); server.handleClient(); }
  handleUsb();
  vTaskDelay(pdMS_TO_TICKS(5));
}
