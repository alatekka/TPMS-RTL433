#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <HTTPUpdateServer.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <TPMS_RTL433.h>
#include "esp_heap_caps.h"
#include "esp_system.h"

#define FW_VERSION "3.0"

// =============================================================================
// خريطة المنافذ والتعيينات العتادية (ESP32-S3 N16R8)
// =============================================================================
#define PIN_CC_CS      10
#define PIN_CC_SCK     12
#define PIN_CC_MOSI    11
#define PIN_CC_MISO    13
#define PIN_CC_GDO0     4
#define PIN_CC_GDO2     5

#define TFT_SCLK       18
#define TFT_MOSI       17
#define TFT_RST        16
#define TFT_DC         21
#define TFT_CS         14
#define TFT_BLK        15

// منفذ GPIO 1 آمن تماماً للزر كمدخل مع مقاومة سحب داخلية
#define PIN_PHYSICAL_BTN 1

#define SCREEN_W       320
#define SCREEN_H       240

const char* AP_SSID = "TPMS-ESP32S3-PRO";
const char* AP_PASS = "12345678";

WebServer server(80);
HTTPUpdateServer httpUpdater;
const byte DNS_PORT = 53;
DNSServer dnsServer;

// =============================================================================
// لوحة الألوان الرقمية (RGB565)
// =============================================================================
#define C_BG            0x0000
#define C_PANEL_BG      0x0862
#define C_PANEL_BORDER  0x1905
#define C_ACCENT        0x04FF
#define C_WHITE         0xFFFF
#define C_TEXT_MUTED    0x8410
#define C_OK            0x07E0
#define C_WARN          0xFDA0
#define C_DANGER        0xF800
#define C_CAR_SHADOW    0x0841
#define C_CAR_BODY      0x3186
#define C_CAR_GLASS     0x122B
#define C_CAR_CHROME    0xCE79
#define C_RIM_SILVER    0xBDD7
#define C_TIRE_TREAD    0x1082

// =============================================================================
// ثوابت السلوك
// =============================================================================
#define NUM_TIRES          5
#define TIRE_SPARE         4
#define MAX_SCANNED        64
#define LOG_LINES          40
#define LOG_LEN            400
#define RF_QUEUE_LEN       16

#define STALE_MS           (15UL * 60UL * 1000UL) // بدون إشارة لأكثر من 15 دقيقة
#define LEAK_WINDOW_MS     (10UL * 60UL * 1000UL) // نافذة كشف التسريب
#define LEAK_SAMPLE_MS     20000UL
#define LEAK_DROP_BAR      0.20f                  // هبوط 0.2 بار خلال النافذة = تسريب
#define LEAK_HIST          32
#define TREND_N            90                     // 90 عينة × دقيقة = ساعة ونصف
#define TREND_SAMPLE_MS    60000UL

#define BASE_FREQ_MHZ      433.92f
#define NVS_COMMIT_DELAY_MS 2000

// أوضاع الشاشة
enum : uint8_t { MODE_CAR = 0, MODE_SCAN = 1, MODE_RADIO = 2, MODE_TECH = 3, MODE_COUNT = 4 };

// وضع فني الإطارات: التقاط الحساس الأقرب فقط
#define TECH_HIST          10
#define TECH_STICKY_MS     10000UL  // الالتقاط الحالي ثابت 10 ثوانٍ...
#define TECH_STICKY_DB     3        // ...إلا إذا جاء حساس آخر أقوى بـ 3 dB
#define TECH_FLASH_MS      1500UL

// أعلام الإنذار
enum : uint8_t {
  AL_LOW  = 1 << 0, // ضغط منخفض
  AL_HIGH = 1 << 1, // ضغط مرتفع
  AL_HOT  = 1 << 2, // حرارة مرتفعة
  AL_BATT = 1 << 3, // بطارية الحساس ضعيفة
  AL_LEAK = 1 << 4  // هبوط سريع في الضغط
};
#define AL_DANGER_MASK (AL_LOW | AL_HOT | AL_LEAK)

// عروض النطاق المتاحة فعلياً في CC1101 (كريستال 26MHz)
static const float CC1101_BW_LIST[] = {58, 68, 81, 102, 116, 135, 162, 203, 232, 270, 325, 406, 464, 541, 650, 812};
static const int CC1101_BW_COUNT = sizeof(CC1101_BW_LIST) / sizeof(CC1101_BW_LIST[0]);

// =============================================================================
// محرك الرسم المخصص في ذاكرة الـ Octal PSRAM (8MB)
// =============================================================================
class PsramCanvas16 : public Adafruit_GFX {
public:
  uint16_t* buffer;
  PsramCanvas16(uint16_t w, uint16_t h) : Adafruit_GFX(w, h), buffer(nullptr) {
    buffer = (uint16_t*)heap_caps_malloc(w * h * sizeof(uint16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  }
  ~PsramCanvas16() {
    if (buffer) heap_caps_free(buffer);
  }
  void drawPixel(int16_t x, int16_t y, uint16_t color) override {
    if (x < 0 || x >= _width || y < 0 || y >= _height || !buffer) return;
    buffer[x + y * _width] = color;
  }
  void fillScreen(uint16_t color) override {
    if (!buffer) return;
    uint32_t total = _width * _height;
    for (uint32_t i = 0; i < total; i++) buffer[i] = color;
  }
  void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) override {
    if (y < 0 || y >= _height || !buffer) return;
    if (x < 0) { w += x; x = 0; }
    if (x + w > _width) w = _width - x;
    if (w <= 0) return;
    uint16_t *row = &buffer[y * _width + x];
    for (int16_t i = 0; i < w; i++) row[i] = color;
  }
  void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color) override {
    if (x < 0 || x >= _width || !buffer) return;
    if (y < 0) { h += y; y = 0; }
    if (y + h > _height) h = _height - y;
    if (h <= 0) return;
    uint16_t *col = &buffer[y * _width + x];
    for (int16_t i = 0; i < h; i++) {
      *col = color;
      col += _width;
    }
  }
  void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) override {
    for (int16_t j = 0; j < h; j++) drawFastHLine(x, y + j, w, color);
  }
  uint16_t* getBuffer() const { return buffer; }
};

SPIClass tftSpi(HSPI);
Adafruit_ST7789 tft = Adafruit_ST7789(&tftSpi, TFT_CS, TFT_DC, TFT_RST);
PsramCanvas16* canvas = nullptr;

// =============================================================================
// النماذج وهياكل البيانات
// =============================================================================
struct TireConfig {
  char  id[12];     // المعرف بعد التوحيد (HEX كبير، 8 خانات على الأقل) أو فارغ
  float pMin;       // بار
  float pMax;       // بار
  float tMax;       // درجة مئوية
};

struct TireState {
  bool     hasData;
  float    bar;      // NAN إن لم يرسل الحساس الضغط
  float    temp;     // NAN إن لم يرسل الحساس الحرارة
  int8_t   battery;  // -1 غير معروف، 0 ضعيفة، 1 سليمة
  int      rssi;
  int      status;   // حقل الحالة الخام إن وجد (-1 غير موجود)
  int8_t   proto;
  uint8_t  alarms;
  uint32_t lastRx;
  uint32_t count;
  // تاريخ الضغط لكشف التسريب السريع
  float    histBar[LEAK_HIST];
  uint32_t histT[LEAK_HIST];
  uint8_t  histHead;
  uint8_t  histN;
  // منحنى الضغط: عينة كل دقيقة لآخر 90 دقيقة (للرسم في صفحة الويب)
  float    trend[TREND_N];
  uint32_t trendLastT;
  uint8_t  trendHead;
  uint8_t  trendN;
};

static const char* const TIRE_LABEL[NUM_TIRES] = {
  "FRONT LEFT", "FRONT RIGHT", "REAR RIGHT", "REAR LEFT", "SPARE"
};
static const char* const TIRE_SHORT[NUM_TIRES] = {"FL", "FR", "RR", "RL", "SP"};

TireConfig tireCfg[NUM_TIRES] = {
  {"D78D8483", 1.80f, 3.00f, 75.0f},
  {"D78D8C0A", 1.80f, 3.00f, 75.0f},
  {"D78D8C17", 1.80f, 3.00f, 75.0f},
  {"D78D8CD9", 1.80f, 3.00f, 75.0f},
  {"D78D8409", 1.80f, 4.50f, 75.0f}
};
TireState tires[NUM_TIRES];

struct RawRfPacket {
  char text[512];
};

struct ScannedSensor {
  char     id[12];
  int8_t   proto;
  int8_t   tire;     // رقم الإطار إن كان من إطاراتي، وإلا -1
  int8_t   battery;
  float    bar;
  float    temp;
  int      rssi;
  int      bestRssi;
  uint32_t firstSeen;
  uint32_t lastSeen;
  uint32_t count;
};

ScannedSensor scannedList[MAX_SCANNED];
int scannedCount = 0;

// وضع فني الإطارات
struct TechCapture {
  bool     valid;
  char     id[12];
  int8_t   proto;
  int8_t   tire;      // إن كان من إطارات سيارتي
  int8_t   battery;
  float    bar;
  float    temp;
  int      rssi;
  int      bestRssi;
  int      status;
  char     mic[10];   // نوع التحقق: CRC / CHECKSUM
  uint32_t firstSeen;
  uint32_t lastSeen;
  uint32_t count;
};
TechCapture techCap;
TechCapture techHist[TECH_HIST];
int techHistCount = 0;
int techThreshold = -70;               // عتبة القرب (dBm)
volatile uint32_t techFlashUntil = 0;

// سجل الحزم الخام (في PSRAM)
char (*logBuf)[LOG_LEN] = nullptr;
int logHead = 0;
int logCount = 0;

// آخر حزمة ملتقطة (لشاشة الراديو)
char lastPktId[12] = "";
int8_t lastPktProto = -1;
int lastPktRssi = 0;
uint32_t lastPktMillis = 0;

rtl_433_ESP rf;
char messageBuffer[512];
QueueHandle_t rfQueue = NULL;
SemaphoreHandle_t dataMutex = NULL;
Preferences prefs;

TaskHandle_t rfLoopTaskHandle = NULL;
TaskHandle_t tftTaskHandle = NULL;
TaskHandle_t pktTaskHandle = NULL;

volatile uint32_t rfPacketCounter = 0;
volatile uint32_t rfQueueDrops = 0;
volatile uint32_t rfRejected = 0;   // حزم رُفضت بفلتر المنطقية
volatile bool serialQuiet = false;  // يوقف طباعة المهام الأخرى أثناء تصدير صورة الشاشة
volatile bool ccOk = false;
volatile int ccErr = 0;
volatile uint32_t ccRecoveries = 0;

// إعدادات الراديو (تُحفظ في NVS)
int   rssiThresholdDelta  = 9;       // أقل = مدى أبعد (مع ضجيج أكثر)
float fskBandwidthSetting = 102.0f;  // أفضل مدى في تجربة ميدانية (حتى -98 dBm)
int   freqOffsetKhz       = 0;       // معايرة انحراف كريستال الـ CC1101
uint32_t userDecoderMask  = TPMS_ALL_MASK;

volatile uint32_t activeDecoderMask = TOYOTA_ONLY_MASK;
volatile uint8_t displayMode = MODE_CAR;
volatile bool screenNeedsFullRedraw = true;
volatile bool wifiEnabled = true;
volatile int8_t wifiRequest = -1;    // -1 لا شيء، 0 إيقاف، 1 تشغيل (تُنفّذ في loop)
volatile uint32_t wifiRequestAt = 0;
volatile bool rebootRequested = false;
volatile uint32_t rebootAt = 0;

static volatile bool nvsDirty = false;
static volatile uint32_t lastNvsChangeTime = 0;

void markDirty() {
  lastNvsChangeTime = millis();
  nvsDirty = true;
}

float currentFrequencyMhz() {
  return BASE_FREQ_MHZ + freqOffsetKhz / 1000.0f;
}

// =============================================================================
// أدوات عامة
// =============================================================================
// يحوّل أي معرف (نص أو رقم) إلى HEX كبير، ويكمّل بالأصفار إلى 8 خانات
void normalizeId(const char* rawId, char* outBuf, size_t maxLen) {
  if (!outBuf || maxLen == 0) return;
  outBuf[0] = '\0';
  if (!rawId) return;
  char clean[16] = {0};
  int idx = 0;
  const char* p = rawId;
  if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) p += 2;
  for (; *p && idx < (int)sizeof(clean) - 1; ++p) {
    char c = *p;
    if (isxdigit((unsigned char)c)) clean[idx++] = (char)toupper((unsigned char)c);
  }
  clean[idx] = '\0';
  size_t len = strlen(clean);
  if (len == 0) return;
  size_t pad = (len < 8) ? 8 - len : 0;
  if (pad + len >= maxLen) {
    strlcpy(outBuf, clean, maxLen);
    return;
  }
  memset(outBuf, '0', pad);
  strlcpy(outBuf + pad, clean, maxLen - pad);
}

int findTireById(const char* normId) {
  if (!normId || !normId[0]) return -1;
  for (int i = 0; i < NUM_TIRES; i++) {
    if (tireCfg[i].id[0] && strcmp(normId, tireCfg[i].id) == 0) return i;
  }
  return -1;
}

int protocolIndex(const char* protocol) {
  if (!protocol || !protocol[0]) return -1;
  int n = rtl_433_ESP::decoderCount();
  for (int i = 0; i < n && i < TPMS_NUM_DECODERS; i++) {
    if (strcmp(protocol, rtl_433_ESP::decoderName(i)) == 0) return i;
  }
  return -1;
}

const char* protoShortName(int idx) {
  if (idx < 0 || idx >= TPMS_NUM_DECODERS) return "TPMS";
  return tpmsShortName(idx);
}

void formatAge(uint32_t ms, char* out, size_t len) {
  uint32_t s = ms / 1000;
  if (s < 60)        snprintf(out, len, "%lus", (unsigned long)s);
  else if (s < 3600) snprintf(out, len, "%lum", (unsigned long)(s / 60));
  else               snprintf(out, len, "%luh", (unsigned long)(s / 3600));
}

int bandwidthIndex(float bw) {
  int best = 0;
  float bestDiff = 1e9f;
  for (int i = 0; i < CC1101_BW_COUNT; i++) {
    float d = fabsf(CC1101_BW_LIST[i] - bw);
    if (d < bestDiff) { bestDiff = d; best = i; }
  }
  return best;
}

// =============================================================================
// تبديل الأوضاع والواي فاي (آمن من أي مهمة)
// =============================================================================
// السيارة: تويوتا فقط. الفني: كل البروتوكولات دائماً (أي إطار يُفحص). غيرهما: اختيار المستخدم
uint32_t maskForMode(uint8_t mode) {
  if (mode == MODE_CAR)  return TOYOTA_ONLY_MASK;
  if (mode == MODE_TECH) return TPMS_ALL_MASK;
  return userDecoderMask;
}

void applyMode(uint8_t mode) {
  if (mode >= MODE_COUNT) mode = MODE_CAR;
  displayMode = mode;
  activeDecoderMask = maskForMode(mode);
  rf.setDecoderMask(activeDecoderMask);
  screenNeedsFullRedraw = true;
  markDirty();
}

void applyRssiDelta(int delta) {
  rssiThresholdDelta = constrain(delta, 3, 30);
  rf.setRSSIThreshold(rssiThresholdDelta);
  markDirty();
}

void applyBandwidth(float khz) {
  fskBandwidthSetting = CC1101_BW_LIST[bandwidthIndex(khz)];
  rtl_433_ESP::setFskBandwidth(fskBandwidthSetting);
  markDirty();
}

void applyFreqOffset(int khz) {
  freqOffsetKhz = constrain(khz, -150, 150);
  rtl_433_ESP::setFrequency(currentFrequencyMhz());
  markDirty();
}

void requestWifi(bool enable, uint32_t delayMs) {
  wifiRequestAt = millis() + delayMs;
  wifiRequest = enable ? 1 : 0;
}

// يُنفَّذ فقط من loop() — نفس المهمة التي تخدم الويب، فلا تزامن مع handleClient
void processWifiRequest() {
  int8_t req = wifiRequest;
  if (req < 0 || (int32_t)(millis() - wifiRequestAt) < 0) return;
  wifiRequest = -1;
  bool enable = (req == 1);
  if (enable == wifiEnabled) return;

  if (enable) {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    WiFi.setTxPower(WIFI_POWER_11dBm);
    dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
    server.begin(); // المسارات مسجلة مرة واحدة في setup
  } else {
    dnsServer.stop();
    server.stop();
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_OFF);
  }
  wifiEnabled = enable;
  screenNeedsFullRedraw = true;
  Serial.printf("[WIFI] %s\n", enable ? "ON" : "OFF");
}

// =============================================================================
// الإعدادات الدائمة (NVS)
// =============================================================================
void loadSettings() {
  prefs.begin("tpms_cfg", false);
  uint8_t ver = prefs.getUChar("cfg_ver", 1);

  rssiThresholdDelta  = prefs.getInt("rssi_delta", 9);
  fskBandwidthSetting = prefs.getFloat("fsk_bw", 102.0f);
  freqOffsetKhz       = prefs.getInt("foff", 0);
  userDecoderMask     = prefs.getUInt("dec_mask32", TPMS_ALL_MASK) & TPMS_ALL_MASK;
  uint8_t mode        = prefs.getUChar("mode", MODE_CAR);
  techThreshold       = constrain(prefs.getInt("tech_thr", -70), -100, -20);

  for (int i = 0; i < NUM_TIRES; i++) {
    char key[16];
    snprintf(key, sizeof(key), "tire_id_%d", i);
    String savedId = prefs.getString(key, tireCfg[i].id);
    normalizeId(savedId.c_str(), tireCfg[i].id, sizeof(tireCfg[i].id));

    snprintf(key, sizeof(key), "pmin_%d", i);
    tireCfg[i].pMin = prefs.getFloat(key, tireCfg[i].pMin);
    snprintf(key, sizeof(key), "pmax_%d", i);
    tireCfg[i].pMax = prefs.getFloat(key, tireCfg[i].pMax);
    snprintf(key, sizeof(key), "tmax_%d", i);
    tireCfg[i].tMax = prefs.getFloat(key, tireCfg[i].tMax);
  }
  prefs.end();

  if (ver < 3) {
    // المكتبة الموحّدة TPMS_RTL433 غيّرت ترتيب المفككات: إعادة القناع إلى الكل مرة واحدة
    userDecoderMask = TPMS_ALL_MASK;
    markDirty();
  }
  if (ver < 2) {
    // ترقية من الإصدار القديم: عتبة 18dB كانت تحجب الإشارات البعيدة،
    // وقناع البروتوكولات القديم (8 بت) لا يطابق الترتيب الجديد.
    rssiThresholdDelta = 9;
    userDecoderMask = TPMS_ALL_MASK;
    markDirty();
  }

  rssiThresholdDelta = constrain(rssiThresholdDelta, 3, 30);
  fskBandwidthSetting = CC1101_BW_LIST[bandwidthIndex(fskBandwidthSetting)];
  freqOffsetKhz = constrain(freqOffsetKhz, -150, 150);
  if (mode >= MODE_COUNT) mode = MODE_CAR;
  displayMode = mode;
  activeDecoderMask = maskForMode(mode);
}

void commitSettingsIfDue() {
  if (!nvsDirty || (millis() - lastNvsChangeTime < NVS_COMMIT_DELAY_MS)) return;
  nvsDirty = false;

  TireConfig snapshot[NUM_TIRES];
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) != pdTRUE) {
    nvsDirty = true;
    return;
  }
  memcpy(snapshot, tireCfg, sizeof(snapshot));
  xSemaphoreGive(dataMutex);

  prefs.begin("tpms_cfg", false);
  prefs.putUChar("cfg_ver", 3);
  prefs.putInt("rssi_delta", rssiThresholdDelta);
  prefs.putFloat("fsk_bw", fskBandwidthSetting);
  prefs.putInt("foff", freqOffsetKhz);
  prefs.putUInt("dec_mask32", userDecoderMask);
  prefs.putUChar("mode", displayMode);
  prefs.putInt("tech_thr", techThreshold);

  for (int i = 0; i < NUM_TIRES; i++) {
    char key[16];
    snprintf(key, sizeof(key), "tire_id_%d", i);
    prefs.putString(key, snapshot[i].id);
    snprintf(key, sizeof(key), "pmin_%d", i);
    prefs.putFloat(key, snapshot[i].pMin);
    snprintf(key, sizeof(key), "pmax_%d", i);
    prefs.putFloat(key, snapshot[i].pMax);
    snprintf(key, sizeof(key), "tmax_%d", i);
    prefs.putFloat(key, snapshot[i].tMax);
  }
  prefs.end();
  Serial.println("[NVS] Settings saved");
}

// =============================================================================
// منطق الإنذارات وكشف التسريب
// =============================================================================
// يسجل عينة (كل 20 ثانية على الأكثر) ويعيد true إذا هبط الضغط 0.2 بار خلال 10 دقائق
bool updateLeakDetector(TireState& t, float bar, uint32_t now) {
  if (isnan(bar)) return false;
  uint8_t lastIdx = (t.histHead + LEAK_HIST - 1) % LEAK_HIST;
  if (t.histN == 0 || now - t.histT[lastIdx] >= LEAK_SAMPLE_MS) {
    t.histBar[t.histHead] = bar;
    t.histT[t.histHead] = now;
    t.histHead = (t.histHead + 1) % LEAK_HIST;
    if (t.histN < LEAK_HIST) t.histN++;
  }
  float maxRecent = bar;
  for (uint8_t i = 0; i < t.histN; i++) {
    uint8_t idx = (t.histHead + LEAK_HIST - 1 - i) % LEAK_HIST;
    if (now - t.histT[idx] > LEAK_WINDOW_MS) break;
    if (t.histBar[idx] > maxRecent) maxRecent = t.histBar[idx];
  }
  return (maxRecent - bar) >= LEAK_DROP_BAR;
}

uint8_t computeAlarms(int idx, float bar, float temp, int8_t battery, bool leak, uint8_t prev) {
  const TireConfig& c = tireCfg[idx];
  const float HYST_P = 0.05f;
  const float HYST_T = 2.0f;
  uint8_t a = 0;
  if (!isnan(bar)) {
    if ((prev & AL_LOW)  ? (bar < c.pMin + HYST_P) : (bar < c.pMin)) a |= AL_LOW;
    if ((prev & AL_HIGH) ? (bar > c.pMax - HYST_P) : (bar > c.pMax)) a |= AL_HIGH;
  }
  if (!isnan(temp)) {
    if ((prev & AL_HOT) ? (temp > c.tMax - HYST_T) : (temp > c.tMax)) a |= AL_HOT;
  }
  if (battery == 0) a |= AL_BATT;
  if (leak) a |= AL_LEAK;
  return a;
}

const char* alarmText(uint8_t a) {
  if (a & AL_LEAK) return "FAST LEAK !";
  if (a & AL_LOW)  return "LOW PRESSURE";
  if (a & AL_HOT)  return "TOO HOT !";
  if (a & AL_HIGH) return "HIGH PRESSURE";
  if (a & AL_BATT) return "BATTERY LOW";
  return "";
}

// =============================================================================
// الماسح وسجل الحزم
// =============================================================================
void addOrUpdateScanned(const char* id, int proto, float bar, float temp, int rssi,
                        int8_t battery, int tire, uint32_t now) {
  int target = -1;
  for (int i = 0; i < scannedCount; i++) {
    if (scannedList[i].proto == proto && strcmp(scannedList[i].id, id) == 0) {
      target = i;
      break;
    }
  }

  if (target < 0) {
    if (scannedCount < MAX_SCANNED) {
      target = scannedCount++;
    } else {
      // استبدال الأقدم فعلياً (LRU) وليس موضعاً عشوائياً
      target = 0;
      for (int i = 1; i < MAX_SCANNED; i++) {
        if ((int32_t)(scannedList[i].lastSeen - scannedList[target].lastSeen) < 0) target = i;
      }
    }
    ScannedSensor& s = scannedList[target];
    memset(&s, 0, sizeof(s));
    strlcpy(s.id, id, sizeof(s.id));
    s.proto = proto;
    s.firstSeen = now;
    s.bestRssi = rssi;
  }

  ScannedSensor& s = scannedList[target];
  s.tire = tire;
  s.battery = battery;
  s.bar = bar;
  s.temp = temp;
  s.rssi = rssi;
  if (rssi > s.bestRssi) s.bestRssi = rssi;
  s.lastSeen = now;
  s.count++;
}

// يملأ order بفهارس الحساسات مرتبة من الأحدث إلى الأقدم
int buildRecentOrder(uint8_t* order) {
  int n = scannedCount;
  for (int i = 0; i < n; i++) order[i] = i;
  for (int i = 1; i < n; i++) {
    uint8_t key = order[i];
    int j = i - 1;
    while (j >= 0 && (int32_t)(scannedList[order[j]].lastSeen - scannedList[key].lastSeen) < 0) {
      order[j + 1] = order[j];
      j--;
    }
    order[j + 1] = key;
  }
  return n;
}

void appendLog(const char* text, uint32_t now) {
  if (!logBuf) return;
  snprintf(logBuf[logHead], LOG_LEN, "[%lus] %s", (unsigned long)(now / 1000), text);
  logHead = (logHead + 1) % LOG_LINES;
  if (logCount < LOG_LINES) logCount++;
}

// =============================================================================
// استقبال ومعالجة الحزم
// =============================================================================
void logSignal(char *message) {
  if (!message || !rfQueue) return;
  RawRfPacket pkt;
  strlcpy(pkt.text, message, sizeof(pkt.text));
  if (xQueueSend(rfQueue, &pkt, 0) != pdTRUE) rfQueueDrops++;
}

// =============================================================================
// منطق وضع فني الإطارات
// =============================================================================
void pushTechHistory(const TechCapture& c) {
  if (!c.valid) return;
  // إزالة نسخة سابقة لنفس الحساس ثم الإدراج في الأعلى
  int w = 0;
  for (int i = 0; i < techHistCount; i++) {
    if (!(techHist[i].proto == c.proto && strcmp(techHist[i].id, c.id) == 0)) techHist[w++] = techHist[i];
  }
  techHistCount = w;
  if (techHistCount == TECH_HIST) techHistCount--;
  memmove(&techHist[1], &techHist[0], techHistCount * sizeof(TechCapture));
  techHist[0] = c;
  techHistCount++;
}

// يُستدعى تحت dataMutex لكل حزمة تتجاوز عتبة القرب
void techConsider(const char* id, int proto, int tire, float bar, float temp, int8_t battery,
                  int rssi, int status, const char* mic, uint32_t now) {
  TechCapture& c = techCap;
  bool same = c.valid && c.proto == proto && strcmp(c.id, id) == 0;
  if (!same && c.valid) {
    // الحساس الملتقط ثابت ما لم يمر 10 ثوانٍ أو تأتِ إشارة أقوى بوضوح (إطاران متجاوران)
    bool sticky = (now - c.lastSeen < TECH_STICKY_MS) && (rssi < c.rssi + TECH_STICKY_DB);
    if (sticky) return;
    pushTechHistory(c);
  }
  if (!same) {
    memset(&c, 0, sizeof(c));
    c.valid = true;
    strlcpy(c.id, id, sizeof(c.id));
    c.proto = proto;
    c.firstSeen = now;
    c.bestRssi = rssi;
    techFlashUntil = now + TECH_FLASH_MS;
    screenNeedsFullRedraw = true;
  }
  c.tire = tire;
  c.bar = bar;
  c.temp = temp;
  c.battery = battery;
  c.rssi = rssi;
  if (rssi > c.bestRssi) c.bestRssi = rssi;
  c.status = status;
  strlcpy(c.mic, mic ? mic : "", sizeof(c.mic));
  c.lastSeen = now;
  c.count++;
}

void techReset() {
  pushTechHistory(techCap);
  memset(&techCap, 0, sizeof(techCap));
  screenNeedsFullRedraw = true;
}

// حكم مختصر على حالة الحساس (للشاشة والسجل)
const char* techVerdict(const TechCapture& c, uint16_t* color) {
  uint16_t col = C_OK;
  const char* txt = "SENSOR OK";
  if (c.battery == 0)                         { col = C_DANGER; txt = "BATTERY LOW"; }
  else if (isnan(c.bar))                      { col = C_WARN;   txt = "NO PRESSURE DATA"; }
  else if (c.bar < 0.3f)                      { col = C_WARN;   txt = "0 BAR: NOT MOUNTED?"; }
  else if (!isnan(c.temp) && c.temp > 90.0f)  { col = C_DANGER; txt = "OVERHEAT"; }
  if (color) *color = col;
  return txt;
}

float readNumber(JsonDocument& doc, const char* key) {
  JsonVariant v = doc[key];
  if (v.is<float>()) return v.as<float>();
  return NAN;
}

void packetProcessingTask(void* pvParameters) {
  RawRfPacket pkt;
  for (;;) {
    if (xQueueReceive(rfQueue, &pkt, portMAX_DELAY) != pdTRUE) continue;


    JsonDocument doc;
    if (deserializeJson(doc, pkt.text) != DeserializationError::Ok) continue;

    int proto = protocolIndex(doc["protocol"] | "");
    // حزمة فُكّت قبل لحظة من تغيير القناع
    if (proto >= 0 && !((activeDecoderMask >> proto) & 1UL)) continue;

    char rawId[24] = {0};
    JsonVariant vid = doc["id"];
    if (vid.is<const char*>()) {
      strlcpy(rawId, vid.as<const char*>(), sizeof(rawId));
    } else if (vid.is<unsigned long>() || vid.is<long>()) {
      snprintf(rawId, sizeof(rawId), "%lX", (unsigned long)vid.as<unsigned long>());
    }
    char normId[12];
    normalizeId(rawId, normId, sizeof(normId));
    if (!normId[0]) continue;

    // الضغط: كل مفكك يرسل بوحدة مختلفة (تويوتا C210 يرسل PSI وليس kPa)
    float kpa = readNumber(doc, "pressure_kPa");
    if (isnan(kpa)) {
      float psi = readNumber(doc, "pressure_PSI");
      if (!isnan(psi)) kpa = psi * 6.894757f;
    }
    if (isnan(kpa)) {
      float b = readNumber(doc, "pressure_bar");
      if (!isnan(b)) kpa = b * 100.0f;
    }
    float bar = isnan(kpa) ? NAN : kpa / 100.0f;

    float temp = readNumber(doc, "temperature_C");
    if (isnan(temp)) {
      float tf = readNumber(doc, "temperature_F");
      if (!isnan(tf)) temp = (tf - 32.0f) * 5.0f / 9.0f;
    }

    // فلتر منطقية: المفككات ضعيفة التحقق تُخرج أحياناً قيماً مستحيلة من إشارات غيرها
    if (proto >= 0 && tpmsIsWeakDecoder(proto)) {
      bool badP = !isnan(kpa) && (kpa < 30.0f || kpa > 700.0f);
      bool badT = !isnan(temp) && (temp < -30.0f || temp > 110.0f);
      if (badP || badT) {
        rfRejected++;
        if (!serialQuiet) Serial.printf("[RF-REJ] %s\n", pkt.text);
        continue;
      }
    }

    int8_t battery = -1;
    if (!doc["battery_ok"].isNull()) battery = doc["battery_ok"].as<int>() ? 1 : 0;

    int status = doc["status"] | -1;
    int rssi = doc["rssi"] | rtl_433_ESP::signalRssi;
    uint32_t now = millis();

    if (!serialQuiet) Serial.printf("[RF] %s\n", pkt.text);
    rfPacketCounter++;

    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(250)) == pdTRUE) {
      int tire = findTireById(normId);
      if (tire >= 0) {
        TireState& t = tires[tire];
        bool leak = updateLeakDetector(t, bar, now);
        if (!isnan(bar) && (t.trendN == 0 || now - t.trendLastT >= TREND_SAMPLE_MS)) {
          t.trend[t.trendHead] = bar;
          t.trendHead = (t.trendHead + 1) % TREND_N;
          if (t.trendN < TREND_N) t.trendN++;
          t.trendLastT = now;
        }
        t.alarms  = computeAlarms(tire, bar, temp, battery, leak, t.alarms);
        t.hasData = true;
        t.bar     = bar;
        t.temp    = temp;
        t.battery = battery;
        t.rssi    = rssi;
        t.status  = status;
        t.proto   = proto;
        t.lastRx  = now;
        t.count++;
      }
      addOrUpdateScanned(normId, proto, bar, temp, rssi, battery, tire, now);
      if (displayMode == MODE_TECH && rssi >= techThreshold) {
        techConsider(normId, proto, tire, bar, temp, battery, rssi, status, doc["mic"] | "", now);
      }
      appendLog(pkt.text, now);
      strlcpy(lastPktId, normId, sizeof(lastPktId));
      lastPktProto = proto;
      lastPktRssi = rssi;
      lastPktMillis = now;
      xSemaphoreGive(dataMutex);
    }
  }
}

// =============================================================================
// محرك الرسم للشاشة
// =============================================================================
void renderRealisticCarChassis(PsramCanvas16* c, int cx, int cy) {
  c->fillRoundRect(cx - 24, cy - 68, 48, 136, 16, C_CAR_SHADOW);
  c->fillRoundRect(cx - 20, cy - 64, 40, 128, 14, C_CAR_BODY);
  c->drawRoundRect(cx - 20, cy - 64, 40, 128, 14, C_CAR_CHROME);
  c->fillRoundRect(cx - 16, cy - 38, 32, 24, 6, C_CAR_GLASS);
  c->drawRoundRect(cx - 16, cy - 38, 32, 24, 6, C_CAR_CHROME);
  c->fillRect(cx - 14, cy - 14, 28, 34, C_CAR_BODY);
  c->drawFastVLine(cx - 14, cy - 14, 34, C_CAR_CHROME);
  c->drawFastVLine(cx + 13, cy - 14, 34, C_CAR_CHROME);
  c->fillRoundRect(cx - 15, cy + 20, 30, 20, 4, C_CAR_GLASS);
  c->drawRoundRect(cx - 15, cy + 20, 30, 20, 4, C_CAR_CHROME);
  c->drawFastHLine(cx - 12, cy - 54, 24, C_CAR_CHROME);
  c->drawFastHLine(cx - 8,  cy - 50, 16, C_CAR_CHROME);
  c->fillRoundRect(cx - 18, cy - 63, 8, 4, 2, C_ACCENT);
  c->fillRoundRect(cx + 10, cy - 63, 8, 4, 2, C_ACCENT);
  c->fillRoundRect(cx - 18, cy + 61, 8, 3, 1, C_DANGER);
  c->fillRoundRect(cx + 10, cy + 61, 8, 3, 1, C_DANGER);
  c->fillRoundRect(cx - 25, cy - 34, 5, 8, 2, C_CAR_BODY);
  c->fillRoundRect(cx + 20, cy - 34, 5, 8, 2, C_CAR_BODY);
}

void renderRealisticWheel(PsramCanvas16* c, int x, int y, uint16_t statusColor) {
  c->drawRoundRect(x - 2, y - 2, 12, 30, 4, statusColor);
  c->fillRoundRect(x, y, 8, 26, 3, C_TIRE_TREAD);
  c->fillRect(x + 2, y + 4, 4, 18, C_RIM_SILVER);
  c->drawFastHLine(x + 1, y + 9,  6, C_WHITE);
  c->drawFastHLine(x + 1, y + 13, 6, statusColor);
  c->drawFastHLine(x + 1, y + 17, 6, C_WHITE);
}

bool tireIsStale(const TireState& t, uint32_t now) {
  return t.hasData && (now - t.lastRx > STALE_MS);
}

uint16_t tireColor(const TireState& t, uint32_t now) {
  if (!t.hasData || tireIsStale(t, now)) return C_TEXT_MUTED;
  if (t.alarms & AL_DANGER_MASK) return C_DANGER;
  if (t.alarms) return C_WARN;
  return C_OK;
}

// مؤشر قوة الإشارة (4 أعمدة)
void renderSignalBars(PsramCanvas16* c, int x, int y, int rssi, uint16_t color) {
  int level = rssi >= -60 ? 4 : rssi >= -72 ? 3 : rssi >= -84 ? 2 : rssi >= -96 ? 1 : 0;
  for (int i = 0; i < 4; i++) {
    int h = 3 + i * 2;
    if (i < level) c->fillRect(x + i * 4, y + 9 - h, 3, h, color);
    else           c->drawRect(x + i * 4, y + 9 - h, 3, h, C_PANEL_BORDER);
  }
}

void renderTireCard(PsramCanvas16* c, int x, int y, int w, int h, int idx, uint32_t now, bool blinkOn) {
  const TireState& t = tires[idx];
  uint16_t stateColor = tireColor(t, now);
  bool danger = t.hasData && !tireIsStale(t, now) && (t.alarms & AL_DANGER_MASK);

  c->fillRoundRect(x, y, w, h, 8, danger && blinkOn ? 0x3000 : C_PANEL_BG);
  c->drawRoundRect(x, y, w, h, 8, stateColor);
  if (danger) c->drawRoundRect(x + 1, y + 1, w - 2, h - 2, 7, stateColor);

  c->setTextSize(1);
  c->setTextColor(C_TEXT_MUTED);
  c->setCursor(x + 6, y + 5);
  c->print(TIRE_LABEL[idx]);
  if (t.hasData) renderSignalBars(c, x + w - 20, y + 3, t.rssi, stateColor);

  c->setTextSize(2);
  c->setTextColor(stateColor);
  c->setCursor(x + 6, y + 17);
  if (t.hasData && !isnan(t.bar)) c->printf("%.2f", t.bar);
  else                            c->print("--.--");
  c->setTextSize(1);
  c->setTextColor(C_TEXT_MUTED);
  c->print(" BAR");

  c->setCursor(x + 6, y + 37);
  c->setTextColor(C_WHITE);
  if (t.hasData) {
    if (!isnan(t.bar)) c->printf("%4.1f PSI", t.bar * 14.5038f);
    else               c->print("--.- PSI");
    if (!isnan(t.temp)) c->printf("  %dC", (int)lroundf(t.temp));
    else                c->print("  --C");
  } else {
    c->print("--.- PSI  --C");
  }

  c->setCursor(x + 6, y + 50);
  char age[8];
  if (!tireCfg[idx].id[0]) {
    c->setTextColor(C_TEXT_MUTED);
    c->print("NOT ASSIGNED");
  } else if (!t.hasData) {
    c->setTextColor(C_TEXT_MUTED);
    c->print("WAITING...");
  } else if (tireIsStale(t, now)) {
    formatAge(now - t.lastRx, age, sizeof(age));
    c->setTextColor(C_TEXT_MUTED);
    c->printf("NO SIGNAL %s", age);
  } else if (t.alarms) {
    c->setTextColor(stateColor);
    c->print(alarmText(t.alarms));
  } else {
    formatAge(now - t.lastRx, age, sizeof(age));
    c->setTextColor(C_OK);
    c->printf("OK %ddB %s", t.rssi, age);
  }
}

void renderHeader(PsramCanvas16* c, uint32_t now, bool blinkOn) {
  // هل هناك إنذار خطر في أي إطار؟
  int alarmTire = -1;
  for (int i = 0; i < NUM_TIRES; i++) {
    const TireState& t = tires[i];
    if (t.hasData && !tireIsStale(t, now) && (t.alarms & AL_DANGER_MASK)) { alarmTire = i; break; }
  }

  bool showAlarm = alarmTire >= 0 && displayMode != MODE_CAR;
  c->fillRect(0, 0, SCREEN_W, 26, showAlarm && blinkOn ? 0x6000 : 0x0842);
  c->drawFastHLine(0, 26, SCREEN_W, displayMode == MODE_CAR ? C_ACCENT : C_WARN);

  c->setTextSize(1);
  c->setTextColor(C_WHITE);
  c->setCursor(8, 9);
  if (showAlarm) {
    c->setTextColor(C_DANGER);
    c->printf("! %s %s", TIRE_SHORT[alarmTire], alarmText(tires[alarmTire].alarms));
  } else {
    c->print("TPMS S3 PRO v" FW_VERSION);
  }

  c->setCursor(186, 9);
  if (displayMode == MODE_CAR) {
    c->setTextColor(C_OK);
    c->print("MODE: CAR (TOYOTA)");
  } else if (displayMode == MODE_SCAN) {
    c->setTextColor(C_WARN);
    c->print("MODE: SCANNER");
  } else if (displayMode == MODE_TECH) {
    c->setTextColor(0xF81F);
    c->print("MODE: TIRE TECH");
  } else {
    c->setTextColor(C_ACCENT);
    c->print("MODE: RADIO DIAG");
  }
}

void renderFooter(PsramCanvas16* c, uint32_t now) {
  c->drawFastHLine(0, 196, SCREEN_W, C_PANEL_BORDER);
  c->fillRect(0, 197, SCREEN_W, 43, 0x0821);
  c->setTextSize(1);

  c->setCursor(6, 201);
  if (displayMode == MODE_CAR) {
    const TireState& sp = tires[TIRE_SPARE];
    c->setTextColor(tireColor(sp, now));
    if (sp.hasData && !isnan(sp.bar)) {
      char age[8];
      formatAge(now - sp.lastRx, age, sizeof(age));
      c->printf("SPARE: %.2f BAR | ", sp.bar);
      if (!isnan(sp.temp)) c->printf("%dC | ", (int)lroundf(sp.temp));
      if (sp.alarms) c->print(alarmText(sp.alarms));
      else           c->printf("%ddBm | %s", sp.rssi, age);
    } else {
      c->print("SPARE (SP): STANDBY / NO BROADCAST");
    }
  } else if (displayMode == MODE_TECH) {
    c->setTextColor(0xF81F);
    c->printf("TECH: NEAR >= %d dBm | CHECKED: %d", techThreshold, techHistCount + (techCap.valid ? 1 : 0));
  } else {
    int enabled = __builtin_popcount(userDecoderMask);
    c->setTextColor(C_WARN);
    c->printf("SENSORS: %d | PROTOCOLS: %d/%d", scannedCount, enabled, TPMS_NUM_DECODERS);
  }

  c->setCursor(6, 214);
  c->setTextColor(ccOk ? C_OK : C_DANGER);
  c->printf("CC1101: %s | PKTS: %-5lu | WiFi: ", ccOk ? "READY" : "OFFLINE", (unsigned long)rfPacketCounter);
  if (wifiEnabled) {
    c->setTextColor(C_OK);
    c->print("ON");
  } else {
    c->setTextColor(C_TEXT_MUTED);
    c->print("OFF");
  }

  c->setCursor(6, 227);
  c->setTextColor(C_TEXT_MUTED);
  c->printf("%.3fMHz BW%d N%d T%d", rtl_433_ESP::currentFrequency, (int)fskBandwidthSetting,
            rtl_433_ESP::averageRssi, rtl_433_ESP::rssiThreshold);
}

void renderCarScreenToBuffer(PsramCanvas16* c, uint32_t now, bool blinkOn) {
  c->fillScreen(C_BG);
  renderHeader(c, now, blinkOn);

  int cx = 160;
  int cy = 112;
  renderRealisticCarChassis(c, cx, cy);

  renderRealisticWheel(c, cx - 29, cy - 48, tireColor(tires[0], now));
  renderRealisticWheel(c, cx + 21, cy - 48, tireColor(tires[1], now));
  renderRealisticWheel(c, cx - 29, cy + 24, tireColor(tires[3], now));
  renderRealisticWheel(c, cx + 21, cy + 24, tireColor(tires[2], now));

  c->drawFastHLine(124, cy - 35, 6, C_PANEL_BORDER);
  c->drawFastHLine(190, cy - 35, 6, C_PANEL_BORDER);
  c->drawFastHLine(124, cy + 37, 6, C_PANEL_BORDER);
  c->drawFastHLine(190, cy + 37, 6, C_PANEL_BORDER);

  renderTireCard(c, 6,   30,  118, 64, 0, now, blinkOn);
  renderTireCard(c, 196, 30,  118, 64, 1, now, blinkOn);
  renderTireCard(c, 6,   124, 118, 64, 3, now, blinkOn);
  renderTireCard(c, 196, 124, 118, 64, 2, now, blinkOn);

  renderFooter(c, now);
}

void renderScannerScreenToBuffer(PsramCanvas16* c, uint32_t now, bool blinkOn) {
  c->fillScreen(C_BG);
  renderHeader(c, now, blinkOn);

  c->fillRect(0, 27, SCREEN_W, 18, 0x10A2);
  c->setTextSize(1);
  c->setTextColor(C_ACCENT);
  c->setCursor(6,   32); c->print("ID");
  c->setCursor(60,  32); c->print("MODEL");
  c->setCursor(126, 32); c->print("BAR");
  c->setCursor(162, 32); c->print("TEMP");
  c->setCursor(196, 32); c->print("RSSI");
  c->setCursor(236, 32); c->print("AGE");
  c->setCursor(272, 32); c->print("CNT");
  c->drawFastHLine(0, 45, SCREEN_W, C_PANEL_BORDER);

  uint8_t order[MAX_SCANNED];
  int total = buildRecentOrder(order);
  int count = min(total, 5);

  if (count == 0) {
    c->setTextColor(C_TEXT_MUTED);
    c->setCursor(60, 110);
    c->print("LISTENING... NO SENSORS YET");
  }

  int y = 50;
  for (int i = 0; i < count; i++) {
    const ScannedSensor& s = scannedList[order[i]];
    uint16_t rowColor = (s.tire >= 0) ? C_OK : (tpmsIsToyota(s.proto) ? C_WARN : C_WHITE);
    c->fillRoundRect(4, y - 2, SCREEN_W - 8, 24, 4, (i % 2 == 0) ? 0x0841 : 0x0000);
    c->drawRoundRect(4, y - 2, SCREEN_W - 8, 24, 4, C_PANEL_BORDER);

    char age[8];
    formatAge(now - s.lastSeen, age, sizeof(age));

    c->setTextColor(rowColor);
    c->setCursor(6,   y + 6); c->print(s.id);
    c->setCursor(60,  y + 6); c->print(protoShortName(s.proto));
    c->setCursor(126, y + 6);
    if (!isnan(s.bar)) c->printf("%.2f", s.bar); else c->print("--");
    c->setCursor(162, y + 6);
    if (!isnan(s.temp)) c->printf("%dC", (int)lroundf(s.temp)); else c->print("--");
    c->setCursor(196, y + 6); c->printf("%d", s.rssi);
    c->setCursor(236, y + 6); c->print(age);
    c->setCursor(272, y + 6);
    if (s.tire >= 0) c->print(TIRE_SHORT[s.tire]);
    else             c->printf("%lu", (unsigned long)s.count);

    y += 28;
  }

  renderFooter(c, now);
}

// شاشة تشخيص الراديو: مقياس RSSI حي لتوجيه الهوائي وضبط الحساسية
void renderRadioScreenToBuffer(PsramCanvas16* c, uint32_t now, bool blinkOn, int peakRssi) {
  c->fillScreen(C_BG);
  renderHeader(c, now, blinkOn);
  rtl_433_ESPStatus st = rtl_433_ESP::statusSnapshot();

  c->setTextSize(1);
  c->setTextColor(C_TEXT_MUTED);
  c->setCursor(8, 34);
  c->print("LIVE RSSI (PEAK 250ms)");

  c->setTextSize(3);
  c->setTextColor(peakRssi > st.rssiThreshold ? C_OK : C_WHITE);
  c->setCursor(8, 46);
  c->printf("%4d", peakRssi);
  c->setTextSize(1);
  c->print(" dBm");

  // المقياس من -110 إلى -30 dBm
  const int mx = 8, my = 76, mw = SCREEN_W - 16, mh = 16;
  auto toX = [&](int dbm) -> int {
    int v = constrain(dbm, -110, -30);
    return mx + (int)((long)(v + 110) * mw / 80);
  };
  c->drawRect(mx, my, mw, mh, C_PANEL_BORDER);
  int px = toX(peakRssi);
  c->fillRect(mx + 1, my + 1, px - mx - 1, mh - 2, peakRssi > st.rssiThreshold ? C_OK : C_ACCENT);
  int nx = toX(st.averageRssi);
  int tx = toX(st.rssiThreshold);
  c->drawFastVLine(nx, my - 4, mh + 8, C_TEXT_MUTED);
  c->drawFastVLine(tx, my - 4, mh + 8, C_WARN);
  c->setTextColor(C_TEXT_MUTED);
  c->setCursor(mx, my + mh + 4);  c->print("-110");
  c->setCursor(mx + mw - 18, my + mh + 4); c->print("-30");
  c->setTextColor(C_WARN);
  c->setCursor(constrain(tx - 9, mx + 30, mx + mw - 50), my + mh + 4); c->print("THR");

  int y = 112;
  c->setTextColor(C_WHITE);
  c->setCursor(8, y);      c->printf("NOISE FLOOR: %d dBm   THRESHOLD: %d dBm", st.averageRssi, st.rssiThreshold);
  c->setCursor(8, y + 12); c->printf("DELTA: %d dB   BW: %d kHz   OFFSET: %+d kHz", rssiThresholdDelta, (int)fskBandwidthSetting, freqOffsetKhz);
  c->setCursor(8, y + 24); c->printf("FREQ: %.3f MHz   CC1101: %s", rtl_433_ESP::currentFrequency, ccOk ? "RX OK" : "OFFLINE");
  c->setTextColor(C_ACCENT);
  c->setCursor(8, y + 40); c->printf("CAPTURED: %u  DECODED: %u", st.capturedSignals, st.decodedMessages);
  c->setCursor(8, y + 52); c->printf("UNPARSED: %u  DROPPED: %u/%u/%lu", st.zeroDecodedSignals,
                                     st.droppedCaptureBuffers, st.droppedDecoderQueue, (unsigned long)rfQueueDrops);
  c->setCursor(8, y + 64);
  if (lastPktMillis) {
    char age[8];
    formatAge(now - lastPktMillis, age, sizeof(age));
    c->setTextColor(C_OK);
    c->printf("LAST: %s %s %ddBm %s ago", lastPktId, protoShortName(lastPktProto), lastPktRssi, age);
  } else {
    c->setTextColor(C_TEXT_MUTED);
    c->print("LAST: NO PACKET DECODED YET");
  }

  renderFooter(c, now);
}

// شاشة فني الإطارات: الحساس الأقرب بخط كبير مع حكم على حالته
void renderTechScreenToBuffer(PsramCanvas16* c, uint32_t now, bool blinkOn, int peakRssi) {
  c->fillScreen(C_BG);
  renderHeader(c, now, blinkOn);
  const TechCapture& t = techCap;

  if (!t.valid) {
    c->setTextSize(2);
    c->setTextColor(C_WHITE);
    c->setCursor(28, 38);  c->print("BRING THE DEVICE");
    c->setCursor(40, 58);  c->print("CLOSE TO THE TIRE");
    c->setTextSize(1);
    c->setTextColor(C_TEXT_MUTED);
    c->setCursor(8, 86);   c->print("Wake the sensor: deflate a little, rotate");
    c->setCursor(8, 98);   c->print("the wheel or use an LF activation tool.");
    c->setTextColor(0xF81F);
    c->setCursor(8, 116);  c->printf("CAPTURE WHEN SIGNAL >= %d dBm", techThreshold);

    // مقياس قرب حي
    const int mx = 8, my = 132, mw = SCREEN_W - 16, mh = 16;
    auto toX = [&](int dbm) -> int {
      int v = constrain(dbm, -110, -30);
      return mx + (int)((long)(v + 110) * mw / 80);
    };
    c->drawRect(mx, my, mw, mh, C_PANEL_BORDER);
    int px = toX(peakRssi);
    c->fillRect(mx + 1, my + 1, px - mx - 1, mh - 2, peakRssi >= techThreshold ? C_OK : C_ACCENT);
    c->drawFastVLine(toX(techThreshold), my - 4, mh + 8, 0xF81F);
    c->setTextColor(C_WHITE);
    c->setCursor(8, my + mh + 6);
    c->printf("LIVE %d dBm", peakRssi);

    if (techHistCount > 0) {
      c->setTextColor(C_TEXT_MUTED);
      c->setCursor(8, 184);
      c->printf("LAST: %s %s", techHist[0].id, protoShortName(techHist[0].proto));
      if (!isnan(techHist[0].bar)) c->printf(" %.2f BAR", techHist[0].bar);
    }
    renderFooter(c, now);
    return;
  }

  uint16_t vColor;
  const char* verdict = techVerdict(t, &vColor);
  bool flash = (int32_t)(techFlashUntil - now) > 0;
  if (flash) {
    c->drawRect(0, 27, SCREEN_W, 169, C_OK);
    c->drawRect(1, 28, SCREEN_W - 2, 167, C_OK);
  }

  // المعرف والبروتوكول
  c->setTextSize(1);
  c->setTextColor(C_TEXT_MUTED);
  c->setCursor(8, 32);
  c->print("SENSOR ID");
  c->setTextColor(C_ACCENT);
  c->setCursor(190, 32);
  c->print(protoShortName(t.proto));
  if (t.tire >= 0) {
    c->setTextColor(C_OK);
    c->printf("  MY %s", TIRE_SHORT[t.tire]);
  }
  c->setTextSize(3);
  c->setTextColor(C_WHITE);
  c->setCursor(8, 42);
  c->print(t.id);

  // الضغط بخط كبير
  c->setTextSize(4);
  c->setTextColor(vColor);
  c->setCursor(8, 72);
  if (!isnan(t.bar)) c->printf("%.2f", t.bar);
  else               c->print("--.--");
  c->setTextSize(2);
  c->setTextColor(C_TEXT_MUTED);
  c->print(" BAR");

  c->setTextColor(C_WHITE);
  c->setCursor(200, 72);
  if (!isnan(t.bar)) c->printf("%.1f PSI", t.bar * 14.5038f);
  else               c->print("-- PSI");
  c->setCursor(200, 92);
  if (!isnan(t.temp)) c->printf("%d C", (int)lroundf(t.temp));
  else                c->print("-- C");

  // التفاصيل
  char age[8];
  formatAge(now - t.lastSeen, age, sizeof(age));
  c->setTextSize(1);
  c->setTextColor(C_WHITE);
  c->setCursor(8, 112);
  c->printf("SIGNAL: %d dBm (BEST %d)   BATTERY: %s", t.rssi, t.bestRssi,
            t.battery == 1 ? "OK" : t.battery == 0 ? "LOW" : "N/A");
  c->setCursor(8, 124);
  c->printf("PACKETS: %lu   SEEN: %s ago   CHECK: %s", (unsigned long)t.count, age, t.mic[0] ? t.mic : "-");
  c->setCursor(8, 136);
  if (t.status >= 0) c->printf("STATUS: 0x%02X   ", t.status);
  c->setTextColor(C_TEXT_MUTED);
  c->print(rtl_433_ESP::decoderName(t.proto >= 0 ? t.proto : 0));

  // صندوق الحكم
  c->fillRoundRect(8, 148, SCREEN_W - 16, 26, 6, vColor == C_OK ? 0x0320 : (vColor == C_WARN ? 0x4A00 : 0x5000));
  c->drawRoundRect(8, 148, SCREEN_W - 16, 26, 6, vColor);
  c->setTextSize(2);
  c->setTextColor(vColor);
  int tw = strlen(verdict) * 12;
  c->setCursor((SCREEN_W - tw) / 2, 154);
  c->print(verdict);

  // آخر حساس مفحوص قبله
  if (techHistCount > 0) {
    c->setTextSize(1);
    c->setTextColor(C_TEXT_MUTED);
    c->setCursor(8, 182);
    c->printf("PREV: %s %s", techHist[0].id, protoShortName(techHist[0].proto));
    if (!isnan(techHist[0].bar)) c->printf(" %.2f BAR", techHist[0].bar);
  }

  renderFooter(c, now);
}

// مهمة الزر المستقلة: قراءة كل 5ms وعدّ القراءات المتتالية.
// GPIO1 مجاور لمنفذي TX/RX في معظم لوحات ESP32-S3، ومقاومة السحب الداخلية ضعيفة (~45k)،
// فحركة المنفذ التسلسلي تُحدث نبضات قصيرة على سلك الزر. لا تُحتسب الضغطة إلا إذا ثبت
// المنفذ على LOW لـ 16 قراءة متتالية (80ms) دون أي انقطاع.
void buttonTask(void* pvParameters) {
  pinMode(PIN_PHYSICAL_BTN, INPUT_PULLUP);
  const int PRESS_SAMPLES = 16;   // 80ms
  const int RELEASE_SAMPLES = 8;  // 40ms
  int lowCount = 0;
  int highCount = 0;
  bool pressed = false;
  bool longFired = false;
  uint32_t pressStart = 0;
  vTaskDelay(pdMS_TO_TICKS(3000)); // تجاهل فترة الإقلاع

  for (;;) {
    bool low = digitalRead(PIN_PHYSICAL_BTN) == LOW;
    uint32_t now = millis();
    if (low) {
      highCount = 0;
      if (lowCount < 1000) lowCount++;
      if (!pressed && lowCount >= PRESS_SAMPLES) {
        pressed = true;
        longFired = false;
        pressStart = now - PRESS_SAMPLES * 5;
      }
    } else {
      lowCount = 0;
      if (highCount < 1000) highCount++;
      if (pressed && highCount >= RELEASE_SAMPLES) {
        pressed = false;
        if (!longFired) {
          // ضغط قصير: السيارة -> الماسح -> تشخيص الراديو
          applyMode((displayMode + 1) % MODE_COUNT);
          if (!serialQuiet) Serial.printf("[BTN] short press (%lums) -> mode %u\n", (unsigned long)(now - pressStart), displayMode);
        }
      }
    }
    if (pressed && !longFired && now - pressStart >= 1500) {
      longFired = true;
      requestWifi(!wifiEnabled, 0); // ضغط مطول: تشغيل/إيقاف الواي فاي
      if (!serialQuiet) Serial.println("[BTN] long press -> toggle WiFi");
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// مهمة الشاشة (Core 1)
void tftUpdateTask(void* pvParameters) {
  uint32_t lastRenderTick = 0;
  int peakRssi = -128;
  uint32_t peakStart = 0;

  for (;;) {
    uint32_t now = millis();

    // قمة RSSI لشاشة الراديو
    int cur = rtl_433_ESP::currentRssi;
    if (cur > peakRssi) peakRssi = cur;

    uint32_t interval = (displayMode == MODE_RADIO || displayMode == MODE_TECH) ? 120 : 250;
    if (screenNeedsFullRedraw || (now - lastRenderTick >= interval)) {
      lastRenderTick = now;
      screenNeedsFullRedraw = false;
      bool blinkOn = (now / 500) % 2;
      int peakToShow = peakRssi;
      if (now - peakStart >= 250) {
        peakStart = now;
        peakRssi = -128;
      }

      if (canvas && canvas->getBuffer() && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        if (displayMode == MODE_CAR)       renderCarScreenToBuffer(canvas, now, blinkOn);
        else if (displayMode == MODE_SCAN) renderScannerScreenToBuffer(canvas, now, blinkOn);
        else if (displayMode == MODE_TECH) renderTechScreenToBuffer(canvas, now, blinkOn, peakToShow);
        else                               renderRadioScreenToBuffer(canvas, now, blinkOn, peakToShow);
        xSemaphoreGive(dataMutex);

        tft.drawRGBBitmap(0, 0, canvas->getBuffer(), SCREEN_W, SCREEN_H);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// مهمة الراديو والمراقبة الذاتية (Core 0)
void rfLoopTask(void* pvParameters) {
  pinMode(PIN_CC_CS, OUTPUT);
  digitalWrite(PIN_CC_CS, HIGH);
  vTaskDelay(pdMS_TO_TICKS(100));

  rf.setCallback(logSignal, messageBuffer, sizeof(messageBuffer));

  uint32_t lastRecoveryAttempt = 0;
  uint32_t lastHealthCheck = 0;
  bool firstStart = true;

  for (;;) {
    if (!ccOk) {
      if (firstStart || millis() - lastRecoveryAttempt >= 3000) {
        lastRecoveryAttempt = millis();
        if (!firstStart) {
          rf.end();
          ccRecoveries++;
        }
        firstStart = false;
        vTaskDelay(pdMS_TO_TICKS(50));

        // تُطبَّق قبل begin حتى يبدأ الراديو بالإعدادات المحفوظة مباشرة
        rf.setRSSIThreshold(rssiThresholdDelta);
        rtl_433_ESP::setFskBandwidth(fskBandwidthSetting);
        rf.setDecoderMask(activeDecoderMask);

        rtl_433_ESP::setPins(PIN_CC_SCK, PIN_CC_MISO, PIN_CC_MOSI, PIN_CC_CS, PIN_CC_GDO0, PIN_CC_GDO2);
        bool radioOk = rf.begin(PIN_CC_GDO0, currentFrequencyMhz());
        if (radioOk) {
          rtl_433_ESP::setNotifyTask(xTaskGetCurrentTaskHandle());
          rf.enableReceiver();
          ccOk = true;
          ccErr = 0;
          lastHealthCheck = millis();
          Serial.printf("[RF] CC1101 ready @ %.3f MHz, BW %d kHz, delta %d dB, %d decoders\n",
                        currentFrequencyMhz(), (int)fskBandwidthSetting, rssiThresholdDelta, rtl_433_ESP::decoderCount());
        } else {
          ccErr = (int)rf.lastError();
          Serial.printf("[RF] CC1101 init failed (err %d), retrying\n", ccErr);
        }
        screenNeedsFullRedraw = true;
      }
      vTaskDelay(pdMS_TO_TICKS(50));
    } else {
      ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
      rf.loop(); // يُستدعى دائماً حتى لو فات إشعار، فلا تتراكم الإشارات
      if (millis() - lastHealthCheck >= 5000) {
        lastHealthCheck = millis();
        if (!rf.isRadioAlive()) {
          ccOk = false;
          ccErr = -99;
          Serial.println("[RF] CC1101 health check failed, reinitialising");
        }
      }
    }
  }
}

// =============================================================================
// خادم الويب و Captive Portal
// =============================================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="ar" dir="rtl">
<head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>TPMS PRO S3</title>
<style>
:root{--bg:#070d1e;--card:#0f172a;--bd:#1e293b;--tx:#f8fafc;--mu:#94a3b8;--ac:#38bdf8;--wa:#f59e0b;--da:#ef4444;--ok:#10b981}
*{box-sizing:border-box}
body{margin:0;font-family:system-ui,-apple-system,sans-serif;background:var(--bg);color:var(--tx)}
header{background:#030712;padding:12px 16px;display:flex;justify-content:space-between;align-items:center;border-bottom:1px solid var(--bd);gap:8px;flex-wrap:wrap}
.tabs{display:flex;gap:6px;margin:12px;flex-wrap:wrap}
.tb{background:var(--card);color:var(--tx);border:1px solid var(--bd);padding:9px 14px;border-radius:8px;cursor:pointer;font-weight:500}
.tb.on{background:var(--ac);color:#070d1e;font-weight:700;border-color:var(--ac)}
.pg{padding:0 12px 20px;display:none}.pg.on{display:block}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:12px;margin-bottom:12px;direction:ltr}
.grid>*{direction:rtl}
.tc{background:var(--card);border:2px solid var(--bd);border-radius:12px;padding:14px;text-align:center}
.tc.wa{border-color:var(--wa);background:#271a0c}.tc.da{border-color:var(--da);background:#2c0b0e;animation:bl 1s infinite}.tc.st{opacity:.55}
@keyframes bl{50%{background:#4c0b0e}}
.tv{font-size:30px;font-weight:800;color:var(--ac);margin:6px 0}
.ts{font-size:13px;color:var(--mu);line-height:1.6}
.card{background:var(--card);border-radius:10px;padding:14px;margin-bottom:12px;border:1px solid var(--bd)}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
.btn{background:#1e293b;color:#fff;border:1px solid #334155;padding:8px 14px;border-radius:6px;cursor:pointer;font-weight:600}
.btn:hover{background:var(--ac);color:#070d1e}
.btn.on{background:var(--ok);color:#000;border-color:var(--ok)}
.btn.red{background:#991b1b;border:none}
.btn.sm{padding:4px 8px;font-size:12px}
table{width:100%;border-collapse:collapse;font-size:13px}
.tw{overflow-x:auto}
th,td{border:1px solid var(--bd);padding:8px;text-align:center;white-space:nowrap}
th{background:#030712;color:var(--ac)}
tr.mine td{color:var(--ok)}tr.toy td{color:var(--wa)}
.bdg{padding:3px 9px;border-radius:6px;font-size:12px;font-weight:700}
.bok{background:var(--ok);color:#000}.bda{background:var(--da);color:#fff}
.dec{display:grid;grid-template-columns:repeat(auto-fill,minmax(260px,1fr));gap:8px;margin-top:10px}
.di{display:flex;justify-content:space-between;align-items:center;background:#030712;padding:8px 12px;border-radius:8px;border:1px solid var(--bd);font-size:13px;gap:8px}
input,select{background:#030712;color:#fff;border:1px solid var(--bd);padding:7px 10px;border-radius:6px;font-family:monospace;font-size:14px;text-align:center}
.idin{width:120px}.nin{width:72px}
.er{display:flex;justify-content:space-between;align-items:center;padding:10px 0;border-bottom:1px solid var(--bd);gap:8px;flex-wrap:wrap}
.meter{direction:ltr;height:22px;background:#030712;border:1px solid var(--bd);border-radius:6px;position:relative;overflow:hidden;margin:10px 0}
.mf{height:100%;background:var(--ac);transition:width .2s}
.mk{position:absolute;top:0;bottom:0;width:2px}
.log{font-family:monospace;font-size:12px;direction:ltr;text-align:left;white-space:pre-wrap;word-break:break-all;background:#030712;padding:10px;border-radius:8px;max-height:65vh;overflow:auto}
.mu{color:var(--mu);font-size:12px}
h3{margin:0 0 10px}
@media(max-width:520px){.grid{grid-template-columns:1fr 1fr;gap:8px}.tv{font-size:24px}.tc{padding:10px}}
</style>
</head>
<body>
<header>
  <div><strong>مراقب الإطارات TPMS PRO S3</strong> <span id="conn" class="bdg bok">متصل</span></div>
  <div id="rfinfo" class="mu">CC1101: --</div>
</header>

<div class="tabs" id="tabs">
  <button class="tb on">إطاراتي</button>
  <button class="tb">الماسح</button>
  <button class="tb">الإطارات والحدود</button>
  <button class="tb">الراديو والهوائي</button>
  <button class="tb">البروتوكولات</button>
  <button class="tb">سجل الحزم</button>
  <button class="tb">النظام</button>
  <button class="tb" style="border-color:#d946ef">🔧 فني الإطارات</button>
</div>

<div class="pg on">
  <div class="card row" style="justify-content:space-between">
    <span>الوضع الحالي: <strong id="mlabel" style="color:var(--ac)">--</strong></span>
    <div class="row">
      <button class="btn" id="m0" onclick="cmd('mode=car')">السيارة</button>
      <button class="btn" id="m1" onclick="cmd('mode=scan')">الماسح</button>
      <button class="btn" id="m2" onclick="cmd('mode=radio')">تشخيص الراديو</button>
      <button class="btn" id="m3" onclick="cmd('mode=tech');setTab(7)">فني الإطارات</button>
    </div>
  </div>
  <div class="grid">
    <div class="tc" id="c0"></div><div class="tc" id="c1"></div>
    <div class="tc" id="c3"></div><div class="tc" id="c2"></div>
  </div>
  <div class="tc" id="c4"></div>
</div>

<div class="pg">
  <div class="card">
    <div class="row" style="justify-content:space-between">
      <h3>الحساسات الملتقطة (الأحدث أولاً) — <span id="scnt">0</span></h3>
      <button class="btn red sm" onclick="if(confirm('مسح قائمة الماسح؟'))cmd('clearscan=1')">مسح القائمة</button>
    </div>
    <div class="tw"><table>
      <thead><tr><th>ID</th><th>البروتوكول</th><th>الضغط</th><th>الحرارة</th><th>RSSI</th><th>أفضل</th><th>البطارية</th><th>منذ</th><th>العدد</th><th>ربط</th></tr></thead>
      <tbody id="stb"></tbody>
    </table></div>
    <p class="mu">الأخضر = إطاراتك، البرتقالي = حساسات تويوتا أخرى. "أفضل" = أقوى إشارة سُجّلت للحساس.</p>
  </div>
</div>

<div class="pg">
  <div class="card">
    <h3>معرفات الحساسات وحدود الإنذار</h3>
    <p class="mu">المعرف HEX (حتى 10 خانات). اتركه فارغاً لإلغاء الربط. الضغط بالبار، الحرارة بالدرجة المئوية.</p>
    <div id="tedit"></div>
    <p class="mu">كشف التسريب السريع: إنذار عند هبوط الضغط 0.2 بار أو أكثر خلال 10 دقائق.</p>
  </div>
</div>

<div class="pg">
  <div class="card">
    <h3>مقياس الإشارة الحي (لتوجيه الهوائي)</h3>
    <div class="row" style="justify-content:space-between"><span>RSSI الحالي: <strong id="rcur">--</strong> dBm</span><span class="mu">الضجيج: <span id="rnoise">--</span> | العتبة: <span id="rthr">--</span></span></div>
    <div class="meter"><div class="mf" id="mf" style="width:0"></div><div class="mk" id="mkn" style="background:#94a3b8"></div><div class="mk" id="mkt" style="background:var(--wa)"></div></div>
    <div class="mu">من -110 إلى -30 dBm. الخط الرمادي = أرضية الضجيج، البرتقالي = عتبة الالتقاط. أي إشارة تتجاوز العتبة تُسجَّل وتُحلَّل.</div>
    <div class="ts" id="rstats" style="margin-top:10px"></div>
  </div>
  <div class="card">
    <div><strong>حساسية الالتقاط (فرق العتبة فوق الضجيج): </strong><span id="rssiv" style="color:var(--ac)">--</span> dB</div>
    <div class="row" style="margin-top:8px">
      <button class="btn" onclick="cmd('rssi='+(cfg.rssi-1))">- حساسية أعلى / مدى أبعد</button>
      <button class="btn" onclick="cmd('rssi='+(cfg.rssi+1))">+ ضجيج أقل</button>
    </div>
    <p class="mu">القيمة المنخفضة (6–9) تلتقط الحساسات البعيدة مع الهوائي الخارجي لكن تزيد الإشارات العشوائية. الافتراضي 9.</p>
  </div>
  <div class="card">
    <div class="row"><strong>عرض نطاق الاستقبال:</strong>
      <select id="bwsel" onchange="cmd('bw='+this.value)"></select> kHz</div>
    <p class="mu">الافتراضي 102 kHz: الأفضل في الاختبار الميداني (يفك حتى -98 dBm مقابل -93 عند 270). الأضيق (68–81) يفقد حساسات منحرفة التردد.</p>
  </div>
  <div class="card">
    <div class="row"><strong>معايرة التردد:</strong>
      <button class="btn sm" onclick="cmd('foff='+(cfg.foff-5))">-5</button>
      <span id="foffv" style="color:var(--ac);min-width:70px;text-align:center">--</span>
      <button class="btn sm" onclick="cmd('foff='+(cfg.foff+5))">+5</button> kHz
      <span class="mu">→ <span id="freqv">--</span> MHz</span>
      <button class="btn sm" onclick="cmd('foff=0')">صفر</button>
    </div>
    <p class="mu">بلورات CC1101 الرخيصة تنحرف 20–50 kHz. جرّب قيماً مختلفة مع النطاق الضيق ولاحظ ارتفاع RSSI وعدد الحزم المفكوكة.</p>
  </div>
</div>

<div class="pg">
  <div class="card">
    <div class="row" style="justify-content:space-between">
      <h3>بروتوكولات وضع الماسح (<span id="decn">0</span> مفعّل)</h3>
      <div class="row">
        <button class="btn sm" onclick="cmd('decmask=all')">تفعيل الكل</button>
        <button class="btn sm" onclick="cmd('decmask=0')">تعطيل الكل</button>
        <button class="btn on sm" onclick="cmd('mode=scan')">انتقل للماسح</button>
      </div>
    </div>
    <p class="mu">وضع السيارة يستخدم بروتوكولَي تويوتا فقط دائماً. تعطيل البروتوكولات غير المطلوبة يقلل الالتقاط الخاطئ.</p>
    <div class="dec" id="decl"></div>
  </div>
</div>

<div class="pg">
  <div class="card">
    <div class="row" style="justify-content:space-between"><h3>آخر الحزم المفكوكة (JSON)</h3><button class="btn sm" onclick="loadLog()">تحديث</button></div>
    <div class="log" id="logv">...</div>
  </div>
</div>

<div class="pg">
  <div class="card ts" id="sysv"></div>
  <div class="card">
    <div class="row">
      <button class="btn" onclick="location.href='/update'">تحديث البرنامج لاسلكياً (OTA)</button>
      <button class="btn" onclick="if(confirm('إعادة تشغيل الجهاز؟'))cmd('reboot=1')">إعادة التشغيل</button>
      <button class="btn red" onclick="if(confirm('إيقاف الواي فاي؟ لإعادته: اضغط الزر مطولاً (1.5 ثانية).'))cmd('wifi=off')">إيقاف الواي فاي</button>
    </div>
    <p class="mu">OTA: المستخدم admin وكلمة المرور هي كلمة مرور الشبكة. ارفع ملف .bin الناتج من Arduino (Export Compiled Binary).</p>
  </div>
</div>

<div class="pg">
  <div class="card row" style="justify-content:space-between">
    <div><strong>وضع فني الإطارات</strong> <span id="ton" class="bdg bda">غير مفعّل</span><br>
      <span class="mu">يلتقط الحساس الأقرب فقط (إشارة ≥ <span dir="ltr"><strong id="tthr">--</strong> dBm</span>) ويتجاهل السيارات البعيدة.</span></div>
    <div class="row">
      <button class="btn on" onclick="cmd('mode=tech')">تفعيل</button>
      <button class="btn sm" onclick="cmd('techthr='+(T.thr-5))">-5 أبعد</button>
      <button class="btn sm" onclick="cmd('techthr='+(T.thr+5))">+5 أقرب</button>
      <button class="btn red" onclick="cmd('techreset=1')">التقاط جديد</button>
    </div>
  </div>
  <div class="card" id="tcap"></div>
  <div class="card">
    <div class="row" style="justify-content:space-between"><h3>سجل الفحص</h3><button class="btn sm" onclick="if(confirm('مسح سجل الفحص؟'))cmd('techclear=1')">مسح السجل</button></div>
    <div class="tw"><table>
      <thead><tr><th>ID</th><th>البروتوكول</th><th>الضغط</th><th>الحرارة</th><th>البطارية</th><th>الإشارة</th><th>الحكم</th><th>منذ</th><th>ربط</th></tr></thead>
      <tbody id="thist"></tbody>
    </table></div>
  </div>
  <p class="mu">لإيقاظ الحساس: نفّس الإطار قليلاً، أو دوّر العجلة، أو استخدم أداة تنشيط TPMS (LF 125kHz). قرّب الجهاز من صمام الإطار. إن التُقطت سيارات مارّة فارفع العتبة (+5)، وإن لم يُلتقط الإطار القريب فاخفضها (-5).</p>
</div>

<script>
const TN=["الأمامي أيسر (FL)","الأمامي أيمن (FR)","الخلفي أيمن (RR)","الخلفي أيسر (RL)","الاحتياطي (SP)"];
const MODES=["إطارات سيارتي (تويوتا)","الماسح العام","تشخيص الراديو","فني الإطارات"];
let T={thr:-70};
function verdict(c){if(c.batt==0)return['بطارية ضعيفة','var(--da)'];if(c.bar==null)return['لا توجد قراءة ضغط','var(--wa)'];if(c.bar<0.3)return['ضغط صفر — غير مركّب؟','var(--wa)'];if(c.temp!=null&&c.temp>90)return['حرارة زائدة','var(--da)'];return['الحساس سليم','var(--ok)']}
const TS=['FL','FR','RR','RL','SP'];
function renderTech(t){
  T=t;$('tthr').innerText=t.thr;$('ton').className='bdg '+(t.on?'bok':'bda');$('ton').innerText=t.on?'مفعّل':'غير مفعّل';
  const c=t.cap;
  if(!c){$('tcap').innerHTML=`<div style="text-align:center;padding:20px"><div style="font-size:22px;font-weight:700">قرّب الجهاز من الإطار</div><div class="mu" style="margin-top:8px">الإشارة الحية: <span dir="ltr">${t.live} dBm</span> — العتبة: <span dir="ltr">${t.thr} dBm</span></div></div>`;}
  else{const v=verdict(c);
    $('tcap').innerHTML=`<div class="row" style="justify-content:space-between"><div><div class="mu">معرف الحساس</div><div style="font-size:30px;font-weight:800;font-family:monospace;direction:ltr">${esc(c.id)}</div>
    <div class="mu">${esc(c.pn)}${c.tire>=0?' — <strong style="color:var(--ok)">إطار سيارتي '+TS[c.tire]+'</strong>':''}</div></div>
    <div style="text-align:center"><div class="tv" style="font-size:42px">${num(c.bar,2)} <small style="font-size:16px">بار</small></div><div class="ts"><span dir="ltr">${c.bar==null?'--':(c.bar*14.5038).toFixed(1)} PSI | ${num(c.temp,0)}°C</span></div></div></div>
    <div style="margin:12px 0;padding:12px;border-radius:8px;border:2px solid ${v[1]};color:${v[1]};font-size:20px;font-weight:800;text-align:center">${v[0]}</div>
    <div class="ts">الإشارة: <span dir="ltr">${c.rssi} dBm</span> (الأقوى <span dir="ltr">${c.best}</span>) | البطارية: ${c.batt==1?'سليمة':c.batt==0?'ضعيفة':'غير متاحة'} | الحزم: ${c.cnt} | منذ ${age(c.age)} | التحقق: ${esc(c.mic||'-')}${c.st!=null?' | الحالة: 0x'+c.st.toString(16).toUpperCase():''}</div>
    <div class="row" style="margin-top:10px"><button class="btn" onclick="assignQuick('${esc(c.id)}')">ربط بإطار من سيارتي</button><button class="btn" onclick="navigator.clipboard&&navigator.clipboard.writeText('${esc(c.id)}')">نسخ المعرف</button></div>`;}
  $('thist').innerHTML=t.hist.map(h=>{const v=verdict(h);return `<tr><td><strong>${esc(h.id)}</strong>${h.tire>=0?' ('+TS[h.tire]+')':''}</td><td>${esc(h.m)}</td><td>${num(h.bar,2)}</td><td>${num(h.temp,0)}°</td><td>${h.batt==1?'سليمة':h.batt==0?'ضعيفة':'—'}</td><td>${h.best}</td><td style="color:${v[1]}">${v[0]}</td><td>${age(h.age)}</td><td><button class="btn sm" onclick="assignQuick('${esc(h.id)}')">ربط</button></td></tr>`}).join('')||'<tr><td colspan="9" class="mu">لا يوجد</td></tr>';
}
let cfg=null,tab=0,busy=false;
const $=id=>document.getElementById(id);
const esc=s=>String(s==null?'':s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const num=(v,d)=>v==null?'--':v.toFixed(d);
const age=s=>s==null?'--':s<60?s+' ث':s<3600?Math.floor(s/60)+' د':Math.floor(s/3600)+' س';
function alarmTxt(a){let r=[];if(a&16)r.push('تسريب سريع!');if(a&1)r.push('ضغط منخفض');if(a&4)r.push('حرارة مرتفعة');if(a&2)r.push('ضغط مرتفع');if(a&8)r.push('بطارية ضعيفة');return r.join(' • ')}
document.querySelectorAll('.tb').forEach((b,i)=>b.onclick=()=>setTab(i));
function setTab(i){tab=i;document.querySelectorAll('.tb').forEach((b,j)=>b.classList.toggle('on',i==j));document.querySelectorAll('.pg').forEach((p,j)=>p.classList.toggle('on',i==j));if(i==5)loadLog();if(i==2&&cfg)renderEdit()}
async function cmd(q){try{const r=await fetch('/cmd?'+q);if(!r.ok)alert(await r.text());}catch(e){}await loadCfg();await upd()}
async function loadCfg(){try{cfg=await (await fetch('/api/config')).json();renderCfg()}catch(e){}}
function renderCfg(){
  $('rssiv').innerText=cfg.rssi;$('foffv').innerText=(cfg.foff>0?'+':'')+cfg.foff;$('freqv').innerText=cfg.freq.toFixed(3);
  $('bwsel').innerHTML=cfg.bws.map(b=>`<option value="${b}"${b==cfg.bw?' selected':''}>${b}</option>`).join('');
  let n=0;$('decl').innerHTML=cfg.decs.map((d,i)=>{const on=(cfg.mask>>>i)&1;n+=on;return `<div class="di"><span>${esc(d)}${i<2?' <span class="bdg bok">تويوتا</span>':''}</span><button class="btn sm ${on?'on':''}" onclick="cmd('dectoggle=${i}')">${on?'مفعل':'معطل'}</button></div>`}).join('');
  $('decn').innerText=n;
  if(tab!=2||!document.activeElement||document.activeElement.tagName!='INPUT')renderEdit();
}
function renderEdit(){
  $('tedit').innerHTML=cfg.tires.map((t,i)=>`<div class="er"><strong style="min-width:150px">${TN[i]}</strong>
  <div class="row"><input class="idin" id="id${i}" value="${esc(t.id)}" maxlength="10" placeholder="بدون">
  <span class="mu">أدنى</span><input class="nin" id="pn${i}" type="number" step="0.05" value="${t.pmin}">
  <span class="mu">أعلى</span><input class="nin" id="px${i}" type="number" step="0.05" value="${t.pmax}">
  <span class="mu">حرارة</span><input class="nin" id="tx${i}" type="number" step="1" value="${t.tmax}">
  <button class="btn on sm" onclick="saveTire(${i})">حفظ</button></div></div>`).join('');
}
async function saveTire(i){
  const id=$('id'+i).value.trim();
  await fetch(`/cmd?settire=${i}&id=${encodeURIComponent(id)}`).then(async r=>{if(!r.ok)alert(await r.text())});
  await cmd(`thr=${i}&pmin=${$('pn'+i).value}&pmax=${$('px'+i).value}&tmax=${$('tx'+i).value}`);
}
function assignQuick(id){
  const t=prompt("اختر رقم الإطار لربطه بالمعرف ["+id+"]:\n1: الأمامي أيسر (FL)\n2: الأمامي أيمن (FR)\n3: الخلفي أيمن (RR)\n4: الخلفي أيسر (RL)\n5: الاحتياطي (SP)");
  const i=parseInt(t);if(i>=1&&i<=5)cmd(`settire=${i-1}&id=${encodeURIComponent(id)}`);
}
function tireCard(i,t){
  const el=$('c'+i);let cls='tc',body;
  if(!t.id){body=`<div class="ts">غير مربوط</div>`}
  else if(!t.has){body=`<div class="tv">--</div><div class="ts">بانتظار الإشارة<br>${esc(t.id)}</div>`}
  else{
    if(t.stale)cls+=' st';else if(t.al&21)cls+=' da';else if(t.al)cls+=' wa';
    const bat=t.batt==0?'<span class="bdg bda">بطارية ضعيفة</span>':t.batt==1?'بطارية سليمة':'';
    body=`<div class="tv">${num(t.bar,2)} <small style="font-size:14px">بار</small></div>
    <div class="ts"><span dir="ltr">${t.bar==null?'--':(t.bar*14.5038).toFixed(1)} PSI | ${num(t.temp,0)}°C | ${t.rssi} dBm</span><br>
    منذ ${age(t.age)} | حزم: ${t.cnt} ${bat}<br>
    ${t.stale?'<span style="color:var(--mu)">لا إشارة حديثة</span>':t.al?'<strong style="color:'+(t.al&21?'var(--da)':'var(--wa)')+'">'+alarmTxt(t.al)+'</strong>':'<span style="color:var(--ok)">سليم</span>'}</div>`;
  }
  if(t.tr&&t.tr.length>1)body+=spark(t.tr,cfg&&cfg.tires[i]);
  el.className=cls;el.innerHTML=`<div>${TN[i]}</div>`+body;
}
// منحنى الضغط لآخر 90 دقيقة مع خطّي الحد الأدنى والأعلى
function spark(v,lim){
  const W=200,H=36;let lo=Math.min(...v),hi=Math.max(...v);
  if(lim){lo=Math.min(lo,lim.pmin);hi=Math.max(hi,lim.pmax)}
  lo-=0.05;hi+=0.05;const y=b=>(H-(b-lo)/(hi-lo)*H).toFixed(1),x=k=>(k*W/(v.length-1)).toFixed(1);
  let s=`<svg viewBox="0 0 ${W} ${H}" preserveAspectRatio="none" style="width:100%;height:${H}px;margin-top:6px;direction:ltr">`;
  if(lim)s+=`<line x1="0" x2="${W}" y1="${y(lim.pmin)}" y2="${y(lim.pmin)}" stroke="#ef4444" stroke-dasharray="3 3" stroke-width="1"/><line x1="0" x2="${W}" y1="${y(lim.pmax)}" y2="${y(lim.pmax)}" stroke="#f59e0b" stroke-dasharray="3 3" stroke-width="1"/>`;
  s+=`<polyline fill="none" stroke="#38bdf8" stroke-width="2" points="${v.map((b,k)=>x(k)+','+y(b)).join(' ')}"/></svg>`;
  return s+`<div class="mu">آخر ${v.length} دقيقة</div>`;
}
async function upd(){
  if(busy)return;busy=true;
  try{
    const d=await (await fetch('/api/data')).json();
    $('conn').className='bdg bok';$('conn').innerText='متصل';
    $('rfinfo').innerText=`CC1101: ${d.cc?'READY':'ERROR'} | حزم: ${d.pk} | ${d.freq.toFixed(3)} MHz`;
    $('mlabel').innerText=MODES[d.mode];[0,1,2,3].forEach(m=>$('m'+m).className='btn'+(d.mode==m?' on':''));
    if(d.tech)renderTech(d.tech);
    d.tires.forEach((t,i)=>tireCard(i,t));
    $('scnt').innerText=d.scan.length;
    $('stb').innerHTML=d.scan.map(s=>`<tr class="${s.tire>=0?'mine':(s.p<2&&s.p>=0?'toy':'')}"><td><strong>${esc(s.id)}</strong>${s.tire>=0?' ('+['FL','FR','RR','RL','SP'][s.tire]+')':''}</td><td>${esc(s.m)}</td><td>${num(s.bar,2)}</td><td>${num(s.temp,0)}°</td><td>${s.rssi}</td><td>${s.best}</td><td>${s.batt==0?'ضعيفة':s.batt==1?'سليمة':'—'}</td><td>${age(s.age)}</td><td>${s.cnt}</td><td><button class="btn sm" onclick="assignQuick('${esc(s.id)}')">ربط</button></td></tr>`).join('');
    const r=d.radio,pc=v=>Math.max(0,Math.min(100,(v+110)*100/80));
    $('rcur').innerText=r.cur;$('rnoise').innerText=r.avg;$('rthr').innerText=r.thr;
    $('mf').style.width=pc(r.cur)+'%';$('mf').style.background=r.cur>r.thr?'var(--ok)':'var(--ac)';
    $('mkn').style.left=pc(r.avg)+'%';$('mkt').style.left=pc(r.thr)+'%';
    $('rstats').innerHTML=`إشارات ملتقطة: ${r.cap} | مفكوكة: ${r.msg} | غير معروفة: ${r.zero} | مفقودة: ${r.dc}/${r.dq}/${r.dp} | مرفوضة (قيم مستحيلة): ${r.rej}<br>آخر حزمة: ${r.last?esc(r.last)+' ('+r.lrssi+' dBm، منذ '+age(r.lage)+')':'لا يوجد بعد'} | إعادات تهيئة الراديو: ${r.rec}`;
    $('sysv').innerHTML=`الإصدار: ${esc(d.sys.fw)} | التشغيل: ${age(d.sys.up)}<br>الذاكرة الداخلية الحرة: ${d.sys.heap} KB | PSRAM الحرة: ${d.sys.psram} KB<br>آخر إعادة تشغيل: ${esc(d.sys.reset)} | عملاء الواي فاي: ${d.sys.cl}`;
  }catch(e){$('conn').className='bdg bda';$('conn').innerText='غير متصل'}
  busy=false;
}
async function loadLog(){try{const d=await (await fetch('/api/log')).json();$('logv').textContent=d.lines.length?d.lines.join('\n\n'):'لا توجد حزم بعد';}catch(e){}}
loadCfg();upd();setInterval(upd,1000);setInterval(()=>{if(tab==5)loadLog()},3000);
</script>
</body>
</html>
)rawliteral";

// دالة إعادة التوجيه لصفحة البوابة (Captive Portal)
void handleCaptivePortal() {
  server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
  server.send(302, "text/plain", "");
}

const char* resetReasonText() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "تشغيل الطاقة";
    case ESP_RST_SW:       return "برمجية";
    case ESP_RST_PANIC:    return "انهيار (Panic)";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      return "Watchdog";
    case ESP_RST_BROWNOUT: return "انخفاض الجهد";
    default:               return "أخرى";
  }
}

void techToJson(JsonObject o, const TechCapture& t, uint32_t now) {
  o["id"] = t.id;
  o["p"] = t.proto;
  o["m"] = protoShortName(t.proto);
  o["pn"] = rtl_433_ESP::decoderName(t.proto >= 0 ? t.proto : 0);
  if (!isnan(t.bar)) o["bar"] = t.bar;
  if (!isnan(t.temp)) o["temp"] = t.temp;
  o["batt"] = t.battery;
  o["rssi"] = t.rssi;
  o["best"] = t.bestRssi;
  o["cnt"] = t.count;
  o["age"] = (now - t.lastSeen) / 1000;
  o["tire"] = t.tire;
  o["mic"] = t.mic;
  if (t.status >= 0) o["st"] = t.status;
}

void handleApiData() {
  JsonDocument doc;
  uint32_t now = millis();
  doc["mode"] = displayMode;
  doc["cc"] = (bool)ccOk;
  doc["pk"] = rfPacketCounter;
  doc["freq"] = rtl_433_ESP::currentFrequency;

  rtl_433_ESPStatus st = rtl_433_ESP::statusSnapshot();
  JsonObject r = doc["radio"].to<JsonObject>();
  r["cur"] = st.currentRssi;
  r["avg"] = st.averageRssi;
  r["thr"] = st.rssiThreshold;
  r["cap"] = st.capturedSignals;
  r["msg"] = st.decodedMessages;
  r["zero"] = st.zeroDecodedSignals;
  r["dc"] = st.droppedCaptureBuffers;
  r["dq"] = st.droppedDecoderQueue;
  r["dp"] = rfQueueDrops;
  r["rec"] = ccRecoveries;
  r["rej"] = rfRejected;

  JsonObject sys = doc["sys"].to<JsonObject>();
  sys["fw"] = FW_VERSION " (" __DATE__ ")";
  sys["up"] = now / 1000;
  sys["heap"] = ESP.getFreeHeap() / 1024;
  sys["psram"] = ESP.getFreePsram() / 1024;
  sys["reset"] = resetReasonText();
  sys["cl"] = WiFi.softAPgetStationNum();

  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    if (lastPktMillis) {
      char last[32];
      snprintf(last, sizeof(last), "%s %s", lastPktId, protoShortName(lastPktProto));
      r["last"] = last;
      r["lrssi"] = lastPktRssi;
      r["lage"] = (now - lastPktMillis) / 1000;
    }

    JsonArray ta = doc["tires"].to<JsonArray>();
    for (int i = 0; i < NUM_TIRES; i++) {
      const TireState& t = tires[i];
      JsonObject o = ta.add<JsonObject>();
      o["id"] = tireCfg[i].id;
      o["has"] = t.hasData;
      if (!t.hasData) continue;
      if (!isnan(t.bar)) o["bar"] = t.bar;
      if (!isnan(t.temp)) o["temp"] = t.temp;
      o["batt"] = t.battery;
      o["rssi"] = t.rssi;
      o["age"] = (now - t.lastRx) / 1000;
      o["cnt"] = t.count;
      o["al"] = t.alarms;
      o["stale"] = tireIsStale(t, now);
      if (t.status >= 0) o["st"] = t.status;
      if (t.trendN > 1) {
        JsonArray tr = o["tr"].to<JsonArray>();
        for (uint8_t k = 0; k < t.trendN; k++) {
          uint8_t idx = (t.trendHead + TREND_N - t.trendN + k) % TREND_N;
          tr.add(serialized(String(t.trend[idx], 2)));
        }
      }
    }

    JsonObject te = doc["tech"].to<JsonObject>();
    te["thr"] = techThreshold;
    te["on"] = displayMode == MODE_TECH;
    te["live"] = rtl_433_ESP::currentRssi;
    if (techCap.valid) techToJson(te["cap"].to<JsonObject>(), techCap, now);
    JsonArray th = te["hist"].to<JsonArray>();
    for (int i = 0; i < techHistCount; i++) techToJson(th.add<JsonObject>(), techHist[i], now);

    uint8_t order[MAX_SCANNED];
    int n = buildRecentOrder(order);
    JsonArray sa = doc["scan"].to<JsonArray>();
    for (int i = 0; i < n; i++) {
      const ScannedSensor& s = scannedList[order[i]];
      JsonObject o = sa.add<JsonObject>();
      o["id"] = s.id;
      o["p"] = s.proto;
      o["m"] = protoShortName(s.proto);
      if (!isnan(s.bar)) o["bar"] = s.bar;
      if (!isnan(s.temp)) o["temp"] = s.temp;
      o["rssi"] = s.rssi;
      o["best"] = s.bestRssi;
      o["batt"] = s.battery;
      o["age"] = (now - s.lastSeen) / 1000;
      o["cnt"] = s.count;
      o["tire"] = s.tire;
    }
    xSemaphoreGive(dataMutex);
  }

  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleApiConfig() {
  JsonDocument doc;
  doc["rssi"] = rssiThresholdDelta;
  doc["bw"] = (int)fskBandwidthSetting;
  doc["foff"] = freqOffsetKhz;
  doc["freq"] = currentFrequencyMhz();
  doc["mask"] = userDecoderMask;
  JsonArray bws = doc["bws"].to<JsonArray>();
  for (int i = 0; i < CC1101_BW_COUNT; i++) bws.add((int)CC1101_BW_LIST[i]);
  JsonArray decs = doc["decs"].to<JsonArray>();
  for (int i = 0; i < TPMS_NUM_DECODERS; i++) decs.add(rtl_433_ESP::decoderName(i));

  JsonArray ta = doc["tires"].to<JsonArray>();
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    for (int i = 0; i < NUM_TIRES; i++) {
      JsonObject o = ta.add<JsonObject>();
      o["id"] = tireCfg[i].id;
      o["pmin"] = serialized(String(tireCfg[i].pMin, 2));
      o["pmax"] = serialized(String(tireCfg[i].pMax, 2));
      o["tmax"] = serialized(String(tireCfg[i].tMax, 0));
    }
    xSemaphoreGive(dataMutex);
  }
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleApiLog() {
  JsonDocument doc;
  JsonArray lines = doc["lines"].to<JsonArray>();
  if (logBuf && xSemaphoreTake(dataMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    for (int i = 0; i < logCount; i++) {
      int idx = (logHead - 1 - i + LOG_LINES) % LOG_LINES;
      lines.add(logBuf[idx]);
    }
    xSemaphoreGive(dataMutex);
  }
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

void handleCmd() {
  if (server.hasArg("wifi") && server.arg("wifi") == "off") {
    server.send(200, "text/plain", "WIFI_TURNING_OFF");
    requestWifi(false, 500); // بعد إرسال الرد، من داخل loop()
    return;
  }
  if (server.hasArg("reboot")) {
    server.send(200, "text/plain", "REBOOTING");
    rebootAt = millis() + 800;
    rebootRequested = true;
    return;
  }

  if (server.hasArg("settire") && server.hasArg("id")) {
    int idx = server.arg("settire").toInt();
    String raw = server.arg("id");
    raw.trim();
    if (idx < 0 || idx >= NUM_TIRES) { server.send(400, "text/plain", "رقم إطار غير صالح"); return; }
    for (size_t i = 0; i < raw.length(); i++) {
      if (!isxdigit((unsigned char)raw[i])) { server.send(400, "text/plain", "المعرف يجب أن يكون HEX فقط (0-9 A-F)"); return; }
    }
    if (raw.length() > 10) { server.send(400, "text/plain", "المعرف أطول من 10 خانات"); return; }
    char norm[12];
    normalizeId(raw.c_str(), norm, sizeof(norm));
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      // نفس المعرف لا يُربط بإطارين
      for (int i = 0; i < NUM_TIRES; i++) {
        if (i != idx && norm[0] && strcmp(tireCfg[i].id, norm) == 0) tireCfg[i].id[0] = '\0';
      }
      strlcpy(tireCfg[idx].id, norm, sizeof(tireCfg[idx].id));
      memset(&tires[idx], 0, sizeof(TireState));
      for (int i = 0; i < scannedCount; i++) scannedList[i].tire = findTireById(scannedList[i].id);
      xSemaphoreGive(dataMutex);
      markDirty();
    }
  }

  if (server.hasArg("thr")) {
    int idx = server.arg("thr").toInt();
    if (idx >= 0 && idx < NUM_TIRES) {
      float pmin = server.hasArg("pmin") ? server.arg("pmin").toFloat() : tireCfg[idx].pMin;
      float pmax = server.hasArg("pmax") ? server.arg("pmax").toFloat() : tireCfg[idx].pMax;
      float tmax = server.hasArg("tmax") ? server.arg("tmax").toFloat() : tireCfg[idx].tMax;
      if (pmin < 0.0f || pmax > 12.0f || pmin >= pmax || tmax < 20.0f || tmax > 150.0f) {
        server.send(400, "text/plain", "حدود غير منطقية: يجب أن يكون الأدنى أقل من الأعلى، والحرارة بين 20 و150");
        return;
      }
      if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
        tireCfg[idx].pMin = pmin;
        tireCfg[idx].pMax = pmax;
        tireCfg[idx].tMax = tmax;
        TireState& t = tires[idx];
        if (t.hasData) t.alarms = computeAlarms(idx, t.bar, t.temp, t.battery, (t.alarms & AL_LEAK) != 0, 0);
        xSemaphoreGive(dataMutex);
        markDirty();
      }
    }
  }

  if (server.hasArg("mode")) {
    String m = server.arg("mode");
    if (m == "car")        applyMode(MODE_CAR);
    else if (m == "scan")  applyMode(MODE_SCAN);
    else if (m == "radio") applyMode(MODE_RADIO);
    else if (m == "tech")  applyMode(MODE_TECH);
  }

  if (server.hasArg("rssi")) applyRssiDelta(server.arg("rssi").toInt());
  if (server.hasArg("bw"))   applyBandwidth(server.arg("bw").toFloat());
  if (server.hasArg("foff")) applyFreqOffset(server.arg("foff").toInt());

  if (server.hasArg("dectoggle")) {
    int idx = server.arg("dectoggle").toInt();
    if (idx >= 0 && idx < TPMS_NUM_DECODERS) {
      userDecoderMask ^= (1UL << idx);
      markDirty();
    }
  }
  if (server.hasArg("decmask")) {
    String v = server.arg("decmask");
    userDecoderMask = (v == "all") ? TPMS_ALL_MASK : ((uint32_t)strtoul(v.c_str(), nullptr, 0) & TPMS_ALL_MASK);
    markDirty();
  }
  if (server.hasArg("dectoggle") || server.hasArg("decmask")) {
    activeDecoderMask = maskForMode(displayMode);
    rf.setDecoderMask(activeDecoderMask);
  }

  if (server.hasArg("techthr")) {
    techThreshold = constrain((int)server.arg("techthr").toInt(), -100, -20);
    markDirty();
  }
  if (server.hasArg("techreset")) {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      techReset();
      xSemaphoreGive(dataMutex);
    }
  }
  if (server.hasArg("techclear")) {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      memset(&techCap, 0, sizeof(techCap));
      techHistCount = 0;
      xSemaphoreGive(dataMutex);
    }
  }

  if (server.hasArg("clearscan")) {
    if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      scannedCount = 0;
      xSemaphoreGive(dataMutex);
    }
  }

  screenNeedsFullRedraw = true;
  server.send(200, "application/json", "{\"ok\":true}");
}

// تُستدعى مرة واحدة فقط في setup — إعادة تشغيل الواي فاي تستخدم server.begin()
void setupWebServer() {
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", INDEX_HTML); });

  // مسارات فحص الاتصال التلقائي (Android, iOS, Windows)
  server.on("/generate_204", HTTP_GET, handleCaptivePortal);
  server.on("/gen_204", HTTP_GET, handleCaptivePortal);
  server.on("/hotspot-detect.html", HTTP_GET, handleCaptivePortal);
  server.on("/canonical.html", HTTP_GET, handleCaptivePortal);
  server.on("/connecttest.txt", HTTP_GET, handleCaptivePortal);
  server.on("/ncsi.txt", HTTP_GET, handleCaptivePortal);
  server.on("/redirect", HTTP_GET, handleCaptivePortal);
  server.onNotFound(handleCaptivePortal);

  server.on("/api/data", HTTP_GET, handleApiData);
  server.on("/api/config", HTTP_GET, handleApiConfig);
  server.on("/api/log", HTTP_GET, handleApiLog);
  server.on("/cmd", HTTP_GET, handleCmd);

  httpUpdater.setup(&server, "/update", "admin", AP_PASS);
  server.begin();
}

// =============================================================================
// تهيئة النظام
// =============================================================================
void setup() {
  Serial.setRxBufferSize(4096); // أوامر inject طويلة قد تتراكم أثناء تصدير صورة الشاشة
  Serial.begin(115200);
  delay(50);
  Serial.printf("\n[SYSTEM] TPMS S3 PRO v%s (%s %s)\n", FW_VERSION, __DATE__, __TIME__);

  if (psramFound()) {
    Serial.printf("[SYSTEM] PSRAM: %d KB total, %d KB free\n", ESP.getPsramSize() / 1024, ESP.getFreePsram() / 1024);
  } else {
    Serial.println("[CRITICAL ERROR] PSRAM not found! Check Tools -> PSRAM -> OPI PSRAM");
  }

  dataMutex = xSemaphoreCreateMutex();
  rfQueue = xQueueCreate(RF_QUEUE_LEN, sizeof(RawRfPacket));

  memset(tires, 0, sizeof(tires));
  loadSettings();

  logBuf = (char (*)[LOG_LEN])heap_caps_calloc(LOG_LINES, LOG_LEN, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

  canvas = new PsramCanvas16(SCREEN_W, SCREEN_H);
  if (!canvas || !canvas->getBuffer()) {
    Serial.println("PSRAM Allocation Failed! Halting.");
    while (1) { delay(1000); }
  }

  pinMode(TFT_BLK, OUTPUT);
  digitalWrite(TFT_BLK, HIGH);

  tftSpi.begin(TFT_SCLK, -1, TFT_MOSI, -1);
  tft.init(240, 320);
  tft.setSPISpeed(40000000);
  tft.setRotation(1);
  tft.fillScreen(C_BG);

  // نقطة وصول + DNS للبوابة التلقائية
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  WiFi.setTxPower(WIFI_POWER_11dBm);
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
  setupWebServer();
  Serial.printf("[WIFI] AP '%s' @ %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());

  xTaskCreatePinnedToCore(rfLoopTask, "rfLoopTask", 8192, NULL, 3, &rfLoopTaskHandle, 1);   // النواة 1: بعيداً عن الواي فاي
  xTaskCreatePinnedToCore(tftUpdateTask, "tftUpdateTask", 8192, NULL, 1, &tftTaskHandle, 1);
  xTaskCreatePinnedToCore(packetProcessingTask, "packetProcessingTask", 8192, NULL, 2, &pktTaskHandle, 1);
  xTaskCreatePinnedToCore(buttonTask, "buttonTask", 3072, NULL, 2, NULL, 1);
}

// =============================================================================
// أوامر المنفذ التسلسلي (للتشخيص والاختبار)
//   shot      : تصدير صورة الشاشة (RGB565 بترميز base64)
//   mode N    : تبديل الوضع (0 سيارة، 1 ماسح، 2 راديو)
//   stat      : طباعة الإحصائيات فوراً
//   reboot    : إعادة التشغيل
// =============================================================================
void printStatusLine() {
  rtl_433_ESPStatus st = rtl_433_ESP::statusSnapshot();
  Serial.printf("[STAT] up %lus | mode %u mask 0x%07lX (%d active) | pkts %lu rej %lu | captured %u decoded %u | noise %d thr %d | heap %u KB (min %u) psram %u KB | radio resets %lu\n",
                (unsigned long)(millis() / 1000), displayMode, (unsigned long)activeDecoderMask,
                rtl_433_ESP::activeDecoderCount(), (unsigned long)rfPacketCounter, (unsigned long)rfRejected,
                st.capturedSignals, st.decodedMessages, st.averageRssi, st.rssiThreshold,
                ESP.getFreeHeap() / 1024, ESP.getMinFreeHeap() / 1024, ESP.getFreePsram() / 1024,
                (unsigned long)ccRecoveries);
  uint32_t n = rtl_433_ESP::decodeCount;
  Serial.printf("[PERF] decode avg %.1f ms max %.1f ms | queue peak %lu/40 | lost capture %u queue %u | cpu %u MHz\n",
                n ? rtl_433_ESP::decodeUsTotal / 1000.0 / n : 0.0, rtl_433_ESP::decodeUsMax / 1000.0,
                (unsigned long)rtl_433_ESP::queuePeak, rtl_433_ESP::droppedCaptureBuffers, rtl_433_ESP::droppedDecoderQueue,
                (unsigned)getCpuFrequencyMhz());
}

void dumpScreenshot() {
  const size_t bytes = SCREEN_W * SCREEN_H * 2;
  uint8_t* copy = (uint8_t*)heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!copy || !canvas || !canvas->getBuffer()) {
    Serial.println("[SHOT-ERROR]");
    if (copy) heap_caps_free(copy);
    return;
  }
  // انتظار إطار جديد يعكس آخر الحزم والأوامر قبل النسخ
  vTaskDelay(pdMS_TO_TICKS(150));
  screenNeedsFullRedraw = true;
  vTaskDelay(pdMS_TO_TICKS(350));
  // الرسم يتم تحت dataMutex، فالنسخ تحته يعطي إطاراً مكتملاً
  if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
    memcpy(copy, canvas->getBuffer(), bytes);
    xSemaphoreGive(dataMutex);
  }
  static const char b64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  serialQuiet = true;
  vTaskDelay(pdMS_TO_TICKS(20)); // إتاحة الوقت لأي طباعة جارية لتكتمل
  Serial.printf("[SHOT-BEGIN %dx%d RGB565LE]\n", SCREEN_W, SCREEN_H);
  char line[80];
  size_t i = 0;
  while (i < bytes) {
    int n = 0;
    for (int k = 0; k < 19 && i < bytes; k++) { // 57 بايت = 76 حرفاً لكل سطر
      uint32_t v = (uint32_t)copy[i] << 16;
      int have = 1;
      if (i + 1 < bytes) { v |= (uint32_t)copy[i + 1] << 8; have++; }
      if (i + 2 < bytes) { v |= copy[i + 2]; have++; }
      line[n++] = b64[(v >> 18) & 63];
      line[n++] = b64[(v >> 12) & 63];
      line[n++] = have > 1 ? b64[(v >> 6) & 63] : '=';
      line[n++] = have > 2 ? b64[v & 63] : '=';
      i += 3;
    }
    line[n] = '\0';
    Serial.println(line);
  }
  Serial.println("[SHOT-END]");
  serialQuiet = false;
  heap_caps_free(copy);
}

void handleSerialCommands() {
  static char line[400];
  static int len = 0;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      line[len] = '\0';
      if (len > 0) {
        if (strcmp(line, "shot") == 0) {
          dumpScreenshot();
        } else if (strncmp(line, "mode ", 5) == 0) {
          applyMode((uint8_t)atoi(line + 5));
          Serial.printf("[CMD] mode %u\n", displayMode);
        } else if (strcmp(line, "stat") == 0) {
          printStatusLine();
        } else if (strncmp(line, "bw ", 3) == 0) {
          applyBandwidth(atof(line + 3));
          Serial.printf("[CMD] bw %d kHz\n", (int)fskBandwidthSetting);
        } else if (strncmp(line, "rssi ", 5) == 0) {
          applyRssiDelta(atoi(line + 5));
          Serial.printf("[CMD] rssi delta %d dB\n", rssiThresholdDelta);
        } else if (strncmp(line, "foff ", 5) == 0) {
          applyFreqOffset(atoi(line + 5));
          Serial.printf("[CMD] freq %.3f MHz (offset %+d kHz)\n", currentFrequencyMhz(), freqOffsetKhz);
        } else if (strncmp(line, "techthr ", 8) == 0) {
          techThreshold = constrain(atoi(line + 8), -100, -20);
          markDirty();
          Serial.printf("[CMD] tech threshold %d dBm\n", techThreshold);
        } else if (strcmp(line, "techreset") == 0) {
          if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
            techReset();
            xSemaphoreGive(dataMutex);
          }
          Serial.println("[CMD] tech capture reset");
        } else if (strncmp(line, "inject ", 7) == 0) {
          // حقن حزمة JSON تجريبية في نفس مسار المعالجة (لاختبار العرض والإنذارات)
          logSignal(line + 7);
          Serial.println("[CMD] injected");
        } else if (strcmp(line, "cleartires") == 0) {
          if (xSemaphoreTake(dataMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
            memset(tires, 0, sizeof(tires));
            memset(&techCap, 0, sizeof(techCap));
            techHistCount = 0;
            scannedCount = 0;
            logCount = 0;
            lastPktMillis = 0;
            xSemaphoreGive(dataMutex);
          }
          screenNeedsFullRedraw = true;
          Serial.println("[CMD] tires cleared");
        } else if (strcmp(line, "reboot") == 0) {
          rebootAt = millis();
          rebootRequested = true;
        } else {
          Serial.printf("[CMD] unknown: %s (shot | mode N | bw N | rssi N | foff N | techthr N | techreset | stat | inject JSON | cleartires | reboot)\n", line);
        }
      }
      len = 0;
    } else if (len < (int)sizeof(line) - 1) {
      line[len++] = ch;
    }
  }
}

// =============================================================================
// الحلقة البرمجية الرئيسية
// =============================================================================
void loop() {
  handleSerialCommands();
  if (wifiEnabled) {
    dnsServer.processNextRequest();
    server.handleClient();
  }
  processWifiRequest();
  commitSettingsIfDue();

  if (rebootRequested && (int32_t)(millis() - rebootAt) >= 0) {
    if (nvsDirty) {
      lastNvsChangeTime = 0;
      commitSettingsIfDue();
    }
    ESP.restart();
  }

  static uint32_t lastStatus = 0;
  if (millis() - lastStatus >= 60000) {
    lastStatus = millis();
    printStatusLine();
  }

  vTaskDelay(pdMS_TO_TICKS(5));
}
