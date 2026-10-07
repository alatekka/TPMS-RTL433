// =============================================================================
//  TPMS_Unified_FRAM.ino — نظام مراقبة ضغط الإطارات الاحترافي المتكامل V3.3
//  - أداء فائق في بيئات الزحام مع ترشيد الذاكرة والفلترة الاستباقية للـ JSON
//  - طابور راديو محسن ومستودع نبضات موسع (Zero-Drop Pipeline)
//  - خادم ويب بنظام البث المباشر مع استقرار عتادي كامل للشاشة (40 MHz)
// =============================================================================

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <HTTPUpdateServer.h>
#include <DNSServer.h>
#include "esp_ota_ops.h"
#include <ArduinoJson.h>
#include <RTClib.h>
#include "esp_pm.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_idf_version.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

// ---------------- 1. هياكل البيانات ومعمارية FRAM ----------------
struct TireThreshold { 
  float pMin, pMax, tMin, tMax; 
};

struct SensorBinding {
  char shortId[12];
  char label[8];
};

#define FRAM_CFG_MAGIC 0xC205u
struct __attribute__((packed)) FramConfig {
  uint16_t      magic;
  int32_t       rssiDeltaCurrent;
  int32_t       wifiPowerIndex;
  char          apSSID[33];
  char          apPassword[65];
  TireThreshold tireThresh[4];
  uint32_t      scannerDecodersMask;
  int32_t       cpuFreqMhz;
  int32_t       freqDevKhz;
  SensorBinding tires[5];
  uint32_t      staleSignalTimeoutSec; // مهلة كشف انقطاع إشارة الحساس (ثوانٍ) — أُخذت من reserved[] دون تغيير الحجم الكلي
  // الحقول التالية أُخذت من reserved[12] السابقة دون تغيير الحجم الكلي (FRAM قديم = أصفار)
  uint8_t       cfgVersion;            // 0 = إعدادات من النسخ القديمة
  uint8_t       rxBwIndex;             // 0 = الافتراضي، وإلا (فهرس CC1101_BW_LIST + 1)
  int16_t       freqOffsetKhz;         // معايرة انحراف كريستال CC1101
  int8_t        techThresholdDbm;      // عتبة القرب لوضع فني الإطارات (0 = الافتراضي)
  uint8_t       unitPressure;          // 0 بار، 1 PSI، 2 kPa
  uint8_t       unitTemp;              // 0 مئوية، 1 فهرنهايت
  uint8_t       buzzerOff;             // 0 = الطنان مفعّل (FRAM قديم = أصفار = مفعّل)
  uint8_t       activeProfile;         // ملف السيارة النشط 0..3
  uint8_t       autoSensorsMode;       // 0 = الافتراضي (تأكيد بالزر)، 1 معطّل، 2 تأكيد بالزر، 3 تلقائي
  uint8_t       baselineOff;           // 0 = تعلّم الضغط الطبيعي مفعّل
  uint8_t       autoTuneOff;           // 0 = المعايرة الذاتية للراديو مفعّلة
};
static_assert(sizeof(FramConfig) == 2 + 4 + 4 + 33 + 65 + 64 + 4 + 4 + 4 + 5 * 20 + 4 + 12,
              "FramConfig size must not change (FRAM layout)");

// ملفات السيارات: حتى 4 سيارات (معرّفات الحساسات + حدود الإنذار لكل سيارة)
#define PROFILE_MAGIC 0xB17Eu
#define PROFILE_COUNT 4
struct __attribute__((packed)) VehicleProfile {
  uint16_t      magic;
  char          name[24];
  SensorBinding tires[5];
  TireThreshold thresh[4];
};

struct __attribute__((packed)) FramTireRecord {
  uint8_t  hasData;
  float    bar;
  float    psi;
  float    temp;
  int32_t  status;
  uint8_t  batteryOk;
  uint8_t  isMoving;
  uint8_t  isLeaking;
  uint32_t lastSeenEpoch;
  uint32_t packetCount;
};

// تغيّر عنوان بيانات السجل (أُضيفت ملفات السيارات قبله) — رقم سحري جديد يعيد تهيئة السجل مرة واحدة
#define FRAM_LOG_MAGIC 0xA5C3D2E8UL   // نهاية السجل صارت قبل منطقة الحساسات الأصلية
struct __attribute__((packed)) LogRecord {
  uint32_t timestamp;
  uint8_t  tireIdx;
  uint8_t  pressureRaw;
  int8_t   tempC;
  int8_t   rssi;
  uint8_t  status;
  uint8_t  batteryOk;
  uint8_t  motionAndLeak;
};

struct __attribute__((packed)) LogHeader {
  uint32_t magic;
  uint32_t writeIndex;
  uint32_t count;
};

struct TireState {
  bool     hasData;
  float    bar;
  float    psi;
  float    temp;
  int      status;
  bool     batteryOk;
  uint8_t  lowBattStreak;
  bool     isMoving;
  bool     isLeaking;
  int      lvl;
  int      rssi;
  uint32_t packetCount;
  uint32_t lastSeenEpoch;
  float    prevBar;
  uint32_t prevBarMs;
};

struct ScannedSensor {
  char     id[24];
  char     model[48];
  float    bar;
  float    temp;
  int      rssi;
  int      status;
  bool     batteryOk;
  bool     isMoving;
  bool     isLeaking;
  uint32_t packetCount;
  uint32_t lastSeenEpoch;
  bool     isMine;
  bool     isToyota;
};

// رفعنا بافر حزمة الراديو من 256 إلى 384 بايت لتوفير هامش أكبر لرسائل
// JSON الأطول القادمة من بعض ديكودرات TPMS (موديلات ذات حقول إضافية) دون
// اقتطاعها، وليتطابق مع حجم messageBuffer أدناه ومع _bufferSize الممرر
// إلى rf.setCallback(). التكلفة: طابور rfQueue من 32 عنصراً × 128 بايت
// زيادة لكل عنصر ≈ 4KB إضافية من RAM فقط — أقل من 1.2% من ذاكرة ESP32
// الداخلية الحرة (>320KB)، فلا خطر عملي لنفاد الذاكرة (Heap Exhaustion).
struct TechCapture {
  bool     valid;
  char     id[24];
  char     model[24];
  float    bar;        // NAN إن لم يرسل الحساس الضغط
  float    temp;
  int      rssi;
  int      bestRssi;
  int      status;
  int8_t   battery;    // -1 غير معروف، 0 ضعيفة، 1 سليمة
  int8_t   tire;       // رقم إطار سيارتي إن تطابق، وإلا -1
  char     mic[10];
  uint32_t lastSeenMs;
  uint32_t count;
  uint32_t epoch;      // وقت أول التقاط (RTC) لتقرير الفحص
};

struct LearnCandidate {
  char     id[24];
  char     model[24];
  uint16_t packets;
  uint16_t bursts;
  int32_t  rssiSum;
  int      bestRssi;
  uint32_t lastMs;
  float    bar;
  float    temp;
};

// ---------------- بيانات التعلّم الذاتي ----------------
#define SMART_MAX_SENSORS 16
#define SMART_MAGIC       0x5A17u
struct __attribute__((packed)) SmartHeader {
  uint16_t magic;
  uint16_t sessionCounter;   // يزيد مع كل إقلاع (قيادة) وكل ساعتين تشغيل متواصل
  uint16_t activeSessions;   // جلسات ظهر فيها حساس واحد مؤهَّل على الأقل (السيارة تتحرك)
  uint16_t reserved;
};
struct __attribute__((packed)) SmartSensor {
  char     id[12];
  char     model[10];
  uint16_t qualSessions;     // عدد الجلسات التي تأهّل فيها (3 دفعات + إشارة قوية)
  uint16_t lastQualSession;  // لمنع العدّ مرتين في نفس الجلسة
  uint16_t lastQualActive;   // رقم الجلسة النشطة لآخر تأهّل (لكشف الحساس المختفي)
  uint16_t activeAtFirst;    // رقم الجلسة النشطة عند أول تأهّل (لحساب النسبة)
};
struct __attribute__((packed)) TireBaseline {
  float    coldBar;          // الضغط البارد الطبيعي المتعلَّم
  uint32_t samples;
  uint32_t lastSampleEpoch;
  uint32_t firstEpoch;
};
struct SmartSessionCand {
  char     id[12];
  uint16_t bursts;
  uint16_t packets;
  int32_t  rssiSum;
  uint32_t lastMs;
  bool     qualified;
};
struct SmartProposal {
  bool     valid;
  uint8_t  count;
  int8_t   slot[5];
  char     id[5][12];
  uint32_t createdMs;
};

// الحساسات الأصلية لسيارة المالك: مرجع ثابت يمكن الرجوع إليه دائماً
#define ORIG_MAGIC 0x0A1Bu
struct __attribute__((packed)) OriginalSet {
  uint16_t magic;
  char     ids[5][12];   // بالترتيب: FL, FR, RL, RR, SPARE
};

struct RawRfPacket {
  char text[384];
  int  rssi;
};

// ---------------- 1.5 الوحدات والضغط البارد ----------------
int unitPressure = 0;   // 0 بار، 1 PSI، 2 kPa
int unitTemp = 0;       // 0 مئوية، 1 فهرنهايت

static inline float pToUnit(float bar) {
  return unitPressure == 1 ? bar * 14.5038f : unitPressure == 2 ? bar * 100.0f : bar;
}
static inline const char* pUnit()   { return unitPressure == 1 ? "PSI" : unitPressure == 2 ? "kPa" : "BAR"; }
static inline const char* pUnitAr() { return unitPressure == 1 ? "PSI" : unitPressure == 2 ? "kPa" : "بار"; }
static inline int pDecimals()       { return unitPressure == 0 ? 2 : unitPressure == 1 ? 1 : 0; }
static inline float tToUnit(float c) { return unitTemp == 1 ? c * 9.0f / 5.0f + 32.0f : c; }
static inline const char* tUnit()   { return unitTemp == 1 ? "F" : "C"; }

// الضغط البارد: ما يعادل الضغط الحالي عند 20°C (قانون الغاز المثالي على الضغط المطلق).
// الضغط يرتفع مع حرارة الإطار أثناء القيادة؛ هذه القيمة تُظهر هل يحتاج الإطار
// نفخاً فعلاً أم أن الارتفاع من الحرارة فقط.
static inline float coldPressureBar(float bar, float tempC) {
  if (isnan(bar) || isnan(tempC) || tempC < -40.0f || tempC > 150.0f) return NAN;
  const float PATM = 1.01325f;
  return (bar + PATM) * (293.15f / (tempC + 273.15f)) - PATM;
}

// ---------------- 2. منافذ الشاشة ST7789 وقفل الحماية ----------------
#define TFT_CS    27
#define TFT_DC    16
#define TFT_RST   17
#define TFT_SCK   18
#define TFT_MOSI  23
#define TFT_MISO  -1
#define SCREEN_WIDTH  240
// لون مميز لحالة "لا توجد إشارة حديثة" — يجب تعريفه هنا (قبل أي دالة رسم)
// لأنه يُستخدم في safeTftRenderTireText() المُعرَّفة لاحقاً في الملف
#define COLOR_STALE 0xFD20
#define SCREEN_HEIGHT 320

SPIClass tftSPI(HSPI);
Adafruit_ST7789 tft = Adafruit_ST7789(&tftSPI, TFT_CS, TFT_DC, TFT_RST);
static SemaphoreHandle_t tftMutex = NULL;

static inline void safeTftFillScreen(uint16_t color) {
  if (tftMutex && xSemaphoreTake(tftMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    tft.fillScreen(color);
    xSemaphoreGive(tftMutex);
  }
}

static inline void safeTftFillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
  if (tftMutex && xSemaphoreTake(tftMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    tft.fillRect(x, y, w, h, color);
    xSemaphoreGive(tftMutex);
  }
}

static inline void safeTftDrawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color) {
  if (tftMutex && xSemaphoreTake(tftMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    tft.drawRoundRect(x, y, w, h, r, color);
    xSemaphoreGive(tftMutex);
  }
}

static inline void safeTftDrawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
  if (tftMutex && xSemaphoreTake(tftMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    tft.drawFastHLine(x, y, w, color);
    xSemaphoreGive(tftMutex);
  }
}

static inline void safeTftFillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color) {
  if (tftMutex && xSemaphoreTake(tftMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    tft.fillRoundRect(x, y, w, h, r, color);
    xSemaphoreGive(tftMutex);
  }
}

static inline void safeTftRenderTireText(int x, int y, const char* label, uint32_t pktCount, float bar, float temp, bool battOk, int lvl, bool isMoving, const char* ageBuf, int rssi, bool isStale) {
  if (tftMutex && xSemaphoreTake(tftMutex, pdMS_TO_TICKS(150)) == pdTRUE) {
    tft.fillRect(x, y - 2, 80, 68, ST77XX_BLACK);

    uint16_t color = ST77XX_GREEN;
    if (lvl == 1) color = ST77XX_RED;
    else if (lvl == 2) color = ST77XX_BLUE;
    else if (lvl == 3) color = ST77XX_MAGENTA;
    else if (lvl == 4) color = ST77XX_YELLOW;
    if (isStale) color = COLOR_STALE; // بيانات قديمة غير موثوقة تتفوّق على أي حالة أخرى معروضة

    tft.setTextSize(1);
    tft.setTextColor(0xAD55, ST77XX_BLACK);
    tft.setCursor(x, y);
    tft.printf("[%s]#%-3u", label, (unsigned int)pktCount);

    tft.setTextSize(3);
    tft.setTextColor(color, ST77XX_BLACK);
    tft.setCursor(x, y + 12);
    tft.printf("%.*f", unitPressure == 0 ? 2 : unitPressure == 1 ? 1 : 0, pToUnit(bar));

    tft.setTextSize(1);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setCursor(x, y + 38);
    tft.printf("%2.0f%s ", tToUnit(temp), tUnit());
    tft.setTextColor(battOk ? ST77XX_GREEN : ST77XX_RED, ST77XX_BLACK);
    tft.print(battOk ? "OK" : "LOW");

    tft.setCursor(x, y + 48);
    if (isStale) {
      tft.setTextColor(COLOR_STALE, ST77XX_BLACK);
      tft.print("NO SIGNAL!");
    } else if (lvl == 3) {
      tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
      tft.print("! LEAK !");
    } else if (lvl == 4) {
      tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
      tft.print("BELOW NORM");
    } else {
      tft.setTextColor(isMoving ? ST77XX_CYAN : 0x7BEF, ST77XX_BLACK);
      tft.print(isMoving ? "ROLLING" : "PARKED");
    }

    tft.setTextColor(isStale ? COLOR_STALE : ST77XX_YELLOW, ST77XX_BLACK);
    tft.setCursor(x, y + 58);
    tft.printf("%-4s %3ddB", ageBuf, rssi);

    xSemaphoreGive(tftMutex);
  }
}

// ---------------- 3. منافذ CC1101 المعزولة (VSPI) ----------------
#define CC1101_SCK   22
#define CC1101_MISO  21
#define CC1101_MOSI  25
#define CC1101_CSN   26
#define CC1101_GDO0  32
#define CC1101_GDO2  33

#define RF_MODULE_SCK   CC1101_SCK
#define RF_MODULE_MISO  CC1101_MISO
#define RF_MODULE_MOSI  CC1101_MOSI
#define RF_MODULE_CS    CC1101_CSN
#define RF_MODULE_GDO0  CC1101_GDO0
#define RF_MODULE_GDO2  CC1101_GDO2

#define RADIOLIB_LOW_LEVEL 1
#include <TPMS_RTL433.h>
// سعة مخزن الإشارات للزحمة. ESP32 بلا PSRAM والويب والشاشة يستهلكان الذاكرة،
// فالقيمة مختارة بحيث يبقى أدنى heap أثناء تحميل كل صفحات الويب فوق 25KB.
#define DEV_CAPTURE_SLOTS 5
#define DEV_DECODE_SLOTS  7

#define CC1101_FREQUENCY 433.92f
rtl_433_ESP rf;
char messageBuffer[384]; // يطابق حجم RawRfPacket.text أعلاه
QueueHandle_t rfQueue = NULL;

// ---------------- 4. منافذ I2C وذاكرة FRAM + RTC ----------------
#define I2C_SDA_PIN 4
#define I2C_SCL_PIN 19

#define FRAM_I2C_ADDR   0x50
#define FRAM_TOTAL_SIZE 32768UL

#define FRAM_CFG_ADDR         0x0000
#define FRAM_TIRES_ADDR       0x0150
#define FRAM_LOG_HEADER_ADDR  0x0250
#define FRAM_PROFILES_ADDR    0x0270   // 4 × 190 بايت = حتى 0x0568
#define FRAM_SMART_ADDR       0x0570   // جدول تعلّم الحساسات: 8 + 16 × 30 = 488 بايت (حتى 0x0758)
#define FRAM_BASELINE_ADDR    0x0760   // الضغط الطبيعي لكل إطار: 5 × 16 = 80 بايت (حتى 0x07B0)
#define FRAM_LOG_DATA_ADDR    0x07B0

#define FRAM_TIRE_RECORD_SIZE ((uint16_t)sizeof(FramTireRecord))
#define FRAM_LOG_RECORD_SIZE  ((uint16_t)sizeof(LogRecord))
#define FRAM_ORIG_ADDR        0x7FC0   // الحساسات الأصلية للسيارة (آخر 64 بايت من FRAM)
#define FRAM_LOG_MAX_RECORDS  ((uint32_t)((FRAM_ORIG_ADDR - FRAM_LOG_DATA_ADDR) / FRAM_LOG_RECORD_SIZE))

RTC_DS3231 rtc;
bool framOk = false;
bool rtcOk  = false;
static SemaphoreHandle_t i2cMutex = NULL;

static volatile uint32_t cachedBaseEpoch = 0;
static volatile unsigned long cachedSyncMillis = 0;

static inline bool i2cLock(uint32_t timeoutMs = 100) {
  if (!i2cMutex) return true;
  return xSemaphoreTake(i2cMutex, pdMS_TO_TICKS(timeoutMs)) == pdTRUE;
}

static inline void i2cUnlock() {
  if (i2cMutex) xSemaphoreGive(i2cMutex);
}

static bool framWrite(uint16_t addr, const uint8_t* data, size_t len) {
  if (!framOk || data == nullptr || len == 0) return false;
  if ((uint32_t)addr + (uint32_t)len > FRAM_TOTAL_SIZE) return false;
  if (!i2cLock()) return false;

  const size_t CHUNK = 30;
  size_t offset = 0;
  bool ok = true;
  while (offset < len) {
    size_t n = (len - offset < CHUNK) ? (len - offset) : CHUNK;
    uint16_t a = addr + (uint16_t)offset;
    Wire.beginTransmission(FRAM_I2C_ADDR);
    Wire.write((uint8_t)(a >> 8));
    Wire.write((uint8_t)(a & 0xFF));
    Wire.write(data + offset, n);
    if (Wire.endTransmission() != 0) { ok = false; break; }
    offset += n;
  }
  i2cUnlock();
  return ok;
}

static bool framRead(uint16_t addr, uint8_t* data, size_t len) {
  if (!framOk || data == nullptr || len == 0) return false;
  if ((uint32_t)addr + (uint32_t)len > FRAM_TOTAL_SIZE) return false;
  if (!i2cLock()) return false;

  const size_t CHUNK = 30;
  size_t offset = 0;
  bool ok = true;
  while (offset < len) {
    size_t n = (len - offset < CHUNK) ? (len - offset) : CHUNK;
    uint16_t a = addr + (uint16_t)offset;
    Wire.beginTransmission(FRAM_I2C_ADDR);
    Wire.write((uint8_t)(a >> 8));
    Wire.write((uint8_t)(a & 0xFF));
    if (Wire.endTransmission(false) != 0) { ok = false; break; }
    size_t got = Wire.requestFrom((uint8_t)FRAM_I2C_ADDR, (uint8_t)n);
    if (got != n) { ok = false; break; }
    for (size_t i = 0; i < n; i++) data[offset + i] = Wire.read();
    offset += n;
  }
  i2cUnlock();
  return ok;
}

static inline bool framPutConfig(uint16_t addr, const FramConfig& v) {
  return framWrite(addr, (const uint8_t*)&v, sizeof(v));
}
static inline bool framGetConfig(uint16_t addr, FramConfig& v) {
  return framRead(addr, (uint8_t*)&v, sizeof(v));
}
static inline bool framPutTireRecord(uint16_t addr, const FramTireRecord& v) {
  return framWrite(addr, (const uint8_t*)&v, sizeof(v));
}
static inline bool framGetTireRecord(uint16_t addr, FramTireRecord& v) {
  return framRead(addr, (uint8_t*)&v, sizeof(v));
}
static inline bool framPutLogRecord(uint16_t addr, const LogRecord& v) {
  return framWrite(addr, (const uint8_t*)&v, sizeof(v));
}
static inline bool framGetLogRecord(uint16_t addr, LogRecord& v) {
  return framRead(addr, (uint8_t*)&v, sizeof(v));
}
static inline bool framPutLogHeader(uint16_t addr, const LogHeader& v) {
  return framWrite(addr, (const uint8_t*)&v, sizeof(v));
}
static inline bool framGetLogHeader(uint16_t addr, LogHeader& v) {
  return framRead(addr, (uint8_t*)&v, sizeof(v));
}

LogHeader logHeader;

void resetLogHeader() {
  logHeader.magic = FRAM_LOG_MAGIC;
  logHeader.writeIndex = 0;
  logHeader.count = 0;
}

static bool framDetect() {
  if (!i2cLock()) return false;
  Wire.beginTransmission(FRAM_I2C_ADDR);
  uint8_t res = Wire.endTransmission();
  i2cUnlock();
  return res == 0;
}

uint32_t epochNow() {
  if (cachedBaseEpoch > 0) {
    return cachedBaseEpoch + ((millis() - cachedSyncMillis) / 1000UL);
  }
  return (uint32_t)(millis() / 1000UL);
}

void syncRtcTime() {
  if (rtcOk && i2cLock(100)) {
    cachedBaseEpoch = rtc.now().unixtime();
    cachedSyncMillis = millis();
    i2cUnlock();
  }
}

bool rtcSetEpoch(uint32_t epoch) {
  if (!rtcOk || !i2cLock(200)) return false;
  rtc.adjust(DateTime(epoch));
  cachedBaseEpoch = epoch;
  cachedSyncMillis = millis();
  i2cUnlock();
  return true;
}

bool rtcSetDateTime(int y, int mo, int d, int h, int mi, int s) {
  if (!rtcOk) return false;
  if (y < 2000 || y > 2099 || mo < 1 || mo > 12 || d < 1 || d > 31 ||
      h < 0 || h > 23 || mi < 0 || mi > 59 || s < 0 || s > 59) return false;
  if (!i2cLock(200)) return false;
  DateTime dt((uint16_t)y, (uint8_t)mo, (uint8_t)d, (uint8_t)h, (uint8_t)mi, (uint8_t)s);
  rtc.adjust(dt);
  cachedBaseEpoch = dt.unixtime();
  cachedSyncMillis = millis();
  i2cUnlock();
  return true;
}

// ---------------- 5. المتغيرات العامة وحالة النظام ----------------
#define BOOT_BUTTON_PIN 0
bool wifiEnabled = false;
bool topBarDirty = true;
volatile bool tireDataDirty = false;
volatile bool statusBarDirty = true;
uint32_t rfPacketCounter = 0;
volatile uint32_t rfQueueDropCounter = 0;
bool isCC1101Ok = false;
int  cc1101ErrCode = 0;
volatile int screenMode = 0;
int  lastDrawnScreenMode = -1;

String apSSID     = "TPMS-Setup";
String apPassword = "";
String deviceIPString = "192.168.4.1";
WebServer server(80);
HTTPUpdateServer httpUpdater;
// خادم DNS للبوابة التلقائية: كل الأسماء تُوجَّه للجهاز، فيفتح الهاتف الصفحة تلقائياً
// بدل اعتبار الشبكة "بلا إنترنت" وتمرير الطلبات عبر بيانات الجوال.
DNSServer dnsServer;

// ---------------- التحديث اللاسلكي مع الرجوع التلقائي ----------------
// نخبر نواة Arduino ألا تعتمد النسخة الجديدة فوراً عند الإقلاع: تبقى "تحت
// الاختبار" حتى ينجح فحص ذاتي (الراديو يعمل + 60 ثانية تشغيل مستقر). إن
// انهارت أو أعيد تشغيلها قبل ذلك، يعيد محمّل الإقلاع النسخة السابقة تلقائياً.
extern "C" {
bool verifyRollbackLater() { return true; }
}
static bool otaImageConfirmed = false;

static void confirmOtaImageIfHealthy(bool radioOk) {
  if (otaImageConfirmed || !radioOk || millis() < 60000UL) return;
  otaImageConfirmed = true;
  const esp_partition_t* running = esp_ota_get_running_partition();
  esp_ota_img_states_t state;
  if (running && esp_ota_get_state_partition(running, &state) == ESP_OK && state == ESP_OTA_IMG_PENDING_VERIFY) {
    esp_ota_mark_app_valid_cancel_rollback();
    Serial.println("[OTA] new firmware passed self-test and is now confirmed");
  } else {
    esp_ota_mark_app_valid_cancel_rollback();
  }
}

const wifi_power_t WIFI_POWER_LEVELS[] = {
  WIFI_POWER_19_5dBm, WIFI_POWER_19dBm, WIFI_POWER_18_5dBm, WIFI_POWER_17dBm,
  WIFI_POWER_15dBm,   WIFI_POWER_13dBm, WIFI_POWER_11dBm,   WIFI_POWER_8_5dBm,
  WIFI_POWER_7dBm,    WIFI_POWER_5dBm,  WIFI_POWER_2dBm
};
const float WIFI_POWER_DBM[] = {
  19.5f, 19.0f, 18.5f, 17.0f, 15.0f, 13.0f, 11.0f, 8.5f, 7.0f, 5.0f, 2.0f
};
const int WIFI_POWER_LEVELS_COUNT = sizeof(WIFI_POWER_LEVELS) / sizeof(WIFI_POWER_LEVELS[0]);
int wifiPowerIndex = 0;

// أضعف قوة مسموحة 8.5 dBm (الفهرس 7): القيم الأقل (7/5/2 dBm) تجعل الهاتف يتصل
// أحياناً لكن الصفحات لا تُحمَّل إلا من مسافة قريبة جداً.
#define WIFI_POWER_MIN_INDEX 7
int clampWifiPowerIndex(int v) {
  if (v < 0) return 0;
  if (v > WIFI_POWER_MIN_INDEX) return WIFI_POWER_MIN_INDEX;
  return v;
}

void applyWifiPower() {
  WiFi.setTxPower(WIFI_POWER_LEVELS[wifiPowerIndex]);
}

void startWifiAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(apSSID.c_str(), apPassword.c_str());
  applyWifiPower();
  deviceIPString = WiFi.softAPIP().toString();
  dnsServer.start(53, "*", WiFi.softAPIP());
  if (MDNS.begin("tpms")) {
    Serial.println("[mDNS] http://tpms.local");
  }
  server.begin();
  wifiEnabled = true;
  topBarDirty = true;
}

void stopWifiAP() {
  dnsServer.stop();
  server.stop();
  MDNS.end();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  wifiEnabled = false;
  topBarDirty = true;
}

void toggleWifiAP() {
  if (wifiEnabled) stopWifiAP();
  else startWifiAP();
}

TireThreshold tireThresh[4] = {
  {1.8f, 3.0f, 0.0f, 60.0f}, {1.8f, 3.0f, 0.0f, 60.0f},
  {1.8f, 3.0f, 0.0f, 60.0f}, {1.8f, 3.0f, 0.0f, 60.0f},
};
const float P_STEP = 0.05f, P_ABS_MIN = 1.0f, P_ABS_MAX = 4.5f;
const float T_STEP = 1.0f,  T_ABS_MIN = -20.0f, T_ABS_MAX = 90.0f;
const float P_MIN_GAP = P_STEP, T_MIN_GAP = T_STEP;
const char* THR_FIELD_LABEL[4] = {"P MIN (Bar)", "P MAX (Bar)", "T MIN (C)", "T MAX (C)"};

// ==== إصلاح أمان جوهري: كشف انقطاع إشارة حساس (Stale Signal Detection) ====
// كانت المشكلة: بمجرد وصول أول حزمة لأي إطار، يُضبط hasData=true بشكل دائم
// ولا يُعاد ضبطه أبداً مهما طال انقطاع الإشارة بعدها (بطارية ميتة، هوائي
// تالف، تشويش/تشويش متعمد، أو استبدال الإطار دون نقل الحساس). النتيجة:
// تستمر الشاشة بعرض آخر قراءة صالحة (وقد تكون "OK" خضراء) إلى ما لا نهاية
// دون أي تمييز بصري أنها بيانات قديمة لم تعد موثوقة — وهذا خطر حقيقي في
// نظام يُعتمد عليه للسلامة: قد يظن السائق أن الإطار سليم بينما حساسه توقف
// عن الإرسال فعلياً منذ ساعات. الإصلاح هنا في طبقة العرض فقط (لا يغيّر
// تنسيق التخزين في FRAM): إن تجاوز عمر آخر إشارة هذه المهلة تُعرض حالة
// "NO SIGNAL" بلون مميز بغض النظر عن آخر قراءة صالحة كانت.
// 1200 ثانية (20 دقيقة) قيمة افتراضية متحفظة لتفادي إنذارات كاذبة أثناء
// الوقوف الطويل (حساسات TPMS ترسل نادراً جداً وهي متوقفة لتوفير البطارية)،
// مع كونها كافية لكشف حساس معطوب فعلياً خلال قيادة عادية. عدّلها حسب حاجتك.
uint32_t staleSignalTimeoutSec = 1200UL;

uint32_t scannerDecodersMask = 0x01FFFFFFUL;

// عروض النطاق المتاحة فعلياً في CC1101 (كريستال 26MHz)
const float CC1101_BW_LIST[] = {58, 68, 81, 102, 116, 135, 162, 203, 232, 270, 325, 406, 464, 541, 650, 812};
const int CC1101_BW_COUNT = sizeof(CC1101_BW_LIST) / sizeof(CC1101_BW_LIST[0]);
const int CC1101_BW_DEFAULT_INDEX = 3;       // 102 kHz: الأفضل في الاختبار الميداني
int rxBwIndex = CC1101_BW_DEFAULT_INDEX;
int freqOffsetKhz = 0;
int techThresholdDbm = -70;
#define CFG_VERSION_CURRENT 3
#define WIFI_POWER_DEFAULT_INDEX 6   // 11 dBm: يكفي للهاتف ويقلل تشويش الواي فاي على CC1101 (رفع الضجيج 4 dB عند 19.5 dBm)

float currentRxBwKhz() { return CC1101_BW_LIST[rxBwIndex]; }
float currentFreqMhz() { return CC1101_FREQUENCY + freqOffsetKhz / 1000.0f; }

extern portMUX_TYPE tireDataMux;   // مُعرَّف لاحقاً مع بيانات الإطارات

// ---------------- الإنذار الصوتي والضوئي ----------------
// طنان نشط (Active Buzzer) على GPIO13 (يفضَّل عبر ترانزستور)، ولمبة اللوحة على GPIO2.
#define BUZZER_PIN      13
#define STATUS_LED_PIN  2
bool buzzerEnabled = true;
int  activeProfile = 0;
volatile bool alarmActive = false;     // يوجد إطار في حالة خطر الآن
volatile bool alarmAcked = false;      // ضغط المستخدم الزر لإسكات الإنذار
volatile uint8_t pendingChirps = 0;    // طلب صفير قصير من مهام أخرى (التقاط فني/انتهاء التعلّم)
static uint8_t  beepRemaining = 0;
static uint16_t beepOnMs = 0, beepOffMs = 0;
static bool     beepOutput = false;
static uint32_t beepNextMs = 0;

void beep(uint8_t count, uint16_t onMs, uint16_t offMs) {
  if (!buzzerEnabled) return;
  beepRemaining = count;
  beepOnMs = onMs;
  beepOffMs = offMs;
  beepOutput = false;
  beepNextMs = millis();
}

// تُستدعى من loop(): تشغيل نمط الصفير بلا حجب، ولمبة الحالة
void serviceBuzzerAndLed() {
  uint32_t now = millis();
  if (pendingChirps > 0 && beepRemaining == 0 && !beepOutput) {
    uint8_t n = pendingChirps;
    pendingChirps = 0;
    beep(n, 70, 90);
  }
  if ((int32_t)(now - beepNextMs) >= 0) {
    if (beepOutput) {
      digitalWrite(BUZZER_PIN, LOW);
      beepOutput = false;
      beepNextMs = now + beepOffMs;
    } else if (beepRemaining > 0) {
      digitalWrite(BUZZER_PIN, HIGH);
      beepOutput = true;
      beepRemaining--;
      beepNextMs = now + beepOnMs;
    }
  }
  // اللمبة: وميض سريع = إنذار غير مُسكَت، ثابتة = إنذار مُسكَت، مطفأة = لا إنذار
  bool led = alarmActive && (alarmAcked || ((now / 250) % 2 == 0));
  digitalWrite(STATUS_LED_PIN, led ? HIGH : LOW);
}

// ---------------- تعلّم إطارات السيارة تلقائياً ----------------
// أثناء القيادة ترسل حساسات سيارتك مراراً بإشارة قوية ثابتة، بينما السيارات
// المارّة تظهر مرة أو مرتين. نجمع الحساسات خلال 3 دقائق ونرتّبها حسب عدد
// "الدفعات" المنفصلة زمنياً ثم قوة الإشارة.
#define LEARN_DURATION_MS   180000UL
#define LEARN_MAX_CAND      24
#define LEARN_BURST_GAP_MS  8000UL

LearnCandidate learnCand[LEARN_MAX_CAND];
int learnCandCount = 0;
volatile bool learnActive = false;
volatile bool learnDone = false;
uint32_t learnStartMs = 0;

// تُستدعى داخل قفل tireDataMux
static void learnAdd(const char* id, const char* model, int rssi, float bar, float temp, uint32_t nowMs) {
  int idx = -1;
  for (int i = 0; i < learnCandCount; i++) if (strcmp(learnCand[i].id, id) == 0) { idx = i; break; }
  if (idx < 0) {
    if (learnCandCount < LEARN_MAX_CAND) idx = learnCandCount++;
    else {
      // استبدال أضعف مرشح (أقل دفعات) بالحساس الجديد
      idx = 0;
      for (int i = 1; i < learnCandCount; i++) if (learnCand[i].bursts < learnCand[idx].bursts) idx = i;
      if (learnCand[idx].bursts > 1) return;
    }
    memset(&learnCand[idx], 0, sizeof(LearnCandidate));
    strncpy(learnCand[idx].id, id, sizeof(learnCand[idx].id) - 1);
    strncpy(learnCand[idx].model, model ? model : "TPMS", sizeof(learnCand[idx].model) - 1);
    learnCand[idx].bestRssi = rssi;
  }
  LearnCandidate& c = learnCand[idx];
  if (c.packets == 0 || nowMs - c.lastMs >= LEARN_BURST_GAP_MS) c.bursts++;
  c.packets++;
  c.rssiSum += rssi;
  if (rssi > c.bestRssi) c.bestRssi = rssi;
  c.lastMs = nowMs;
  c.bar = bar;
  c.temp = temp;
}

static int learnScore(const LearnCandidate& c) {
  int avg = c.packets ? (int)(c.rssiSum / (int32_t)c.packets) : -120;
  return c.bursts * 100 + (avg + 120);
}

// ترتيب تنازلي حسب الدرجة (يُستدعى داخل القفل)
static void learnSortLocked() {
  for (int i = 1; i < learnCandCount; i++) {
    LearnCandidate key = learnCand[i];
    int j = i - 1;
    while (j >= 0 && learnScore(learnCand[j]) < learnScore(key)) { learnCand[j + 1] = learnCand[j]; j--; }
    learnCand[j + 1] = key;
  }
}

// ---------------- مفاتيح التعلّم الذاتي ----------------
int  autoSensorsMode = 2;        // 1 معطّل، 2 تأكيد بالزر، 3 تلقائي
bool baselineEnabled = true;
bool autoTuneEnabled = true;

// المعايرة الذاتية للتردد: مقارنة متناوبة بين الإزاحة الحالية وإزاحة مجاورة
#define TUNE_INTERVAL_MS   (3UL * 3600UL * 1000UL)
#define TUNE_FIRST_MS      (20UL * 60UL * 1000UL)
#define TUNE_SLICE_MS      120000UL
#define TUNE_SLICES        10
#define TUNE_STEP_KHZ      5
volatile bool tuneRunning = false;
uint32_t tuneNextMs = TUNE_FIRST_MS;
int      tuneDir = 1;
String   tuneLastResult = "لم تُجرَ بعد";

// إحصائيات وتشخيص للمنفذ التسلسلي
volatile uint32_t rfRejectedCounter = 0;
volatile uint32_t ccRecoveryCounter = 0;
volatile bool serialPacketLog = true;

// آخر حزمة مفكوكة (لشاشة تشخيص الراديو)
char lastPktId[24] = "";
int  lastPktRssi = 0;
uint32_t lastPktMs = 0;

void syncDecoderEngineWithScreenMode() {
  if (screenMode == 0) {
    rtl_433_ESP::applyToyotaOnlyDecoders();
  } else if (screenMode == 2 || learnActive) {
    rtl_433_ESP::applyTpmsMask(0x01FFFFFFUL); // الفني/التعلّم: كل البروتوكولات
  } else {
    rtl_433_ESP::applyTpmsMask(scannerDecodersMask);
  }
}

void learnStart() {
  portENTER_CRITICAL(&tireDataMux);
  learnCandCount = 0;
  learnDone = false;
  learnActive = true;
  portEXIT_CRITICAL(&tireDataMux);
  learnStartMs = millis();
  rtl_433_ESP::applyTpmsMask(0x01FFFFFFUL); // كل البروتوكولات أثناء التعلّم
  Serial.println("[LEARN] started: drive the car for 3 minutes");
}

void learnFinish() {
  portENTER_CRITICAL(&tireDataMux);
  learnActive = false;
  learnDone = true;
  learnSortLocked();
  portEXIT_CRITICAL(&tireDataMux);
  syncDecoderEngineWithScreenMode();
  pendingChirps = 2;
  Serial.printf("[LEARN] finished: %d candidates\n", learnCandCount);
  for (int i = 0; i < learnCandCount && i < 8; i++) {
    LearnCandidate& c = learnCand[i];
    Serial.printf("[LEARN] #%d %s %s bursts=%u packets=%u avg=%d best=%d\n", i + 1, c.id, c.model,
                  c.bursts, c.packets, c.packets ? (int)(c.rssiSum / (int32_t)c.packets) : 0, c.bestRssi);
  }
}

const int RSSI_DELTA_MIN = 3;
const int RSSI_DELTA_MAX = 30;
const int RSSI_DELTA_STEP = 1;
int rssiDeltaCurrent = 18;

const int FREQ_DEV_MIN = 10;
const int FREQ_DEV_MAX = 100;
const int FREQ_DEV_STEP = 5;
int freqDevCurrent = 40;

int clampFreqDev(int v) {
  if (v < FREQ_DEV_MIN) return FREQ_DEV_MIN;
  if (v > FREQ_DEV_MAX) return FREQ_DEV_MAX;
  return v;
}

int cpuFreqCurrent = 240;

int clampCpuFreq(int f) {
  if (f == 80 || f == 160 || f == 240) return f;
  return 240;
}

int clampRssiDelta(int v) {
  if (v < RSSI_DELTA_MIN) return RSSI_DELTA_MIN;
  if (v > RSSI_DELTA_MAX) return RSSI_DELTA_MAX;
  return v;
}

// حدود معقولة لمهلة كشف انقطاع إشارة الحساس: لا تقل عن دقيقتين (تفادي
// إنذارات كاذبة من فجوات إرسال طبيعية قصيرة)، ولا تزيد عن 6 ساعات (تفادي
// عدم اكتشاف حساس معطوب فعلياً لوقت طويل جداً). تُستخدم أيضاً لترقيع FRAM
// قديم من قبل إضافة هذا الحقل (كانت بايتاته السابقة جزءاً من reserved[]
// المصفَّرة غالباً = 0، وهي قيمة خطيرة هنا لأنها تعني "كل شيء قديم دائماً").
#define STALE_TIMEOUT_MIN_SEC 120UL
#define STALE_TIMEOUT_MAX_SEC 21600UL
uint32_t clampStaleTimeout(uint32_t v) {
  if (v < STALE_TIMEOUT_MIN_SEC || v > STALE_TIMEOUT_MAX_SEC) return 1200UL;
  return v;
}

FramConfig cfg;
SensorBinding myTires[5];

void saveConfigToFram();

void applyAndSaveFreqDev(int newDev) {
  freqDevCurrent = clampFreqDev(newDev);
  if (isCC1101Ok) {
    rf.setFrequencyDeviation((float)freqDevCurrent);
  }
  saveConfigToFram();
  statusBarDirty = true;
}

void applyAndSaveCpuFreq(int newFreq) {
  cpuFreqCurrent = clampCpuFreq(newFreq);
  setCpuFrequencyMhz(cpuFreqCurrent);
  saveConfigToFram();
}

void setDefaultConfig() {
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic = FRAM_CFG_MAGIC;
  cfg.rssiDeltaCurrent = 18;
  cfg.wifiPowerIndex = WIFI_POWER_DEFAULT_INDEX;
  cfg.scannerDecodersMask = 0x01FFFFFFUL;
  cfg.cpuFreqMhz = 240;
  cfg.freqDevKhz = 40;
  cfg.staleSignalTimeoutSec = 1200UL;
  cfg.cfgVersion = CFG_VERSION_CURRENT;
  cfg.rssiDeltaCurrent = 9;
  cfg.rxBwIndex = CC1101_BW_DEFAULT_INDEX + 1;
  cfg.freqOffsetKhz = 0;
  cfg.techThresholdDbm = -70;
  strncpy(cfg.apSSID, "TPMS-Setup", sizeof(cfg.apSSID) - 1);
  strncpy(cfg.apPassword, "12345678", sizeof(cfg.apPassword) - 1);

  for (int i = 0; i < 4; i++) cfg.tireThresh[i] = TireThreshold{1.8f, 3.0f, 0.0f, 60.0f};

  strncpy(cfg.tires[0].shortId, "78D8483", sizeof(cfg.tires[0].shortId) - 1);
  strncpy(cfg.tires[0].label,   "FL",      sizeof(cfg.tires[0].label) - 1);
  strncpy(cfg.tires[1].shortId, "78D8C0A", sizeof(cfg.tires[1].shortId) - 1);
  strncpy(cfg.tires[1].label,   "FR",      sizeof(cfg.tires[1].label) - 1);
  strncpy(cfg.tires[2].shortId, "78D8CD9", sizeof(cfg.tires[2].shortId) - 1);
  strncpy(cfg.tires[2].label,   "RL",      sizeof(cfg.tires[2].label) - 1);
  strncpy(cfg.tires[3].shortId, "78D8C17", sizeof(cfg.tires[3].shortId) - 1);
  strncpy(cfg.tires[3].label,   "RR",      sizeof(cfg.tires[3].label) - 1);
  strncpy(cfg.tires[4].shortId, "78D8409", sizeof(cfg.tires[4].shortId) - 1);
  strncpy(cfg.tires[4].label,   "SPARE",   sizeof(cfg.tires[4].label) - 1);
}

void loadConfigFromFram() {
  bool ok = framOk && framGetConfig(FRAM_CFG_ADDR, cfg);
  if (!ok || cfg.magic != FRAM_CFG_MAGIC) {
    setDefaultConfig();
    if (framOk) {
      framPutConfig(FRAM_CFG_ADDR, cfg);
      for (int i = 0; i < 5; i++) {
        FramTireRecord rec{};
        rec.hasData = 0;
        rec.batteryOk = 1;
        framPutTireRecord((uint16_t)(FRAM_TIRES_ADDR + i * FRAM_TIRE_RECORD_SIZE), rec);
      }
      resetLogHeader();
      framPutLogHeader(FRAM_LOG_HEADER_ADDR, logHeader);
    }
  }
  rssiDeltaCurrent    = clampRssiDelta(cfg.rssiDeltaCurrent);
  wifiPowerIndex      = clampWifiPowerIndex(cfg.wifiPowerIndex);
  scannerDecodersMask = cfg.scannerDecodersMask;
  cpuFreqCurrent      = clampCpuFreq(cfg.cpuFreqMhz);
  freqDevCurrent      = clampFreqDev(cfg.freqDevKhz);
  // clampStaleTimeout يعيد القيمة الافتراضية (1200) تلقائياً إن كانت القيمة
  // المحمَّلة 0 أو غير منطقية — وهذا بالضبط حال أي FRAM محفوظ قبل إضافة هذا
  // الحقل (كانت بايتاته السابقة جزءاً من reserved[] المصفَّرة عادة)، فتُمنع
  // بذلك حالة خطيرة: قيمة 0 كانت ستجعل النظام يعتبر كل الإطارات "منقطعة
  // الإشارة" فوراً بعد أول تحديث للكود.
  staleSignalTimeoutSec = clampStaleTimeout(cfg.staleSignalTimeoutSec);
  // ترقية إعدادات النسخ القديمة: عتبة 18dB كانت تحجب الإشارات البعيدة، وعرض
  // النطاق كان ثابتاً على 270 kHz (أضعف حساسية في القياس الميداني).
  if (cfg.cfgVersion < 3 && wifiPowerIndex == 0) {
    wifiPowerIndex = WIFI_POWER_DEFAULT_INDEX; // كان أقصى قوة افتراضياً
  }
  if (cfg.cfgVersion < 2) {
    rssiDeltaCurrent = 9;
    cfg.rxBwIndex = CC1101_BW_DEFAULT_INDEX + 1;
    cfg.freqOffsetKhz = 0;
    cfg.techThresholdDbm = -70;
  }
  cfg.cfgVersion = CFG_VERSION_CURRENT;
  rxBwIndex = (cfg.rxBwIndex >= 1 && cfg.rxBwIndex <= CC1101_BW_COUNT) ? cfg.rxBwIndex - 1 : CC1101_BW_DEFAULT_INDEX;
  freqOffsetKhz = constrain((int)cfg.freqOffsetKhz, -150, 150);
  techThresholdDbm = (cfg.techThresholdDbm <= -20 && cfg.techThresholdDbm >= -100) ? cfg.techThresholdDbm : -70;
  unitPressure  = cfg.unitPressure <= 2 ? cfg.unitPressure : 0;
  unitTemp      = cfg.unitTemp <= 1 ? cfg.unitTemp : 0;
  buzzerEnabled = cfg.buzzerOff == 0;
  activeProfile = cfg.activeProfile <= 3 ? cfg.activeProfile : 0;
  autoSensorsMode = (cfg.autoSensorsMode >= 1 && cfg.autoSensorsMode <= 3) ? cfg.autoSensorsMode : 2;
  baselineEnabled = cfg.baselineOff == 0;
  autoTuneEnabled = cfg.autoTuneOff == 0;
  apSSID     = String(cfg.apSSID);
  apPassword = String(cfg.apPassword);
  for (int i = 0; i < 4; i++) tireThresh[i] = cfg.tireThresh[i];
  for (int i = 0; i < 5; i++) {
    myTires[i] = cfg.tires[i];
    // شفاء تلقائي: معرّفات محفوظة في FRAM قبل إصلاح توحيد حالة الأحرف قد تكون
    // بحروف صغيرة، مما يمنع نجاح المطابقة الدقيقة في storageTask() (idBuf
    // هناك دائمًا حروف كبيرة). نوحّدها هنا عند كل إقلاع بدل الاعتماد على أن
    // يعيد المستخدم حفظ صفحة الإعدادات يدويًا بعد تحديث الكود.
    for (char* p = myTires[i].shortId; *p; ++p) *p = (char)toupper((unsigned char)*p);
  }

  setCpuFrequencyMhz(cpuFreqCurrent);
}

// ---------------- ملفات السيارات ----------------
extern volatile bool tirePersistDirty[5];
extern TireState tireState[5];
static uint16_t profileAddr(int slot) { return (uint16_t)(FRAM_PROFILES_ADDR + slot * sizeof(VehicleProfile)); }

bool profileLoad(int slot, VehicleProfile& p) {
  if (!framOk || slot < 0 || slot >= PROFILE_COUNT) return false;
  if (!framRead(profileAddr(slot), (uint8_t*)&p, sizeof(p))) return false;
  if (p.magic != PROFILE_MAGIC) return false;
  p.name[sizeof(p.name) - 1] = '\0';
  return true;
}

static void profileFillFromCurrent(VehicleProfile& p) {
  p.magic = PROFILE_MAGIC;
  portENTER_CRITICAL(&tireDataMux);
  for (int i = 0; i < 5; i++) p.tires[i] = myTires[i];
  for (int i = 0; i < 4; i++) p.thresh[i] = tireThresh[i];
  portEXIT_CRITICAL(&tireDataMux);
}

// حفظ الإعدادات الحالية في ملف السيارة النشط (يُستدعى مع كل حفظ للإعدادات)
void profileSyncActive() {
  if (!framOk) return;
  VehicleProfile p;
  char name[24];
  if (profileLoad(activeProfile, p)) memcpy(name, p.name, sizeof(name));
  else snprintf(name, sizeof(name), "سيارة %d", activeProfile + 1);
  profileFillFromCurrent(p);
  memcpy(p.name, name, sizeof(p.name));
  framWrite(profileAddr(activeProfile), (const uint8_t*)&p, sizeof(p));
}

void saveConfigToFram();

void profileActivate(int slot) {
  if (slot < 0 || slot >= PROFILE_COUNT || slot == activeProfile) return;
  profileSyncActive();                       // حفظ السيارة الحالية أولاً
  VehicleProfile p;
  bool ok = profileLoad(slot, p);
  portENTER_CRITICAL(&tireDataMux);
  const char* defLabels[5] = {"FL", "FR", "RL", "RR", "SPARE"};
  for (int i = 0; i < 5; i++) {
    if (ok) myTires[i] = p.tires[i];
    else { memset(&myTires[i], 0, sizeof(myTires[i])); strncpy(myTires[i].label, defLabels[i], sizeof(myTires[i].label) - 1); }
    tireState[i].hasData = false;
    tireState[i].packetCount = 0;
    tireState[i].lvl = 0;
    tireState[i].isLeaking = false;
    tireState[i].prevBarMs = 0;
  }
  for (int i = 0; i < 4; i++) tireThresh[i] = ok ? p.thresh[i] : TireThreshold{1.8f, 3.0f, 0.0f, 60.0f};
  portEXIT_CRITICAL(&tireDataMux);
  activeProfile = slot;
  baselineReset(-1);                         // سيارة مختلفة: ضغط طبيعي جديد
  saveConfigToFram();                        // يحفظ أيضاً ملف السيارة الجديدة
  for (int i = 0; i < 5; i++) tirePersistDirty[i] = true;
  tireDataDirty = true;
  Serial.printf("[PROFILE] activated slot %d\n", slot + 1);
}

void profileRename(int slot, const String& name) {
  if (slot < 0 || slot >= PROFILE_COUNT || !framOk) return;
  VehicleProfile p;
  if (!profileLoad(slot, p)) {
    if (slot == activeProfile) profileFillFromCurrent(p);
    else {
      memset(&p, 0, sizeof(p));
      p.magic = PROFILE_MAGIC;
      const char* defLabels[5] = {"FL", "FR", "RL", "RR", "SPARE"};
      for (int i = 0; i < 5; i++) strncpy(p.tires[i].label, defLabels[i], sizeof(p.tires[i].label) - 1);
      for (int i = 0; i < 4; i++) p.thresh[i] = TireThreshold{1.8f, 3.0f, 0.0f, 60.0f};
    }
  }
  memset(p.name, 0, sizeof(p.name));
  strncpy(p.name, name.c_str(), sizeof(p.name) - 1);
  // عدم قطع حرف UTF-8 عربي في المنتصف
  int len = strlen(p.name);
  while (len > 0 && ((uint8_t)p.name[len - 1] & 0xC0) == 0x80) len--;
  if (len > 0 && ((uint8_t)p.name[len - 1] & 0xC0) == 0xC0) len--;
  if (strlen(name.c_str()) > sizeof(p.name) - 1) p.name[len] = '\0';
  framWrite(profileAddr(slot), (const uint8_t*)&p, sizeof(p));
}

String activeProfileName() {
  VehicleProfile p;
  if (profileLoad(activeProfile, p) && p.name[0]) return String(p.name);
  return "سيارة " + String(activeProfile + 1);
}

void saveConfigToFram() {
  cfg.magic = FRAM_CFG_MAGIC;
  cfg.rssiDeltaCurrent    = rssiDeltaCurrent;
  cfg.wifiPowerIndex      = wifiPowerIndex;
  cfg.scannerDecodersMask = scannerDecodersMask;
  cfg.cpuFreqMhz          = cpuFreqCurrent;
  cfg.freqDevKhz          = freqDevCurrent;
  cfg.staleSignalTimeoutSec = staleSignalTimeoutSec;
  cfg.cfgVersion = CFG_VERSION_CURRENT;
  cfg.rxBwIndex = (uint8_t)(rxBwIndex + 1);
  cfg.freqOffsetKhz = (int16_t)freqOffsetKhz;
  cfg.techThresholdDbm = (int8_t)techThresholdDbm;
  cfg.unitPressure = (uint8_t)unitPressure;
  cfg.unitTemp = (uint8_t)unitTemp;
  cfg.buzzerOff = buzzerEnabled ? 0 : 1;
  cfg.activeProfile = (uint8_t)activeProfile;
  cfg.autoSensorsMode = (uint8_t)autoSensorsMode;
  cfg.baselineOff = baselineEnabled ? 0 : 1;
  cfg.autoTuneOff = autoTuneEnabled ? 0 : 1;
  strncpy(cfg.apSSID, apSSID.c_str(), sizeof(cfg.apSSID) - 1);
  strncpy(cfg.apPassword, apPassword.c_str(), sizeof(cfg.apPassword) - 1);
  for (int i = 0; i < 4; i++) cfg.tireThresh[i] = tireThresh[i];
  for (int i = 0; i < 5; i++) cfg.tires[i] = myTires[i];
  framPutConfig(FRAM_CFG_ADDR, cfg);
  profileSyncActive();
}

void saveWifiPower()   { saveConfigToFram(); }
void saveThreshold(int) { saveConfigToFram(); }

void applyAndSaveRxBw(int index) {
  rxBwIndex = constrain(index, 0, CC1101_BW_COUNT - 1);
  rtl_433_ESP::setRxBandwidth(currentRxBwKhz());
  saveConfigToFram();
  statusBarDirty = true;
}

void applyAndSaveFreqOffset(int khz) {
  tuneRunning = false;
  tuneNextMs = millis() + TUNE_INTERVAL_MS;
  freqOffsetKhz = constrain(khz, -150, 150);
  rtl_433_ESP::setFrequency(currentFreqMhz());
  saveConfigToFram();
  statusBarDirty = true;
}

void applyAndSaveRssiDelta(int newValue) {
  rssiDeltaCurrent = clampRssiDelta(newValue);
  rf.setRSSIThreshold(rssiDeltaCurrent);
  saveConfigToFram();
}

// الواجهة تتعامل بالدقائق (أوضح للمستخدم)؛ التخزين والاستخدام الداخلي كله
// بالثواني كما هو معمول به في staleSignalTimeoutSec في بقية الملف.
void applyAndSaveStaleTimeoutMinutes(int minutes) {
  staleSignalTimeoutSec = clampStaleTimeout((uint32_t)minutes * 60UL);
  saveConfigToFram();
}

int computeLevel(int idx, float bar, float temp) {
  if (idx < 0 || idx > 3) return 0;
  TireThreshold& t = tireThresh[idx];
  if (bar > t.pMax || temp > t.tMax) return 1;
  if (bar < t.pMin || temp < t.tMin) return 2;
  return 0;
}

void formatAge(uint32_t sec, char* out, size_t outLen) {
  if (sec < 60) snprintf(out, outLen, "%lus", (unsigned long)sec);
  else if (sec < 3600) snprintf(out, outLen, "%lum", (unsigned long)(sec / 60));
  else snprintf(out, outLen, "%luh", (unsigned long)(sec / 3600));
}

// ---------------- 6. بيانات الحساسات والإطارات ----------------
TireState tireState[5];
portMUX_TYPE tireDataMux = portMUX_INITIALIZER_UNLOCKED;

void loadPersistedTireStates() {
  for (int i = 0; i < 5; i++) {
    FramTireRecord rec;
    bool ok = framOk && framGetTireRecord((uint16_t)(FRAM_TIRES_ADDR + i * FRAM_TIRE_RECORD_SIZE), rec);
    if (ok && rec.hasData) {
      tireState[i].hasData = true;
      tireState[i].bar = rec.bar;
      tireState[i].psi = rec.psi;
      tireState[i].temp = rec.temp;
      tireState[i].status = rec.status;
      tireState[i].batteryOk = (rec.batteryOk != 0);
      tireState[i].lowBattStreak = 0;
      tireState[i].isMoving = (rec.isMoving != 0);
      tireState[i].isLeaking = (rec.isLeaking != 0);
      tireState[i].lastSeenEpoch = rec.lastSeenEpoch;
      tireState[i].packetCount = rec.packetCount;
      tireState[i].rssi = 0;
      tireState[i].prevBar = rec.bar;
      tireState[i].prevBarMs = millis();
      tireState[i].lvl = (i < 4) ? computeLevel(i, tireState[i].bar, tireState[i].temp) : 0;
      if (tireState[i].isLeaking) tireState[i].lvl = 3;
    } else {
      tireState[i].hasData = false;
      tireState[i].bar = 0; tireState[i].psi = 0; tireState[i].temp = 0;
      tireState[i].status = 0; tireState[i].batteryOk = true; tireState[i].lowBattStreak = 0; 
      tireState[i].isMoving = false; tireState[i].isLeaking = false;
      tireState[i].lvl = 0; tireState[i].rssi = 0;
      tireState[i].packetCount = 0; tireState[i].lastSeenEpoch = 0;
      tireState[i].prevBar = 0; tireState[i].prevBarMs = 0;
    }
  }
}

bool containsIgnoreCase(const char* haystack, const char* needle) {
  if (!haystack || !needle || !*needle) return false;
  for (const char* p = haystack; *p; ++p) {
    const char* a = p;
    const char* b = needle;
    while (*a && *b && tolower((unsigned char)*a) == tolower((unsigned char)*b)) { ++a; ++b; }
    if (!*b) return true;
  }
  return false;
}

volatile bool tirePersistDirty[5] = {false, false, false, false, false};
static uint32_t lastPersistMs[5] = {0,0,0,0,0};
static uint32_t lastLoggedMs[5] = {0,0,0,0,0};
static float lastLoggedBar[5] = {0,0,0,0,0};
static float lastLoggedTemp[5] = {0,0,0,0,0};
static int8_t lastLoggedRssi[5] = {0,0,0,0,0};
static SemaphoreHandle_t logMutex = NULL;
static volatile uint32_t processedPacketCounter = 0;
static volatile uint32_t decodeErrorCounter = 0;

struct FramJob {
  uint8_t type;
  uint8_t index;
  uint16_t reserved;
  FramTireRecord tire;
  LogRecord log;
};
static QueueHandle_t framQueue = NULL;
static volatile uint32_t framJobDropCounter = 0;

static const uint32_t TIRE_PERSIST_INTERVAL_MS = 3000;
static const uint32_t HISTORY_HEARTBEAT_MS = 30000;
static const float HISTORY_PRESSURE_DELTA_BAR = 0.02f;
static const float HISTORY_TEMP_DELTA_C = 1.0f;

void loadLogHeader() {
  bool ok = framOk && framGetLogHeader(FRAM_LOG_HEADER_ADDR, logHeader);
  if (!ok || logHeader.magic != FRAM_LOG_MAGIC || logHeader.writeIndex >= FRAM_LOG_MAX_RECORDS) {
    resetLogHeader();
    if (framOk) framPutLogHeader(FRAM_LOG_HEADER_ADDR, logHeader);
  }
}

bool queueHistoryRecord(uint8_t tireIdx, float bar, float temp, int rssi, uint8_t status, bool battOk, bool isMov, bool isLeak, uint32_t ts) {
  if (!framQueue || !framOk || tireIdx >= 5) return false;
  FramJob job{};
  job.type = 2;
  job.index = tireIdx;
  job.log.timestamp = ts;
  job.log.tireIdx = tireIdx;
  long rawP = lroundf(bar * 50.0f);
  job.log.pressureRaw = (uint8_t)constrain(rawP, 0L, 255L);
  long rawT = lroundf(temp);
  job.log.tempC = (int8_t)constrain(rawT, -128L, 127L);
  job.log.rssi = (int8_t)constrain(rssi, -128, 127);
  job.log.status = status;
  job.log.batteryOk = battOk ? 1 : 0;
  job.log.motionAndLeak = (isMov ? 0x01 : 0x00) | (isLeak ? 0x02 : 0x00);
  
  if (xQueueSend(framQueue, &job, 0) != pdTRUE) {
    framJobDropCounter++;
    return false;
  }
  return true;
}

static bool queueTirePersist(int idx, const FramTireRecord& rec) {
  if (!framQueue || !framOk || idx < 0 || idx >= 5) return false;
  FramJob job{};
  job.type = 1;
  job.index = (uint8_t)idx;
  job.tire = rec;
  if (xQueueSend(framQueue, &job, 0) != pdTRUE) {
    framJobDropCounter++;
    return false;
  }
  return true;
}

void framWriterTask(void* pvParameters) {
  FramJob job;
  // إصلاح أمان: تسجيل المهمة في الـWatchdog. عدم وجود عمل FRAM لعدة دقائق
  // (لا تغيّر يستدعي حفظاً) سلوك طبيعي تماماً وليس تجمّداً — لهذا استُبدلت
  // portMAX_DELAY بمهلة 5 ثوانٍ أدناه، لتُطعم الـWatchdog دورياً بغض النظر
  // عن وجود عمل فعلي، بينما تبقى قادرة على كشف تجمّد حقيقي (قفل I2C لا
  // يُحرَّر أبداً منتصف framPutLogRecord مثلاً).
  esp_task_wdt_add(NULL);
  for (;;) {
    esp_task_wdt_reset();
    if (xQueueReceive(framQueue, &job, pdMS_TO_TICKS(5000)) != pdTRUE) continue;

    if (job.type == 1) {
      framPutTireRecord((uint16_t)(FRAM_TIRES_ADDR + job.index * FRAM_TIRE_RECORD_SIZE), job.tire);
    } else if (job.type == 2) {
      if (!framOk) continue;
      if (logMutex && xSemaphoreTake(logMutex, pdMS_TO_TICKS(100)) != pdTRUE) continue;
      uint16_t addr = (uint16_t)(FRAM_LOG_DATA_ADDR + logHeader.writeIndex * FRAM_LOG_RECORD_SIZE);
      bool ok = framPutLogRecord(addr, job.log);
      if (ok) {
        logHeader.writeIndex = (logHeader.writeIndex + 1) % FRAM_LOG_MAX_RECORDS;
        if (logHeader.count < FRAM_LOG_MAX_RECORDS) logHeader.count++;
        framPutLogHeader(FRAM_LOG_HEADER_ADDR, logHeader);
      }
      if (logMutex) xSemaphoreGive(logMutex);
    }
    taskYIELD();
  }
}

#define MAX_SCANNED 15
ScannedSensor scannedList[MAX_SCANNED];
int scannedCount = 0;

// ---------------- وضع فني الإطارات (screenMode == 2) ----------------
// يلتقط الحساس الأقرب فقط (إشارة >= techThresholdDbm) ويتجاهل السيارات المارّة.
#define SCREEN_MODE_COUNT 4   // 0 السيارة، 1 الماسح، 2 فني الإطارات، 3 تشخيص الراديو
#define TECH_HIST         20      // جلسة فحص كاملة لعدة سيارات
#define TECH_STICKY_MS    10000UL   // الالتقاط ثابت 10 ثوانٍ...
#define TECH_STICKY_DB    3         // ...إلا إذا جاء حساس آخر أقوى بـ 3 dB (إطاران متجاوران)
#define TECH_FLASH_MS     1500UL


TechCapture techCap;
TechCapture techHist[TECH_HIST];
int techHistCount = 0;
volatile uint32_t techFlashUntil = 0;
volatile uint32_t techRevision = 0;   // يزيد مع كل تغيير لإعادة الرسم الكامل

// تُستدعى داخل قفل tireDataMux
static void techPushHistory(const TechCapture& c) {
  if (!c.valid) return;
  int w = 0;
  for (int i = 0; i < techHistCount; i++) {
    if (strcmp(techHist[i].id, c.id) != 0) techHist[w++] = techHist[i];
  }
  techHistCount = w;
  if (techHistCount == TECH_HIST) techHistCount--;
  for (int i = techHistCount; i > 0; i--) techHist[i] = techHist[i - 1];
  techHist[0] = c;
  techHistCount++;
}

// تُستدعى داخل قفل tireDataMux لكل حزمة تتجاوز عتبة القرب
static void techConsider(const char* id, const char* model, float bar, float temp, int rssi,
                         int status, int8_t battery, int tire, const char* mic, uint32_t nowMs) {
  TechCapture& c = techCap;
  bool same = c.valid && strcmp(c.id, id) == 0;
  if (!same && c.valid) {
    bool sticky = (nowMs - c.lastSeenMs < TECH_STICKY_MS) && (rssi < c.rssi + TECH_STICKY_DB);
    if (sticky) return;
    techPushHistory(c);
  }
  if (!same) {
    memset(&c, 0, sizeof(c));
    c.valid = true;
    strncpy(c.id, id, sizeof(c.id) - 1);
    c.bestRssi = rssi;
    c.epoch = epochNow();
    techFlashUntil = nowMs + TECH_FLASH_MS;
    pendingChirps = 1;
  }
  strncpy(c.model, model ? model : "TPMS", sizeof(c.model) - 1);
  c.bar = bar;
  c.temp = temp;
  c.rssi = rssi;
  if (rssi > c.bestRssi) c.bestRssi = rssi;
  c.status = status;
  c.battery = battery;
  c.tire = (int8_t)tire;
  strncpy(c.mic, mic ? mic : "", sizeof(c.mic) - 1);
  c.lastSeenMs = nowMs;
  c.count++;
  techRevision++;
}

static void techReset() {
  portENTER_CRITICAL(&tireDataMux);
  techPushHistory(techCap);
  memset(&techCap, 0, sizeof(techCap));
  techRevision++;
  portEXIT_CRITICAL(&tireDataMux);
}

// حكم مختصر على حالة الحساس
static const char* techVerdict(const TechCapture& c, uint16_t* color, const char** arabic) {
  uint16_t col = ST77XX_GREEN;
  const char* txt = "SENSOR OK";
  const char* ar = "الحساس سليم";
  if (c.battery == 0)                        { col = ST77XX_RED;    txt = "BATTERY LOW";   ar = "بطارية ضعيفة"; }
  else if (isnan(c.bar))                     { col = 0xFD20;        txt = "NO PRESSURE";   ar = "لا توجد قراءة ضغط"; }
  else if (c.bar < 0.3f)                     { col = 0xFD20;        txt = "0 BAR: UNMOUNTED"; ar = "ضغط صفر — غير مركّب؟"; }
  else if (!isnan(c.temp) && c.temp > 90.0f) { col = ST77XX_RED;    txt = "OVERHEAT";      ar = "حرارة زائدة"; }
  if (color) *color = col;
  if (arabic) *arabic = ar;
  return txt;
}

void addOrUpdateScanned(const char* id, const char* model, float bar,
                        float temp, int rssi, int status, bool batteryOk, bool isMoving, bool isLeaking, bool isMine, bool isToyota, uint32_t nowSec) {
  if (!id || !*id) return;
  for (int i = 0; i < scannedCount; i++) {
    if (strncmp(scannedList[i].id, id, sizeof(scannedList[i].id)) == 0) {
      scannedList[i].bar = bar;
      scannedList[i].temp = temp;
      scannedList[i].rssi = rssi;
      scannedList[i].status = status;
      scannedList[i].batteryOk = batteryOk;
      scannedList[i].isMoving = isMoving;
      scannedList[i].isLeaking = isLeaking;
      scannedList[i].packetCount++;
      scannedList[i].lastSeenEpoch = nowSec;
      scannedList[i].isMine = isMine;
      scannedList[i].isToyota = isToyota;
      if (model && *model) {
        strncpy(scannedList[i].model, model, sizeof(scannedList[i].model)-1);
        scannedList[i].model[sizeof(scannedList[i].model)-1] = '\0';
      }
      return;
    }
  }
  int lastIndex = (scannedCount < MAX_SCANNED) ? scannedCount : (MAX_SCANNED - 1);
  for (int i = lastIndex; i > 0; i--) scannedList[i] = scannedList[i - 1];
  memset(&scannedList[0], 0, sizeof(scannedList[0]));
  strncpy(scannedList[0].id, id, sizeof(scannedList[0].id)-1);
  strncpy(scannedList[0].model, model ? model : "TPMS", sizeof(scannedList[0].model)-1);
  scannedList[0].bar = bar;
  scannedList[0].temp = temp;
  scannedList[0].rssi = rssi;
  scannedList[0].status = status;
  scannedList[0].batteryOk = batteryOk;
  scannedList[0].isMoving = isMoving;
  scannedList[0].isLeaking = isLeaking;
  scannedList[0].packetCount = 1;
  scannedList[0].lastSeenEpoch = nowSec;
  scannedList[0].isMine = isMine;
  scannedList[0].isToyota = isToyota;
  if (scannedCount < MAX_SCANNED) scannedCount++;
}

static void markTireDirty(int idx) {
  if (idx >= 0 && idx < 5) tirePersistDirty[idx] = true;
}

static void maybeQueueHistory(uint8_t idx, float bar, float temp, int rssi, uint8_t status, bool battOk, bool isMov, bool isLeak, uint32_t nowSec, uint32_t nowMs) {
  if (idx >= 5 || !framOk) return;
  bool first = (lastLoggedMs[idx] == 0);
  bool pressureChanged = first || fabsf(bar - lastLoggedBar[idx]) >= HISTORY_PRESSURE_DELTA_BAR;
  bool tempChanged = first || fabsf(temp - lastLoggedTemp[idx]) >= HISTORY_TEMP_DELTA_C;
  bool heartbeat = first || (nowMs - lastLoggedMs[idx] >= HISTORY_HEARTBEAT_MS);
  if (pressureChanged || tempChanged || heartbeat) {
    if (!queueHistoryRecord(idx, bar, temp, rssi, status, battOk, isMov, isLeak, nowSec)) return;
    lastLoggedMs[idx] = nowMs;
    lastLoggedBar[idx] = bar;
    lastLoggedTemp[idx] = temp;
    lastLoggedRssi[idx] = (int8_t)constrain(rssi, -128, 127);
  }
}

static void flushDirtyTires(bool force) {
  uint32_t now = millis();
  for (int i = 0; i < 5; ++i) {
    if (!tirePersistDirty[i]) continue;
    if (!force && (now - lastPersistMs[i] < TIRE_PERSIST_INTERVAL_MS)) continue;
    FramTireRecord rec;
    portENTER_CRITICAL(&tireDataMux);
    rec.hasData = tireState[i].hasData ? 1 : 0;
    rec.bar = tireState[i].bar;
    rec.psi = tireState[i].psi;
    rec.temp = tireState[i].temp;
    rec.status = tireState[i].status;
    rec.batteryOk = tireState[i].batteryOk ? 1 : 0;
    rec.isMoving = tireState[i].isMoving ? 1 : 0;
    rec.isLeaking = tireState[i].isLeaking ? 1 : 0;
    rec.lastSeenEpoch = tireState[i].lastSeenEpoch;
    rec.packetCount = tireState[i].packetCount;
    portEXIT_CRITICAL(&tireDataMux);
    if (queueTirePersist(i, rec)) {
      tirePersistDirty[i] = false;
      lastPersistMs[i] = now;
    }
  }
}

void formatAndResetFRAM() {
  if (!framOk) return;

  setDefaultConfig();
  framPutConfig(FRAM_CFG_ADDR, cfg);
  rssiDeltaCurrent    = cfg.rssiDeltaCurrent;
  wifiPowerIndex      = cfg.wifiPowerIndex;
  scannerDecodersMask = cfg.scannerDecodersMask;
  cpuFreqCurrent      = cfg.cpuFreqMhz;
  freqDevCurrent      = cfg.freqDevKhz;
  apSSID     = String(cfg.apSSID);
  apPassword = String(cfg.apPassword);
  for (int i = 0; i < 4; i++) tireThresh[i] = cfg.tireThresh[i];
  for (int i = 0; i < 5; i++) myTires[i] = cfg.tires[i];

  for (int i = 0; i < 5; i++) {
    FramTireRecord rec{};
    rec.hasData = 0;
    rec.batteryOk = 1;
    framPutTireRecord((uint16_t)(FRAM_TIRES_ADDR + i * FRAM_TIRE_RECORD_SIZE), rec);
  }
  loadPersistedTireStates();

  if (logMutex && xSemaphoreTake(logMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    resetLogHeader();
    framPutLogHeader(FRAM_LOG_HEADER_ADDR, logHeader);
    xSemaphoreGive(logMutex);
  }

  portENTER_CRITICAL(&tireDataMux);
  scannedCount = 0;
  portEXIT_CRITICAL(&tireDataMux);

  setCpuFrequencyMhz(cpuFreqCurrent);
  rf.setRSSIThreshold(rssiDeltaCurrent);
  rf.setFrequencyDeviation((float)freqDevCurrent);
  applyWifiPower();
  syncDecoderEngineWithScreenMode();

  tireDataDirty = true;
  statusBarDirty = true;
  topBarDirty = true;
}

// ---------------- 7. معالجة إشارات الراديو بفلترة استباقية سريعة ----------------
void logSignal(char* message) {
  if (!message || !rfQueue) return;
  RawRfPacket pkt;
  pkt.rssi = rtl_433_ESP::signalRssi;
  strncpy(pkt.text, message, sizeof(pkt.text) - 1);
  pkt.text[sizeof(pkt.text) - 1] = '\0';
  
  if (xQueueSend(rfQueue, &pkt, 0) != pdTRUE) {
    rfQueueDropCounter++;
  }
}

// =============================================================================
// التعلّم الذاتي (1): اكتشاف حساسات السيارة من تكرار ظهورها عبر القيادات
// =============================================================================
#define SMART_QUAL_BURSTS   3
#define SMART_QUAL_RSSI     -80
#define SMART_SESSION_MS    (2UL * 3600UL * 1000UL)
#define SMART_MISSING_AFTER 3     // جلسات نشطة متتالية دون الحساس = مختفٍ
SmartHeader      smartHdr;
SmartSensor      smartTab[SMART_MAX_SENSORS];
SmartSessionCand smartSess[24];
int              smartSessCount = 0;
bool             smartSessionCounted = false;
uint32_t         smartSessionStartMs = 0;
volatile bool    smartDirty = false;
SmartProposal    smartProposal;
uint32_t         smartPromptUntil = 0;
char             smartDismissed[8][12];
int              smartDismissedCount = 0;
String           smartLastAction = "";

// هل المعرّف المخزَّن (قد يكون لاحقة قصيرة) يطابق معرّفاً كاملاً؟
static bool idMatchesBinding(const char* fullId, const char* shortId) {
  size_t fl = strlen(fullId), sl = strlen(shortId);
  if (sl == 0 || sl > fl) return false;
  return strcmp(fullId + fl - sl, shortId) == 0;
}

static int smartAssignedSlot(const char* id) {
  for (int i = 0; i < 5; i++) if (idMatchesBinding(id, myTires[i].shortId)) return i;
  return -1;
}

static bool smartConfirmed(const SmartSensor& t) {
  if (t.id[0] == 0 || t.qualSessions < 3) return false;
  uint16_t span = smartHdr.activeSessions - t.activeAtFirst + 1;
  return (uint32_t)t.qualSessions * 10 >= (uint32_t)span * 6;   // ظهر في 60% من القيادات منذ اكتشافه
}

// تُستدعى داخل قفل tireDataMux لكل حزمة
static void smartFeed(const char* id, const char* model, int rssi, uint32_t nowMs) {
  int k = -1;
  for (int i = 0; i < smartSessCount; i++) if (strcmp(smartSess[i].id, id) == 0) { k = i; break; }
  if (k < 0) {
    if (smartSessCount >= 24) return;
    k = smartSessCount++;
    memset(&smartSess[k], 0, sizeof(SmartSessionCand));
    strncpy(smartSess[k].id, id, sizeof(smartSess[k].id) - 1);
  }
  SmartSessionCand& c = smartSess[k];
  if (c.packets == 0 || nowMs - c.lastMs >= 8000UL) c.bursts++;
  c.packets++;
  c.rssiSum += rssi;
  c.lastMs = nowMs;
  if (c.qualified || c.bursts < SMART_QUAL_BURSTS || c.rssiSum / (int32_t)c.packets < SMART_QUAL_RSSI) return;
  c.qualified = true;

  if (!smartSessionCounted) { smartHdr.activeSessions++; smartSessionCounted = true; }
  int e = -1;
  for (int i = 0; i < SMART_MAX_SENSORS; i++) if (strcmp(smartTab[i].id, id) == 0) { e = i; break; }
  if (e < 0) {
    for (int i = 0; i < SMART_MAX_SENSORS; i++) if (smartTab[i].id[0] == 0) { e = i; break; }
    if (e < 0) {  // الجدول ممتلئ: استبدال أضعف حساس غير مربوط
      for (int i = 0; i < SMART_MAX_SENSORS; i++) {
        if (smartAssignedSlot(smartTab[i].id) >= 0) continue;
        if (e < 0 || smartTab[i].qualSessions < smartTab[e].qualSessions) e = i;
      }
      if (e < 0) return;
    }
    memset(&smartTab[e], 0, sizeof(SmartSensor));
    strncpy(smartTab[e].id, id, sizeof(smartTab[e].id) - 1);
    strncpy(smartTab[e].model, model ? model : "TPMS", sizeof(smartTab[e].model) - 1);
    smartTab[e].activeAtFirst = smartHdr.activeSessions;
    smartTab[e].lastQualSession = 0xFFFF;
  }
  if (smartTab[e].lastQualSession != smartHdr.sessionCounter) {
    smartTab[e].qualSessions++;
    smartTab[e].lastQualSession = smartHdr.sessionCounter;
    smartTab[e].lastQualActive = smartHdr.activeSessions;
    smartDirty = true;
  }
}

static void smartNewSession(const char* why) {
  portENTER_CRITICAL(&tireDataMux);
  smartHdr.sessionCounter++;
  smartSessCount = 0;
  smartSessionCounted = false;
  smartDirty = true;
  portEXIT_CRITICAL(&tireDataMux);
  smartSessionStartMs = millis();
  Serial.printf("[SMART] session %u (%s), active sessions %u\n", smartHdr.sessionCounter, why, smartHdr.activeSessions);
}

void smartLoad() {
  bool ok = framOk && framRead(FRAM_SMART_ADDR, (uint8_t*)&smartHdr, sizeof(smartHdr)) && smartHdr.magic == SMART_MAGIC &&
            framRead(FRAM_SMART_ADDR + sizeof(SmartHeader), (uint8_t*)smartTab, sizeof(smartTab));
  if (!ok) {
    memset(&smartHdr, 0, sizeof(smartHdr));
    memset(smartTab, 0, sizeof(smartTab));
    smartHdr.magic = SMART_MAGIC;
  }
  for (int i = 0; i < SMART_MAX_SENSORS; i++) { smartTab[i].id[11] = 0; smartTab[i].model[9] = 0; }
  smartNewSession("boot");
}

void smartReset() {
  portENTER_CRITICAL(&tireDataMux);
  memset(&smartHdr, 0, sizeof(smartHdr));
  memset(smartTab, 0, sizeof(smartTab));
  smartHdr.magic = SMART_MAGIC;
  smartSessCount = 0;
  smartSessionCounted = false;
  smartProposal.valid = false;
  smartDirty = true;
  portEXIT_CRITICAL(&tireDataMux);
}

// تطبيق الاقتراح الحالي على مواقع الإطارات
void smartApplyProposal(bool automatic) {
  SmartProposal p;
  portENTER_CRITICAL(&tireDataMux);
  p = smartProposal;
  smartProposal.valid = false;
  portEXIT_CRITICAL(&tireDataMux);
  if (!p.valid) return;
  String msg;
  portENTER_CRITICAL(&tireDataMux);
  for (int i = 0; i < p.count; i++) {
    int sl = p.slot[i];
    if (sl < 0 || sl > 4) continue;
    memset(myTires[sl].shortId, 0, sizeof(myTires[sl].shortId));
    strncpy(myTires[sl].shortId, p.id[i], sizeof(myTires[sl].shortId) - 1);
    tireState[sl].hasData = false;
    tireState[sl].packetCount = 0;
    tireState[sl].lvl = 0;
  }
  portEXIT_CRITICAL(&tireDataMux);
  for (int i = 0; i < p.count; i++) msg += String(myTires[p.slot[i]].label) + "=" + p.id[i] + " ";
  saveConfigToFram();
  smartPromptUntil = 0;
  tireDataDirty = true;
  lastDrawnScreenMode = -1;
  pendingChirps = 2;
  smartLastAction = String(automatic ? "تحديث تلقائي: " : "تم التطبيق: ") + msg;
  Serial.printf("[SMART] applied (%s): %s\n", automatic ? "auto" : "confirmed", msg.c_str());
}

void smartDismissProposal() {
  portENTER_CRITICAL(&tireDataMux);
  if (smartProposal.valid) {
    for (int i = 0; i < smartProposal.count && smartDismissedCount < 8; i++)
      strncpy(smartDismissed[smartDismissedCount++], smartProposal.id[i], 11);
  }
  smartProposal.valid = false;
  portEXIT_CRITICAL(&tireDataMux);
  smartPromptUntil = 0;
  lastDrawnScreenMode = -1;
}

static bool smartIsDismissed(const char* id) {
  for (int i = 0; i < smartDismissedCount; i++) if (strcmp(smartDismissed[i], id) == 0) return true;
  return false;
}

bool slotHasOriginal(int slot);

// يبني اقتراحاً: (أ) أول إعداد بلا معرّفات، أو (ب) حساس مربوط اختفى وحلّ محله حساس جديد
void smartEvaluate() {
  if (autoSensorsMode == 1 || smartProposal.valid || learnActive) return;
  SmartSensor tab[SMART_MAX_SENSORS];
  SmartHeader hdr;
  SensorBinding tires[5];
  portENTER_CRITICAL(&tireDataMux);
  memcpy(tab, smartTab, sizeof(tab));
  hdr = smartHdr;
  memcpy(tires, myTires, sizeof(tires));
  portEXIT_CRITICAL(&tireDataMux);

  // المرشحون المؤكَّدون غير المربوطين، الأكثر ظهوراً أولاً
  int cand[SMART_MAX_SENSORS], nc = 0;
  for (int i = 0; i < SMART_MAX_SENSORS; i++) {
    if (!smartConfirmed(tab[i]) || smartIsDismissed(tab[i].id)) continue;
    bool assigned = false;
    for (int t = 0; t < 5; t++) if (idMatchesBinding(tab[i].id, tires[t].shortId)) assigned = true;
    if (!assigned) cand[nc++] = i;
  }
  for (int a = 1; a < nc; a++) { int k = cand[a], b = a - 1; while (b >= 0 && tab[cand[b]].qualSessions < tab[k].qualSessions) { cand[b + 1] = cand[b]; b--; } cand[b + 1] = k; }
  if (nc == 0) return;

  SmartProposal p;
  memset(&p, 0, sizeof(p));
  bool anyAssigned = false;
  for (int t = 0; t < 5; t++) if (tires[t].shortId[0]) anyAssigned = true;

  if (!anyAssigned) {
    if (nc < 4) return;                              // ننتظر حتى تتأكد 4 حساسات على الأقل
    for (int i = 0; i < nc && i < 5; i++) { p.slot[p.count] = i; strncpy(p.id[p.count], tab[cand[i]].id, 11); p.count++; }
  } else {
    // إطارات مربوطة اختفت (الاحتياطي مستثنى لأنه لا يرسل وهو لا يدور)
    int missing[4], nm = 0, healthy = 0;
    for (int t = 0; t < 4; t++) {
      if (!tires[t].shortId[0]) continue;
      int e = -1;
      for (int i = 0; i < SMART_MAX_SENSORS; i++) if (tab[i].id[0] && idMatchesBinding(tab[i].id, tires[t].shortId)) { e = i; break; }
      bool recent = e >= 0 && (uint16_t)(hdr.activeSessions - tab[e].lastQualActive) < SMART_MISSING_AFTER;
      if (recent) healthy++;
      else if (hdr.activeSessions >= SMART_MISSING_AFTER) missing[nm++] = t;
    }
    if (nm == 0) return;
    // نحتاج دليلاً أن السيارة تتحرك فعلاً: إطاران سليمان على الأقل، أو كل المعرّفات المخزنة
    // خاطئة (سيارة أخرى/حساسات قديمة) مع 4 حساسات جديدة مؤكَّدة
    if (healthy < 2 && !(healthy == 0 && nc >= 4)) return;
    for (int i = 0; i < nm && i < nc; i++) {
      if ((uint16_t)(hdr.activeSessions - tab[cand[i]].lastQualActive) >= 2) continue; // الجديد يجب أن يكون حديثاً
      p.slot[p.count] = missing[i];
      strncpy(p.id[p.count], tab[cand[i]].id, 11);
      p.count++;
    }
  }
  if (p.count == 0) return;
  p.valid = true;
  p.createdMs = millis();
  portENTER_CRITICAL(&tireDataMux);
  smartProposal = p;
  portEXIT_CRITICAL(&tireDataMux);
  Serial.printf("[SMART] proposal: %d sensor(s)\n", p.count);
  bool touchesOriginal = false;
  for (int i = 0; i < p.count; i++) if (slotHasOriginal(p.slot[i])) touchesOriginal = true;
  if (autoSensorsMode == 3 && !touchesOriginal) {
    smartApplyProposal(true);
  } else {
    smartPromptUntil = millis() + 60000UL;
    pendingChirps = 1;
  }
}

// =============================================================================
// الحساسات الأصلية للسيارة
// =============================================================================
static const char* const FACTORY_ORIGINAL_IDS[5] = {"D78D8483", "D78D8C0A", "D78D8CD9", "D78D8C17", "D78D8409"};
OriginalSet originals;

void originalsLoad() {
  bool ok = framOk && framRead(FRAM_ORIG_ADDR, (uint8_t*)&originals, sizeof(originals)) && originals.magic == ORIG_MAGIC;
  if (!ok) {
    memset(&originals, 0, sizeof(originals));
    originals.magic = ORIG_MAGIC;
    for (int i = 0; i < 5; i++) strncpy(originals.ids[i], FACTORY_ORIGINAL_IDS[i], 11);
    if (framOk) framWrite(FRAM_ORIG_ADDR, (const uint8_t*)&originals, sizeof(originals));
  }
  for (int i = 0; i < 5; i++) originals.ids[i][11] = 0;
}

// هل الإطار يحمل حساسه الأصلي الآن؟ (مطابقة بالاحقة في الاتجاهين: 7 أو 8 خانات)
bool slotHasOriginal(int slot) {
  const char* cur = myTires[slot].shortId;
  const char* org = originals.ids[slot];
  if (!cur[0] || !org[0]) return false;
  return idMatchesBinding(org, cur) || idMatchesBinding(cur, org);
}

void originalsRestore() {
  portENTER_CRITICAL(&tireDataMux);
  for (int i = 0; i < 5; i++) {
    memset(myTires[i].shortId, 0, sizeof(myTires[i].shortId));
    strncpy(myTires[i].shortId, originals.ids[i], sizeof(myTires[i].shortId) - 1);
    tireState[i].hasData = false;
    tireState[i].packetCount = 0;
    tireState[i].lvl = 0;
  }
  smartProposal.valid = false;
  portEXIT_CRITICAL(&tireDataMux);
  saveConfigToFram();
  tireDataDirty = true;
  lastDrawnScreenMode = -1;
  Serial.println("[ORIG] original sensors restored");
}

void originalsSaveCurrent() {
  portENTER_CRITICAL(&tireDataMux);
  for (int i = 0; i < 5; i++) {
    memset(originals.ids[i], 0, sizeof(originals.ids[i]));
    strncpy(originals.ids[i], myTires[i].shortId, 11);
  }
  portEXIT_CRITICAL(&tireDataMux);
  if (framOk) framWrite(FRAM_ORIG_ADDR, (const uint8_t*)&originals, sizeof(originals));
  Serial.println("[ORIG] current sensors saved as originals");
}

// =============================================================================
// التعلّم الذاتي (2): الضغط البارد الطبيعي لكل إطار
// =============================================================================
#define BASE_SAMPLE_SEC   300UL       // عينة واحدة كل 5 دقائق لكل إطار
#define BASE_MIN_SAMPLES  20
#define BASE_MIN_SPAN_SEC 86400UL     // يوم على الأقل قبل التفعيل
#define BASE_DEV_BAR      0.15f       // الانخفاض عن الطبيعي الذي يطلق التنبيه
TireBaseline tireBaseline[5];
volatile bool baselineDirty = false;

static bool baselineReady(const TireBaseline& b) {
  return b.samples >= BASE_MIN_SAMPLES && (b.lastSampleEpoch - b.firstEpoch) >= BASE_MIN_SPAN_SEC;
}

// تُستدعى داخل قفل tireDataMux. تعيد true إذا كان الإطار أقل من ضغطه الطبيعي.
static bool baselineUpdateLocked(int idx, float bar, float temp, uint32_t nowSec, bool allowLearn) {
  if (!baselineEnabled || idx < 0 || idx > 4) return false;
  float cold = coldPressureBar(bar, temp);
  if (isnan(cold) || bar < 0.5f) return false;
  TireBaseline& b = tireBaseline[idx];
  bool ready = baselineReady(b);
  bool belowNormal = ready && cold < b.coldBar - BASE_DEV_BAR;
  if (!allowLearn) return belowNormal;
  if (b.samples == 0) {
    b.coldBar = cold; b.samples = 1; b.firstEpoch = nowSec; b.lastSampleEpoch = nowSec;
    baselineDirty = true;
  } else if (nowSec - b.lastSampleEpoch >= BASE_SAMPLE_SEC) {
    // لا نتعلّم من الانخفاض (حتى لا يصبح التسريب هو "الطبيعي")، ونتبع النفخ بسرعة
    if (!belowNormal) {
      float alpha = (cold > b.coldBar + 0.10f) ? 0.25f : (b.samples < BASE_MIN_SAMPLES ? 1.0f / (b.samples + 1) : 0.02f);
      b.coldBar += alpha * (cold - b.coldBar);
      b.samples++;
    }
    b.lastSampleEpoch = nowSec;
    baselineDirty = true;
  }
  return belowNormal;
}

void baselineLoad() {
  bool ok = framOk && framRead(FRAM_BASELINE_ADDR, (uint8_t*)tireBaseline, sizeof(tireBaseline));
  for (int i = 0; i < 5; i++) {
    if (!ok || isnan(tireBaseline[i].coldBar) || tireBaseline[i].coldBar < 0.3f || tireBaseline[i].coldBar > 8.0f)
      memset(&tireBaseline[i], 0, sizeof(TireBaseline));
  }
}

void baselineReset(int idx) {
  portENTER_CRITICAL(&tireDataMux);
  for (int i = 0; i < 5; i++) if (idx < 0 || i == idx) memset(&tireBaseline[i], 0, sizeof(TireBaseline));
  baselineDirty = true;
  portEXIT_CRITICAL(&tireDataMux);
}

// =============================================================================
// التعلّم الذاتي (3): المعايرة الذاتية لتردد الراديو
// =============================================================================
static int      tuneBaseOff = 0, tuneCandOff = 0, tuneSlice = 0;
static uint32_t tuneSliceStartMs = 0, tuneSliceStartCount = 0, tuneCountBase = 0, tuneCountCand = 0;

static void tuneSetOffset(int off) { rtl_433_ESP::setFrequency(CC1101_FREQUENCY + off / 1000.0f); }

void tuneStart() {
  tuneBaseOff = freqOffsetKhz;
  tuneCandOff = constrain(freqOffsetKhz + tuneDir * TUNE_STEP_KHZ, -150, 150);
  if (tuneCandOff == tuneBaseOff) { tuneDir = -tuneDir; tuneCandOff = constrain(freqOffsetKhz + tuneDir * TUNE_STEP_KHZ, -150, 150); }
  tuneSlice = 0;
  tuneCountBase = tuneCountCand = 0;
  tuneSliceStartMs = millis();
  tuneSliceStartCount = rtl_433_ESP::decodedMessages;
  tuneRunning = true;
  tuneSetOffset(tuneBaseOff);
  Serial.printf("[TUNE] comparing %+d vs %+d kHz (20 min)\n", tuneBaseOff, tuneCandOff);
}

void tuneService() {
  uint32_t now = millis();
  if (!tuneRunning) {
    if (autoTuneEnabled && isCC1101Ok && !learnActive && screenMode != 2 && (int32_t)(now - tuneNextMs) >= 0) tuneStart();
    return;
  }
  if (!autoTuneEnabled || learnActive || screenMode == 2 || !isCC1101Ok) {   // إلغاء: العودة للإزاحة الأصلية
    tuneRunning = false;
    tuneSetOffset(freqOffsetKhz);
    tuneNextMs = now + TUNE_INTERVAL_MS;
    return;
  }
  if (now - tuneSliceStartMs < TUNE_SLICE_MS) return;
  uint32_t got = rtl_433_ESP::decodedMessages - tuneSliceStartCount;
  if (tuneSlice % 2 == 0) tuneCountBase += got; else tuneCountCand += got;
  tuneSlice++;
  if (tuneSlice < TUNE_SLICES) {
    tuneSetOffset(tuneSlice % 2 == 0 ? tuneBaseOff : tuneCandOff);
    tuneSliceStartMs = now;
    tuneSliceStartCount = rtl_433_ESP::decodedMessages;
    return;
  }
  tuneRunning = false;
  uint32_t total = tuneCountBase + tuneCountCand;
  char res[96];
  if (total < 20) {
    tuneSetOffset(tuneBaseOff);
    tuneNextMs = now + 30UL * 60UL * 1000UL;   // حركة قليلة: إعادة المحاولة بعد 30 دقيقة
    snprintf(res, sizeof(res), "غير حاسمة (%lu حزمة فقط)", (unsigned long)total);
  } else if (tuneCountCand * 100 >= tuneCountBase * 115 && tuneCountCand >= tuneCountBase + 4) {
    snprintf(res, sizeof(res), "تحسّن: %+d ← %+d kHz (%lu مقابل %lu حزمة)", tuneBaseOff, tuneCandOff,
             (unsigned long)tuneCountCand, (unsigned long)tuneCountBase);
    applyAndSaveFreqOffset(tuneCandOff);       // نبقى في نفس الاتجاه في الدورة القادمة
    tuneNextMs = now + TUNE_INTERVAL_MS;
  } else {
    tuneSetOffset(tuneBaseOff);
    tuneDir = -tuneDir;                        // جرّب الاتجاه الآخر في المرة القادمة
    tuneNextMs = now + TUNE_INTERVAL_MS;
    snprintf(res, sizeof(res), "الحالية أفضل: %+d kHz (%lu مقابل %lu عند %+d)", tuneBaseOff,
             (unsigned long)tuneCountBase, (unsigned long)tuneCountCand, tuneCandOff);
  }
  DateTime dt(epochNow());
  char when[24];
  snprintf(when, sizeof(when), " — %02d/%02d %02d:%02d", dt.day(), dt.month(), dt.hour(), dt.minute());
  tuneLastResult = String(res) + when;
  Serial.printf("[TUNE] %s\n", res);
}

// خدمة دورية من loop(): جلسات، اقتراحات، حفظ في FRAM
void smartService() {
  uint32_t now = millis();
  if (now - smartSessionStartMs >= SMART_SESSION_MS) smartNewSession("2h");
  static uint32_t lastEval = 0, lastSave = 0;
  if (now - lastEval >= 60000UL) { lastEval = now; smartEvaluate(); }
  if (now - lastSave >= 30000UL) {
    lastSave = now;
    if (smartDirty && framOk) {
      SmartHeader h; static SmartSensor t[SMART_MAX_SENSORS];
      portENTER_CRITICAL(&tireDataMux);
      h = smartHdr; memcpy(t, smartTab, sizeof(t)); smartDirty = false;
      portEXIT_CRITICAL(&tireDataMux);
      framWrite(FRAM_SMART_ADDR, (const uint8_t*)&h, sizeof(h));
      framWrite(FRAM_SMART_ADDR + sizeof(SmartHeader), (const uint8_t*)t, sizeof(t));
    }
    if (baselineDirty && framOk) {
      TireBaseline b[5];
      portENTER_CRITICAL(&tireDataMux);
      memcpy(b, tireBaseline, sizeof(b)); baselineDirty = false;
      portEXIT_CRITICAL(&tireDataMux);
      framWrite(FRAM_BASELINE_ADDR, (const uint8_t*)b, sizeof(b));
    }
  }
  tuneService();
}

void storageTask(void* pvParameters) {
  RawRfPacket pkt;
  uint32_t lastFlush = millis();
  // إصلاح أمان: تسجيل المهمة في الـWatchdog. المهلة أدناه محدودة أصلاً
  // (20ms) فالإطعام هنا آمن تماماً دون أي خطر إنذار كاذب.
  esp_task_wdt_add(NULL);
  for (;;) {
    esp_task_wdt_reset();
    if (xQueueReceive(rfQueue, &pkt, pdMS_TO_TICKS(20)) == pdTRUE) {
      processedPacketCounter++;

      // إصلاح تزامن (Race Condition): أخذ نسخة محلية سريعة من المعرّفات تحت
      // القفل بدل قراءة myTires[] مباشرة من هنا وهناك بلا حماية طوال معالجة
      // الحزمة (كانت تتقاطع نظرياً مع الكتابة من صفحة "تخصيص الإطارات" —
      // انظر تعليق الإصلاح في معالج do=tires_config). القفل هنا قصير جداً
      // (نسخ 5×12 بايت فقط)، والمطابقة الفعلية تتم بعده على النسخة المحلية.
      char localShortIds[5][sizeof(myTires[0].shortId)];
      portENTER_CRITICAL(&tireDataMux);
      for (int i = 0; i < 5; i++) memcpy(localShortIds[i], myTires[i].shortId, sizeof(localShortIds[i]));
      portEXIT_CRITICAL(&tireDataMux);

      // فلتر استباقي سريع (Fast Pre-Filter):
      // إذا كنا في واجهة السيارة، نفحص فوراً إن كان النص يحتوي على معرّف أي من إطاراتنا
      // إن لم يكن كذلك، نتجاوز الحزمة فوراً دون إهدار وقت المعالج في فك الـ JSON
      bool matchesMyCar = false;
      for (int i = 0; i < 5; i++) {
        if (strlen(localShortIds[i]) > 0 && containsIgnoreCase(pkt.text, localShortIds[i])) {
          matchesMyCar = true;
          break;
        }
      }

      // اكتشاف الحساسات الذاتي يحتاج رؤية كل الحزم حتى في وضع السيارة
      if (screenMode == 0 && !matchesMyCar && !learnActive && autoSensorsMode == 1) {
        continue; // تجاهل حزم السيارات المجاورة بسرعة فائقة O(1)
      }

      // JsonDocument (ArduinoJson v7) يستخدم مخصص ذاكرة ديناميكي داخلياً.
      // إعلانه "static" هنا يجعله يُنشأ مرة واحدة فقط طوال عمر البرنامج،
      // ثم يُعاد استخدام نفس كتلة الذاكرة الداخلية مع كل حزمة عبر doc.clear()
      // بدل تخصيص (malloc) وتحرير (free) جزء من الـ Heap مع كل إشارة راديو —
      // وهو ما كان يحدث سابقاً بمعدل عالٍ جداً في وضع الماسح (25 ديكودر نشط)
      // ويُسبب تراكم تفتت الذاكرة (Heap Fragmentation) مع طول مدة التشغيل.
      static JsonDocument doc;
      doc.clear();
      DeserializationError error = deserializeJson(doc, pkt.text);
      if (error) { decodeErrorCounter++; continue; }

      // RSSI الفعلي لهذه الإشارة (من بداية التقاطها) بدل آخر قيمة عامة
      pkt.rssi = doc["rssi"] | pkt.rssi;

      // الضغط والحرارة: المكتبة تحوّل إلى SI، مع احتياط لأي وحدة أخرى
      float kpaIn = doc["pressure_kPa"].is<float>() ? doc["pressure_kPa"].as<float>()
                  : doc["pressure_PSI"].is<float>() ? doc["pressure_PSI"].as<float>() * 6.894757f
                  : doc["pressure_bar"].is<float>() ? doc["pressure_bar"].as<float>() * 100.0f : NAN;
      float tempIn = doc["temperature_C"].is<float>() ? doc["temperature_C"].as<float>()
                   : doc["temperature_F"].is<float>() ? (doc["temperature_F"].as<float>() - 32.0f) * 5.0f / 9.0f : NAN;

      // فلتر منطقية: المفككات ضعيفة التحقق تُخرج أحياناً قيماً مستحيلة من إشارات غيرها
      {
        const char* proto = doc["protocol"] | "";
        bool weak = containsIgnoreCase(proto, "Jansite TPMS Model TY02S") || containsIgnoreCase(proto, "TY588") ||
                    containsIgnoreCase(proto, "Jeep") || containsIgnoreCase(proto, "Abarth");
        bool badP = !isnan(kpaIn) && (kpaIn < 30.0f || kpaIn > 700.0f);
        bool badT = !isnan(tempIn) && (tempIn < -30.0f || tempIn > 110.0f);
        if (weak && (badP || badT)) {
          rfRejectedCounter++;
          if (serialPacketLog) Serial.printf("[RF-REJ] %s\n", pkt.text);
          continue;
        }
      }
      if (serialPacketLog) Serial.printf("[RF] %s\n", pkt.text);

      const char* model = doc["model"] | "TPMS";
      bool isToyota = containsIgnoreCase(model, "toyota") || containsIgnoreCase(model, "pmv");

      char idRawBuf[24] = {0};
      if (doc["id"].is<const char*>()) {
        strncpy(idRawBuf, doc["id"].as<const char*>(), sizeof(idRawBuf)-1);
      } else if (doc["id"].is<long>()) {
        snprintf(idRawBuf, sizeof(idRawBuf), "%lX", (long)doc["id"]);
      } else {
        continue;
      }

      char idBuf[24] = {0};
      int idLen = 0;
      for (char* p = idRawBuf; *p && idLen < (int)sizeof(idBuf) - 1; ++p) {
        if (isalnum((unsigned char)*p)) idBuf[idLen++] = *p;
      }
      idBuf[idLen] = '\0';
      if (idLen == 0) continue;

      for (char* p = idBuf; *p; ++p) *p = (char)toupper((unsigned char)*p);

      int tireIdx = -1;
      for (int i = 0; i < 5; i++) {
        if (strlen(localShortIds[i]) > 0 && strstr(idBuf, localShortIds[i])) {
          const char* tail = idBuf + strlen(idBuf) - strlen(localShortIds[i]);
          if (strcmp(tail, localShortIds[i]) == 0) { tireIdx = i; break; }
        }
      }

      rfPacketCounter++;
      float kpa = isnan(kpaIn) ? 0.0f : kpaIn;
      float temp = isnan(tempIn) ? 0.0f : tempIn;
      float bar = kpa / 100.0f;
      float psi = kpa * 0.145038f;
      int status = doc["status"] | 0;

      // الحركة: بت 0x02 في حالة تويوتا فقط (معناه غير معروف في البروتوكولات الأخرى).
      // التسريب من الحساس نفسه: حقول صريحة فقط (PMV-107J يرسل rapid_change)، بدل
      // تخمين بتات الحالة الذي كان قد يطلق إنذار تسريب كاذباً.
      bool isMoving = isToyota && (status & 0x02) != 0;
      bool isHwLeak = (doc["rapid_change"] | 0) != 0 || (doc["fast_leak"] | 0) != 0 || (doc["leaking"] | 0) != 0;

      uint32_t nowMs = millis();
      bool isSwLeak = false;

      bool packetReportsBatt = false;
      bool packetBattOk = true;

      if (!doc["battery_ok"].isNull()) {
        packetBattOk = (doc["battery_ok"].as<int>() != 0);
        packetReportsBatt = true;
      } else if (!doc["battery_low"].isNull()) {
        packetBattOk = (doc["battery_low"].as<int>() == 0);
        packetReportsBatt = true;
      } else if (!doc["battery_mV"].isNull()) {
        packetBattOk = (doc["battery_mV"].as<int>() >= 2200);
        packetReportsBatt = true;
      }

      uint32_t nowSec = epochNow();

      if (tireIdx != -1) {
        portENTER_CRITICAL(&tireDataMux);

        if (tireState[tireIdx].hasData && tireState[tireIdx].prevBarMs > 0) {
          uint32_t dtMs = nowMs - tireState[tireIdx].prevBarMs;
          float dBar = tireState[tireIdx].prevBar - bar;
          if (dBar >= 0.08f && dtMs <= 300000UL) {
            isSwLeak = true;
          }
          if (bar >= tireState[tireIdx].prevBar || dtMs > 300000UL) {
            tireState[tireIdx].prevBar = bar;
            tireState[tireIdx].prevBarMs = nowMs;
          }
        } else {
          tireState[tireIdx].prevBar = bar;
          tireState[tireIdx].prevBarMs = nowMs;
        }

        bool finalLeakAlert = isHwLeak || isSwLeak;

        if (packetReportsBatt) {
          if (!packetBattOk) {
            if (tireState[tireIdx].lowBattStreak < 255) tireState[tireIdx].lowBattStreak++;
            if (tireState[tireIdx].lowBattStreak >= 2) {
              tireState[tireIdx].batteryOk = false;
            }
          } else {
            tireState[tireIdx].lowBattStreak = 0;
            tireState[tireIdx].batteryOk = true;
          }
        } else {
          if (!tireState[tireIdx].hasData) {
            tireState[tireIdx].batteryOk = true;
          }
        }

        bool finalBattOk = tireState[tireIdx].batteryOk;
        tireState[tireIdx].hasData = true;
        tireState[tireIdx].bar = bar;
        tireState[tireIdx].psi = psi;
        tireState[tireIdx].temp = temp;
        tireState[tireIdx].status = status;
        tireState[tireIdx].isMoving = isMoving;
        tireState[tireIdx].isLeaking = finalLeakAlert;

        int lvl = computeLevel(tireIdx, bar, temp);
        if (finalLeakAlert) lvl = 3;
        bool belowNormal = baselineUpdateLocked(tireIdx, bar, temp, nowSec, !finalLeakAlert && lvl == 0);
        if (lvl == 0 && belowNormal) lvl = 4;   // ضمن الحدود لكن أقل من ضغطه الطبيعي المعتاد
        tireState[tireIdx].lvl = lvl;

        tireState[tireIdx].rssi = pkt.rssi;
        tireState[tireIdx].packetCount++;
        tireState[tireIdx].lastSeenEpoch = nowSec;
        portEXIT_CRITICAL(&tireDataMux);

        markTireDirty(tireIdx);
        maybeQueueHistory((uint8_t)tireIdx, bar, temp, pkt.rssi, (uint8_t)(status & 0xFF), finalBattOk, isMoving, finalLeakAlert, nowSec, nowMs);
      }

      bool scannedBattOk = packetReportsBatt ? packetBattOk : true;
      portENTER_CRITICAL(&tireDataMux);
      addOrUpdateScanned(idBuf, model, bar, temp, pkt.rssi, status, scannedBattOk, isMoving, isHwLeak, (tireIdx != -1), isToyota, nowSec);
      if (learnActive) learnAdd(idBuf, model, pkt.rssi, bar, temp, nowMs);
      if (autoSensorsMode != 1) smartFeed(idBuf, model, pkt.rssi, nowMs);
      strncpy(lastPktId, idBuf, sizeof(lastPktId) - 1);
      lastPktRssi = pkt.rssi;
      lastPktMs = nowMs;
      if (screenMode == 2 && pkt.rssi >= techThresholdDbm) {
        techConsider(idBuf, model, isnan(kpaIn) ? NAN : bar, tempIn, pkt.rssi, status,
                     packetReportsBatt ? (packetBattOk ? 1 : 0) : -1, tireIdx, doc["mic"] | "", nowMs);
      }
      portEXIT_CRITICAL(&tireDataMux);

      tireDataDirty = true;
      statusBarDirty = true;
    }

    if (millis() - lastFlush >= 250) {
      lastFlush = millis();
      flushDirtyTires(false);
    }
    taskYIELD();
  }
}

// ---------------- 8. واجهات الشاشة ST7789 المحسنة ----------------
void drawBaseCarLayout() {
  safeTftFillScreen(ST77XX_BLACK);
  safeTftFillRect(0, 0, 240, 24, 0x18E3);

  safeTftDrawRoundRect(102, 28, 36, 156, 10, 0xCE59);
  safeTftDrawFastHLine(104, 80, 32, 0x52AA);
  safeTftDrawFastHLine(104, 132, 32, 0x52AA);

  safeTftFillRect(98, 60, 3, 8, 0xCE59);
  safeTftFillRect(139, 60, 3, 8, 0xCE59);

  safeTftDrawRoundRect(4, 194, 232, 80, 6, 0x2965);
  safeTftDrawFastHLine(0, 292, 240, 0x4208);
}

void updateTopBar() {
  if (!topBarDirty) return;
  topBarDirty = false;

  if (tftMutex && xSemaphoreTake(tftMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    tft.fillRect(0, 0, 240, 24, 0x18E3);
    tft.setTextSize(1);
    if (wifiEnabled) {
      tft.setTextColor(ST77XX_GREEN, 0x18E3);
      tft.setCursor(8, 8);
      tft.printf("WiFi:ON  %s", deviceIPString.c_str());
    } else {
      tft.setTextColor(0x9CD3, 0x18E3);
      tft.setCursor(8, 8);
      tft.print("WiFi:OFF (Hold BOOT)");
    }
    xSemaphoreGive(tftMutex);
  }
}

void updateTireReadouts() {
  int textPos[4][2] = {
    {4,   28},   // FL
    {156, 28},   // FR
    {4,   110},  // RL
    {156, 110}   // RR
  };

  int tirePos[4][2] = {
    {88,  38}, {145, 38}, {88,  120}, {145, 120}
  };

  char ageBuf[16];
  uint32_t nowSec = epochNow();

  TireState localState[5];
  portENTER_CRITICAL(&tireDataMux);
  for (int i = 0; i < 5; i++) localState[i] = tireState[i];
  portEXIT_CRITICAL(&tireDataMux);

  for (int i = 0; i < 4; i++) {
    int x = textPos[i][0];
    int y = textPos[i][1];

    uint16_t color = ST77XX_GREEN;
    if (localState[i].lvl == 1) color = ST77XX_RED;
    else if (localState[i].lvl == 2) color = ST77XX_BLUE;
    else if (localState[i].lvl == 3) color = ST77XX_MAGENTA;
    else if (localState[i].lvl == 4) color = ST77XX_YELLOW;

    if (localState[i].hasData) {
      uint32_t age = nowSec - localState[i].lastSeenEpoch;
      bool isStale = (age > staleSignalTimeoutSec);
      if (isStale) color = COLOR_STALE; // إشارة منقطعة منذ مدة طويلة: لا يُعتمد على آخر قراءة صالحة
      safeTftFillRoundRect(tirePos[i][0], tirePos[i][1], 7, 30, 2, color);
      formatAge(age, ageBuf, sizeof(ageBuf));
      safeTftRenderTireText(x, y, myTires[i].label, localState[i].packetCount, localState[i].bar, localState[i].temp, localState[i].batteryOk, localState[i].lvl, localState[i].isMoving, ageBuf, localState[i].rssi, isStale);
    } else {
      if (tftMutex && xSemaphoreTake(tftMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        safeTftFillRoundRect(tirePos[i][0], tirePos[i][1], 7, 30, 2, 0x3186);
        tft.fillRect(x, y - 2, 80, 68, ST77XX_BLACK);

        tft.setTextSize(1);
        tft.setTextColor(0x7BEF, ST77XX_BLACK);
        tft.setCursor(x, y);
        tft.printf("[%s]", myTires[i].label);

        tft.setTextSize(3);
        tft.setTextColor(0x52AA, ST77XX_BLACK);
        tft.setCursor(x, y + 12);
        tft.print("---");

        tft.setTextSize(1);
        tft.setTextColor(0x7BEF, ST77XX_BLACK);
        tft.setCursor(x, y + 42);
        tft.print("WAITING");
        xSemaphoreGive(tftMutex);
      }
    }
  }

  if (tftMutex && xSemaphoreTake(tftMutex, pdMS_TO_TICKS(150)) == pdTRUE) {
    tft.fillRect(6, 196, 228, 76, ST77XX_BLACK);

    uint16_t spareColor = ST77XX_GREEN;
    if (localState[4].lvl == 1) spareColor = ST77XX_RED;
    else if (localState[4].lvl == 2) spareColor = ST77XX_BLUE;
    else if (localState[4].lvl == 3) spareColor = ST77XX_MAGENTA;
    else if (localState[4].lvl == 4) spareColor = ST77XX_YELLOW;

    uint32_t spAge = nowSec - localState[4].lastSeenEpoch;
    bool spareStale = localState[4].hasData && (spAge > staleSignalTimeoutSec);
    if (spareStale) spareColor = COLOR_STALE;

    if (localState[4].hasData) {
      tft.fillRoundRect(12, 208, 28, 14, 3, spareColor);
      tft.setTextSize(1);
      tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      tft.setCursor(10, 226);
      tft.print(myTires[4].label);

      tft.setTextSize(3);
      tft.setTextColor(spareColor, ST77XX_BLACK);
      tft.setCursor(50, 206);
      tft.printf("%.*f", pDecimals(), pToUnit(localState[4].bar));

      tft.setTextSize(1);
      tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      tft.setCursor(124, 222);
      tft.print(pUnit());

      tft.setTextSize(1);
      tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      tft.setCursor(154, 204);
      tft.printf("%2.0f %s", tToUnit(localState[4].temp), tUnit());

      tft.setCursor(154, 216);
      tft.print("BATT: ");
      tft.setTextColor(localState[4].batteryOk ? ST77XX_GREEN : ST77XX_RED, ST77XX_BLACK);
      tft.print(localState[4].batteryOk ? "OK" : "LOW");

      tft.setCursor(154, 228);
      if (spareStale) {
        tft.setTextColor(COLOR_STALE, ST77XX_BLACK);
        tft.print("NO SIGNAL!");
      } else if (localState[4].lvl == 3) {
        tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
        tft.print("! LEAK !");
      } else {
        tft.setTextColor(0x7BEF, ST77XX_BLACK);
        tft.print(localState[4].isMoving ? "ROLLING" : "PARKED");
      }

      formatAge(spAge, ageBuf, sizeof(ageBuf));
      tft.setTextColor(spareStale ? COLOR_STALE : ST77XX_YELLOW, ST77XX_BLACK);
      tft.setCursor(50, 248);
      tft.printf("AGE:%-4s #%-3u  RSSI:%3ddB", ageBuf, (unsigned int)localState[4].packetCount, localState[4].rssi);
    } else {
      tft.fillRoundRect(12, 218, 28, 14, 3, 0x3186);
      tft.setTextSize(1);
      tft.setTextColor(0x7BEF, ST77XX_BLACK);
      tft.setCursor(10, 236);
      tft.print(myTires[4].label);

      tft.setCursor(60, 226);
      tft.print("SPARE: SLEEP / WAITING");
    }
    xSemaphoreGive(tftMutex);
  }
}

void updateStatusBar() {
  if (tftMutex && xSemaphoreTake(tftMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    tft.fillRect(0, 294, 240, 26, ST77XX_BLACK);
    tft.setTextSize(1);
    tft.setCursor(8, 302);
    if (isCC1101Ok) {
      tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
      tft.printf("ONLINE #%-4u BW%d d%d N%d", rfPacketCounter, (int)currentRxBwKhz(), rssiDeltaCurrent, rtl_433_ESP::averageRssi);
    } else {
      tft.setTextColor(ST77XX_RED, ST77XX_BLACK);
      tft.printf("CC1101: FAIL err %-2d", cc1101ErrCode);
    }
    xSemaphoreGive(tftMutex);
  }
}

void drawScannerLayout() {
  if (tftMutex && xSemaphoreTake(tftMutex, pdMS_TO_TICKS(150)) == pdTRUE) {
    tft.fillScreen(ST77XX_BLACK);
    tft.fillRect(0, 0, 240, 24, 0x18E3);

    tft.setTextSize(1);
    tft.setTextColor(ST77XX_WHITE, 0x18E3);
    tft.setCursor(8, 8);
    tft.print("SCANNER | ALL SELECTED TPMS");

    tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
    tft.setCursor(4, 30);
    tft.print("ID       Bar   Temp BATT RSSI  #Cnt");
    tft.drawFastHLine(0, 40, 240, 0x4208);
    xSemaphoreGive(tftMutex);
  }
}

void updateScannerValues() {
  char ageBuf[12];
  uint32_t nowSec = epochNow();

  ScannedSensor localList[MAX_SCANNED];
  int count = 0;
  portENTER_CRITICAL(&tireDataMux);
  count = min(scannedCount, 10);
  for (int i = 0; i < count; i++) localList[i] = scannedList[i];
  portEXIT_CRITICAL(&tireDataMux);

  if (tftMutex && xSemaphoreTake(tftMutex, pdMS_TO_TICKS(150)) == pdTRUE) {
    for (int i = 0; i < count; i++) {
      int y = 46 + (i * 24);
      tft.fillRect(0, y, 240, 20, ST77XX_BLACK);
      uint16_t color = localList[i].isMine ? ST77XX_GREEN : (localList[i].isToyota ? 0xFD20 : ST77XX_WHITE);
      uint32_t age = nowSec - localList[i].lastSeenEpoch;
      formatAge(age, ageBuf, sizeof(ageBuf));

      tft.setTextSize(1);
      tft.setTextColor(color, ST77XX_BLACK);
      tft.setCursor(4, y);
      char pbuf[8];
      snprintf(pbuf, sizeof(pbuf), "%.*f%c", pDecimals(), pToUnit(localList[i].bar), unitPressure == 0 ? 'B' : unitPressure == 1 ? 'P' : 'k');
      tft.printf("%-7s %-6s %.0f%s %-3s %3ddB #%-2u",
                 localList[i].id,
                 pbuf,
                 tToUnit(localList[i].temp), tUnit(),
                 localList[i].batteryOk ? "OK" : "LOW",
                 localList[i].rssi,
                 (unsigned int)localList[i].packetCount);
    }

    if (count == 0) {
      tft.setTextColor(0x7BEF, ST77XX_BLACK);
      tft.setCursor(30, 140);
      tft.print("Listening for sensors...");
    }
    xSemaphoreGive(tftMutex);
  }
}

// ---------------- 8.5 شاشة فني الإطارات (240x320 عمودية) ----------------
void drawTechLayout() {
  if (tftMutex && xSemaphoreTake(tftMutex, pdMS_TO_TICKS(150)) == pdTRUE) {
    tft.fillScreen(ST77XX_BLACK);
    tft.fillRect(0, 0, 240, 24, 0x780F);
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_WHITE, 0x780F);
    tft.setCursor(8, 8);
    tft.printf("TIRE TECH | NEAR >= %d dBm", techThresholdDbm);
    xSemaphoreGive(tftMutex);
  }
}

// full=true يعيد رسم كل شيء، وإلا يحدّث الأجزاء المتغيرة فقط (المقياس/العمر)
void updateTechValues(bool full) {
  TechCapture c;
  TechCapture prev;
  bool hasPrev;
  portENTER_CRITICAL(&tireDataMux);
  c = techCap;
  hasPrev = techHistCount > 0;
  if (hasPrev) prev = techHist[0];
  portEXIT_CRITICAL(&tireDataMux);
  uint32_t nowMs = millis();
  int live = rtl_433_ESP::currentRssi;

  if (!tftMutex || xSemaphoreTake(tftMutex, pdMS_TO_TICKS(150)) != pdTRUE) return;

  if (!c.valid) {
    if (full) {
      tft.fillRect(0, 25, 240, 268, ST77XX_BLACK);
      tft.setTextSize(2);
      tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
      tft.setCursor(48, 44);  tft.print("BRING THE");
      tft.setCursor(36, 66);  tft.print("DEVICE CLOSE");
      tft.setCursor(48, 88);  tft.print("TO THE TIRE");
      tft.setTextSize(1);
      tft.setTextColor(0x9CD3, ST77XX_BLACK);
      tft.setCursor(8, 122);  tft.print("Wake the sensor: deflate a little,");
      tft.setCursor(8, 134);  tft.print("rotate the wheel or use an LF tool.");
      tft.setTextColor(ST77XX_MAGENTA, ST77XX_BLACK);
      tft.setCursor(8, 156);  tft.printf("CAPTURE AT >= %d dBm", techThresholdDbm);
      if (hasPrev) {
        tft.setTextColor(0x7BEF, ST77XX_BLACK);
        tft.setCursor(8, 232);
        tft.printf("LAST: %s", prev.id);
        tft.setCursor(8, 244);
        if (!isnan(prev.bar)) tft.printf("%.2f BAR  %s", prev.bar, prev.model);
        else                  tft.print(prev.model);
      }
    }
    // مقياس قرب حي (-110..-30 dBm)
    const int mx = 8, my = 172, mw = 224, mh = 16;
    int v = constrain(live, -110, -30);
    int px = (int)((long)(v + 110) * (mw - 2) / 80);
    int tx = mx + (int)((long)(constrain(techThresholdDbm, -110, -30) + 110) * mw / 80);
    tft.drawRect(mx, my, mw, mh, 0x4208);
    tft.fillRect(mx + 1, my + 1, px, mh - 2, live >= techThresholdDbm ? ST77XX_GREEN : 0x04FF);
    tft.fillRect(mx + 1 + px, my + 1, mw - 2 - px, mh - 2, ST77XX_BLACK);
    tft.drawFastVLine(tx, my - 3, mh + 6, ST77XX_MAGENTA);
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setCursor(8, 194);
    tft.printf("LIVE %4d dBm  ", live);
    xSemaphoreGive(tftMutex);
    return;
  }

  uint16_t vColor;
  const char* verdict = techVerdict(c, &vColor, nullptr);
  bool flash = (int32_t)(techFlashUntil - nowMs) > 0;
  char age[12];
  formatAge((nowMs - c.lastSeenMs) / 1000, age, sizeof(age));

  if (full) {
    tft.fillRect(0, 25, 240, 268, ST77XX_BLACK);
    if (flash) {
      tft.drawRect(0, 25, 240, 268, ST77XX_GREEN);
      tft.drawRect(1, 26, 238, 266, ST77XX_GREEN);
    }
    tft.setTextSize(1);
    tft.setTextColor(0x7BEF, ST77XX_BLACK);
    tft.setCursor(8, 32);
    tft.print("SENSOR ID");
    tft.setTextColor(0x04FF, ST77XX_BLACK);
    tft.setCursor(80, 32);
    tft.printf("%.22s", c.model);
    if (c.tire >= 0) {
      tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
      tft.setCursor(190, 44);
      tft.printf("MY %s", myTires[c.tire].label);
    }
    tft.setTextSize(3);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setCursor(8, 44);
    tft.printf("%.10s", c.id);

    tft.setTextSize(5);
    tft.setTextColor(vColor, ST77XX_BLACK);
    tft.setCursor(8, 80);
    if (!isnan(c.bar)) tft.printf("%.*f", pDecimals(), pToUnit(c.bar));
    else               tft.print("--.-");
    tft.setTextSize(2);
    tft.setTextColor(0x7BEF, ST77XX_BLACK);
    tft.setCursor(170, 98);
    tft.print(pUnit());

    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setCursor(8, 126);
    float cold = coldPressureBar(c.bar, c.temp);
    if (!isnan(cold)) tft.printf("@20C %.*f", pDecimals(), pToUnit(cold));
    else              tft.print("@20C --");
    tft.setCursor(170, 126);
    if (!isnan(c.temp)) tft.printf("%d%s", (int)lroundf(tToUnit(c.temp)), tUnit());
    else                tft.print("--");

    tft.setTextSize(1);
    tft.setCursor(8, 150);
    tft.printf("SIGNAL %d dBm (BEST %d)", c.rssi, c.bestRssi);
    tft.setCursor(8, 162);
    tft.printf("BATTERY: %s   CHECK: %s", c.battery == 1 ? "OK" : c.battery == 0 ? "LOW" : "N/A", c.mic[0] ? c.mic : "-");
    if (c.status >= 0) { tft.setCursor(8, 174); tft.printf("STATUS: 0x%02X", c.status & 0xFF); }

    uint16_t boxBg = (vColor == ST77XX_GREEN) ? 0x0320 : (vColor == ST77XX_RED ? 0x5000 : 0x4A00);
    tft.fillRoundRect(8, 196, 224, 30, 6, boxBg);
    tft.drawRoundRect(8, 196, 224, 30, 6, vColor);
    tft.setTextSize(2);
    tft.setTextColor(vColor, boxBg);
    int tw = strlen(verdict) * 12;
    tft.setCursor((240 - tw) / 2, 204);
    tft.print(verdict);

    if (hasPrev) {
      tft.setTextSize(1);
      tft.setTextColor(0x7BEF, ST77XX_BLACK);
      tft.setCursor(8, 240);
      tft.printf("PREV: %s", prev.id);
      tft.setCursor(8, 252);
      if (!isnan(prev.bar)) tft.printf("%.2f BAR  %s", prev.bar, prev.model);
      else                  tft.print(prev.model);
    }
  }
  // سطر متغير كل ثانية: عدد الحزم وعمر آخر قراءة
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
  tft.setCursor(8, 272);
  tft.printf("PACKETS %-4lu  SEEN %-5s ago   ", (unsigned long)c.count, age);
  xSemaphoreGive(tftMutex);
}

// ---------------- 8.6 شاشة تشخيص الراديو (توجيه الهوائي) ----------------
void drawRadioLayout() {
  if (tftMutex && xSemaphoreTake(tftMutex, pdMS_TO_TICKS(150)) == pdTRUE) {
    tft.fillScreen(ST77XX_BLACK);
    tft.fillRect(0, 0, 240, 24, 0x0210);
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_WHITE, 0x0210);
    tft.setCursor(8, 8);
    tft.print("RADIO DIAG | ANTENNA AIMING");
    tft.setTextColor(0x7BEF, ST77XX_BLACK);
    tft.setCursor(8, 32);
    tft.print("LIVE RSSI (PEAK 150ms)");
    xSemaphoreGive(tftMutex);
  }
}

void updateRadioValues(int peakRssi) {
  rtl_433_ESPStatus st = rtl_433_ESP::statusSnapshot();
  char lastId[24];
  int lastRssi;
  uint32_t lastMs;
  portENTER_CRITICAL(&tireDataMux);
  memcpy(lastId, lastPktId, sizeof(lastId));
  lastRssi = lastPktRssi;
  lastMs = lastPktMs;
  portEXIT_CRITICAL(&tireDataMux);
  if (!tftMutex || xSemaphoreTake(tftMutex, pdMS_TO_TICKS(100)) != pdTRUE) return;

  tft.setTextSize(4);
  tft.setTextColor(peakRssi > st.rssiThreshold ? ST77XX_GREEN : ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(8, 46);
  tft.printf("%4d", peakRssi);
  tft.setTextSize(2);
  tft.setTextColor(0x7BEF, ST77XX_BLACK);
  tft.setCursor(110, 60);
  tft.print("dBm");

  const int mx = 8, my = 86, mw = 224, mh = 18;
  auto toX = [&](int dbm) -> int { int v = constrain(dbm, -110, -30); return mx + (int)((long)(v + 110) * (mw - 1) / 80); };
  int px = toX(peakRssi);
  tft.drawRect(mx, my, mw, mh, 0x4208);
  tft.fillRect(mx + 1, my + 1, px - mx, mh - 2, peakRssi > st.rssiThreshold ? ST77XX_GREEN : 0x04FF);
  tft.fillRect(px + 1, my + 1, mx + mw - 2 - px, mh - 2, ST77XX_BLACK);
  tft.drawFastVLine(toX(st.averageRssi), my + 1, mh - 2, 0x7BEF);
  tft.drawFastVLine(toX(st.rssiThreshold), my + 1, mh - 2, 0xFD20);
  tft.setTextSize(1);
  tft.setTextColor(0x7BEF, ST77XX_BLACK);
  tft.setCursor(mx, my + mh + 3);  tft.print("-110");
  tft.setCursor(mx + mw - 18, my + mh + 3); tft.print("-30");
  tft.setTextColor(0xFD20, ST77XX_BLACK);
  tft.setCursor(80, my + mh + 3); tft.print("|=THRESHOLD");

  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(8, 128); tft.printf("NOISE %4d dBm   THR %4d dBm  ", st.averageRssi, st.rssiThreshold);
  tft.setCursor(8, 142); tft.printf("FREQ %.3f MHz  BW %d kHz  ", currentFreqMhz(), (int)currentRxBwKhz());
  tft.setCursor(8, 156); tft.printf("DELTA %d dB   OFFSET %+d kHz  ", rssiDeltaCurrent, freqOffsetKhz);
  tft.setTextColor(0x04FF, ST77XX_BLACK);
  tft.setCursor(8, 176); tft.printf("CAPTURED %-6u DECODED %-6u", st.capturedSignals, st.decodedMessages);
  tft.setCursor(8, 190); tft.printf("UNKNOWN %-6u LOST %u/%u/%lu  ", st.zeroDecodedSignals, st.droppedCaptureBuffers,
                                    st.droppedDecoderQueue, (unsigned long)rfQueueDropCounter);
  tft.setCursor(8, 204); tft.printf("REJECTED %-5lu RADIO RESETS %-4lu", (unsigned long)rfRejectedCounter, (unsigned long)ccRecoveryCounter);
  tft.setCursor(8, 224);
  if (lastMs) {
    char age[12];
    formatAge((millis() - lastMs) / 1000, age, sizeof(age));
    tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    tft.printf("LAST %-10.10s %4ddBm %-4s ago  ", lastId, lastRssi, age);
  } else {
    tft.setTextColor(0x7BEF, ST77XX_BLACK);
    tft.print("LAST: NO PACKET YET");
  }
  tft.setTextColor(0x7BEF, ST77XX_BLACK);
  tft.setCursor(8, 248); tft.printf("HEAP %u KB  UP %lus   ", ESP.getFreeHeap() / 1024, (unsigned long)(millis() / 1000));
  xSemaphoreGive(tftMutex);
}

// ---------------- 8.7 شاشة تعلّم الإطارات ----------------
void drawLearnScreen(bool full) {
  LearnCandidate local[5];
  int n;
  bool active = learnActive;
  portENTER_CRITICAL(&tireDataMux);
  if (!active) learnSortLocked();
  n = min(learnCandCount, 5);
  for (int i = 0; i < n; i++) local[i] = learnCand[i];
  if (active) {
    // ترتيب النسخة المحلية فقط أثناء التعلّم
    for (int i = 1; i < n; i++) { LearnCandidate k = local[i]; int j = i - 1; while (j >= 0 && learnScore(local[j]) < learnScore(k)) { local[j + 1] = local[j]; j--; } local[j + 1] = k; }
  }
  portEXIT_CRITICAL(&tireDataMux);
  if (!tftMutex || xSemaphoreTake(tftMutex, pdMS_TO_TICKS(150)) != pdTRUE) return;
  if (full) {
    tft.fillScreen(ST77XX_BLACK);
    tft.fillRect(0, 0, 240, 24, 0x03E0);
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_BLACK, 0x03E0);
    tft.setCursor(8, 8);
    tft.print(active ? "LEARNING YOUR TIRES..." : "LEARN RESULT (CONFIRM ON WEB)");
  }
  tft.setTextSize(2);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(8, 34);
  if (active) {
    uint32_t left = (LEARN_DURATION_MS - min(LEARN_DURATION_MS, millis() - learnStartMs)) / 1000;
    tft.printf("DRIVE  %lu:%02lu   ", (unsigned long)(left / 60), (unsigned long)(left % 60));
  } else {
    tft.print("TOP CANDIDATES  ");
  }
  tft.setTextSize(1);
  tft.setTextColor(0x7BEF, ST77XX_BLACK);
  tft.setCursor(8, 60);
  tft.print("ID         BURSTS  AVG dBm  PRESS");
  for (int i = 0; i < 5; i++) {
    int y = 76 + i * 22;
    tft.fillRect(0, y - 2, 240, 18, ST77XX_BLACK);
    if (i >= n) continue;
    int avg = local[i].packets ? (int)(local[i].rssiSum / (int32_t)local[i].packets) : 0;
    tft.setTextColor(local[i].bursts >= 3 ? ST77XX_GREEN : ST77XX_WHITE, ST77XX_BLACK);
    tft.setCursor(8, y);
    tft.printf("%-10.10s %4u    %4d     %.*f", local[i].id, local[i].bursts, avg, pDecimals(), pToUnit(local[i].bar));
  }
  tft.setTextColor(0x7BEF, ST77XX_BLACK);
  tft.setCursor(8, 196);
  tft.print(active ? "Your tires repeat with strong signal." : "Open /learn on the web page to");
  tft.setCursor(8, 208);
  tft.print(active ? "Passing cars appear only once." : "assign positions. Press BOOT to exit.");
  xSemaphoreGive(tftMutex);
}

// ---------------- 8.8 شاشة اقتراح الحساسات (التعلّم الذاتي) ----------------
void drawSmartPrompt(bool full) {
  SmartProposal p;
  portENTER_CRITICAL(&tireDataMux);
  p = smartProposal;
  portEXIT_CRITICAL(&tireDataMux);
  if (!tftMutex || xSemaphoreTake(tftMutex, pdMS_TO_TICKS(150)) != pdTRUE) return;
  if (full) {
    tft.fillScreen(ST77XX_BLACK);
    tft.fillRect(0, 0, 240, 24, 0x03EF);
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_BLACK, 0x03EF);
    tft.setCursor(8, 8);
    tft.print("SELF-LEARNING: NEW SENSORS");
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setCursor(8, 36);
    tft.print("DETECTED:");
    tft.setTextSize(1);
    for (int i = 0; i < p.count; i++) {
      int sl = p.slot[i];
      tft.setTextColor(ST77XX_YELLOW, ST77XX_BLACK);
      tft.setCursor(8, 66 + i * 26);
      tft.printf("%-6s", myTires[sl].label);
      tft.setTextColor(0x7BEF, ST77XX_BLACK);
      tft.printf(" %-10.10s ->", myTires[sl].shortId[0] ? myTires[sl].shortId : "(empty)");
      tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
      tft.setCursor(8, 78 + i * 26);
      tft.printf("       %s", p.id[i]);
    }
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    tft.setCursor(8, 216);
    tft.print("PRESS BOOT");
    tft.setCursor(8, 236);
    tft.print("TO APPLY");
    tft.setTextSize(1);
    tft.setTextColor(0x7BEF, ST77XX_BLACK);
    tft.setCursor(8, 262);
    tft.print("Ignore it, or dismiss on the web page.");
  }
  uint32_t left = (int32_t)(smartPromptUntil - millis()) > 0 ? (smartPromptUntil - millis()) / 1000 : 0;
  tft.setTextSize(1);
  tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
  tft.setCursor(150, 238);
  tft.printf("%2lus ", (unsigned long)left);
  xSemaphoreGive(tftMutex);
}

// ---------------- 9. معالجة زر التحكم الذكي (BOOT Button) ----------------
unsigned long buttonPressTime = 0;
bool buttonIsPressed = false;
bool longPressTriggered = false;

void handleSmartButton() {
  int reading = digitalRead(BOOT_BUTTON_PIN);

  if (reading == LOW && !buttonIsPressed) {
    buttonPressTime = millis();
    buttonIsPressed = true;
    longPressTriggered = false;
  }
  else if (reading == LOW && buttonIsPressed) {
    // إمساك 5 ثوانٍ: بدء تعلّم الإطارات (صفير قصير كتأكيد أثناء الإمساك)
    if (!longPressTriggered && (millis() - buttonPressTime >= 5000)) {
      longPressTriggered = true;
      learnStart();
      lastDrawnScreenMode = -1;
      pendingChirps = 1;
    }
  }
  else if (reading == HIGH && buttonIsPressed) {
    unsigned long pressDuration = millis() - buttonPressTime;
    buttonIsPressed = false;

    if (longPressTriggered) {
      // التعلّم بدأ أثناء الإمساك
    } else if (pressDuration >= 800) {
      toggleWifiAP();
      topBarDirty = true;
    } else if (pressDuration >= 60) {
      if (alarmActive && !alarmAcked) {
        alarmAcked = true;           // أول ضغطة أثناء الإنذار: إسكات فقط
        beepRemaining = 0;
        digitalWrite(BUZZER_PIN, LOW);
      } else if (smartProposal.valid && (int32_t)(smartPromptUntil - millis()) > 0) {
        smartApplyProposal(false);   // تأكيد اقتراح الحساسات الجديدة
      } else if (learnDone && !learnActive) {
        learnDone = false;           // الخروج من شاشة نتيجة التعلّم
        lastDrawnScreenMode = -1;
      } else {
        screenMode = (screenMode + 1) % SCREEN_MODE_COUNT; // السيارة -> الماسح -> الفني -> الراديو
      }
    }
  }
}

// ---------------- 10. خادم الويب بنظام البث المباشر (Chunked Streaming) ----------------
// إرسال دفعة من الصفحة مع تغذية مؤقت الحماية: هاتف بإشارة ضعيفة قد يجعل كل دفعة
// تنتظر ثوانٍ، ومجموعها كان يتجاوز مهلة الـ Watchdog فيُعاد تشغيل الجهاز أثناء التحميل.
// تجميع قطع الصفحة في مخزن ~1.4KB قبل الإرسال: كانت الصفحة تُرسل في ~115 قطعة صغيرة
// تنتظر كل منها تأكيد الهاتف، فتستغرق 3-16 ثانية وقد يستسلم المتصفح. الآن ~8 رزم فقط.
static String webBuf;
static const size_t WEB_FLUSH_AT = 1400;
static void webFlush() {
  if (webBuf.length()) { esp_task_wdt_reset(); server.sendContent(webBuf); webBuf = ""; }
}
static void webSend(const char* s) {
  if (!s || !s[0]) {               // "" = نهاية الصفحة (chunked): إفراغ المخزن ثم إنهاء الإرسال
    webFlush();
    server.sendContent("");
    return;
  }
  if (webBuf.length() == 0) webBuf.reserve(WEB_FLUSH_AT + 512);
  webBuf += s;
  if (webBuf.length() >= WEB_FLUSH_AT) webFlush();
}
static inline void webSend(const String& s) { webSend(s.c_str()); }

String htmlEscape(const String& in) {
  String out;
  out.reserve(in.length() + 8);
  for (size_t i = 0; i < in.length(); i++) {
    char c = in.charAt(i);
    switch (c) {
      case '&':  out += "&amp;";  break;
      case '<':  out += "&lt;";   break;
      case '>':  out += "&gt;";   break;
      case '"':  out += "&quot;"; break;
      case '\'': out += "&#39;";  break;
      default:   out += c;        break;
    }
  }
  return out;
}
String htmlEscape(const char* in) { return htmlEscape(String(in ? in : "")); }

// إصلاح أمان (CSV/Formula Injection): تُدرَج تسمية الإطار (نص حر يدخله
// المستخدم، حتى العربية) في ملف CSV المُصدَّر بلا تنقية سابقاً. لو بدأت
// التسمية بـ = أو + أو - أو @ يفسّرها Excel/Sheets كصيغة قابلة للتنفيذ عند
// الفتح (فئة ثغرات معروفة: CSV Injection / Formula Injection)، وغياب أي
// اقتباس يعني أن فاصلة واحدة داخل التسمية تكسر بنية الأعمدة بالكامل.
// الحل القياسي: اقتباس الحقل دائماً بعلامتي تنصيص (مع مضاعفة أي علامة
// تنصيص داخلية كما تنص RFC 4180)، وإضافة علامة اقتباس أحادية (') بادئة إن
// بدأ الحقل بأحد الرموز الأربعة أعلاه، فتُجبر البرنامج على معاملته كنص حرفي.
String csvSafeField(const char* in) {
  String s(in ? in : "");
  if (s.length() > 0) {
    char c0 = s.charAt(0);
    if (c0 == '=' || c0 == '+' || c0 == '-' || c0 == '@') s = "'" + s;
  }
  s.replace("\"", "\"\"");
  return "\"" + s + "\"";
}

String csrfToken;

String generateCsrfToken() {
  uint8_t raw[16];
  for (int i = 0; i < 16; i++) raw[i] = (uint8_t)(esp_random() & 0xFF);
  char hex[33];
  for (int i = 0; i < 16; i++) snprintf(hex + (i * 2), 3, "%02X", raw[i]);
  hex[32] = '\0';
  return String(hex);
}

String actionUrl(const String& query) {
  return "/action?" + query + "&csrf=" + csrfToken;
}

String csrfHiddenField() {
  return "<input type='hidden' name='csrf' value='" + csrfToken + "'>";
}

void streamPageHeader(const char* title, int refreshSec = 0) {
  webBuf = "";
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html; charset=utf-8", "");

  webSend(
    "<!DOCTYPE html><html lang='ar' dir='rtl'><head><meta charset='UTF-8'>"
    "<meta name='viewport' content='width=device-width, initial-scale=1, maximum-scale=1'>"
  );
  if (refreshSec > 0) {
    webSend("<meta http-equiv='refresh' content='" + String(refreshSec) + "'>");
  }
  webSend("<title>" + String(title) + "</title><style>"
    "*{box-sizing:border-box}"
    "body{font-family:system-ui,-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;background:#0d1117;color:#e6edf3;margin:0;padding:0}"
    "header{background:#161b22;padding:16px;text-align:center;border-bottom:1px solid #30363d}"
    "header h1{margin:0;font-size:18px;color:#58a6ff;font-weight:600}"
    "nav{display:flex;background:#161b22;border-bottom:1px solid #30363d;overflow-x:auto}"
    "nav a{flex:1;text-align:center;padding:12px 10px;color:#8b949e;text-decoration:none;font-size:14px;font-weight:500;white-space:nowrap;border-bottom:2px solid transparent}"
    "nav a:hover,nav a.active{color:#58a6ff;border-bottom:2px solid #58a6ff}"
    ".wrap{padding:14px;max-width:720px;margin:0 auto}"
    ".card{background:#161b22;border-radius:8px;padding:14px;margin-bottom:12px;border:1px solid #30363d}"
    ".grid{display:grid;grid-template-columns:1fr 1fr;gap:12px}"
    "@media(max-width:380px){.grid{grid-template-columns:1fr}}"
    ".dot{display:inline-block;width:10px;height:10px;border-radius:50%;margin-right:6px}"
    ".g{background:#238636}.r{background:#da3633}.b{background:#1f6feb}.m{background:#d2a8ff}.y{background:#d29922}"
    // ---- بطاقات الإطارات المُعاد تصميمها (أكثر بصرية وجاذبية) ----
    ".tcard{position:relative;border-radius:14px;overflow:hidden;border:1px solid #30363d;"
    "background:linear-gradient(160deg,#161b22,#10141a);transition:transform .15s}"
    ".tcard-head{display:flex;justify-content:space-between;align-items:center;padding:9px 12px;"
    "background:rgba(255,255,255,.04);border-bottom:1px solid #30363d}"
    ".tcard-label{font-size:15px;font-weight:800;letter-spacing:.5px}"
    ".tcard-sub{font-size:11px;color:#8b949e}"
    ".tcard-body{padding:12px}"
    ".tcard-pressure{font-size:32px;font-weight:800;line-height:1.15;display:flex;align-items:baseline;gap:6px}"
    ".tcard-pressure small{font-size:12px;font-weight:600;color:#8b949e}"
    ".tcard-sub2{font-size:12px;color:#8b949e;margin-top:2px}"
    ".tcard-stats{display:grid;grid-template-columns:1fr 1fr;gap:6px;margin-top:10px}"
    ".tcard-stat{display:flex;align-items:center;gap:5px;background:rgba(255,255,255,.04);"
    "border-radius:8px;padding:6px 8px;font-size:12px}"
    ".tcard-meta{margin-top:8px;color:#8b949e}"
    ".tcard.ok{border-color:#2ea04355}.tcard.ok .tcard-pressure{color:#3fb950}"
    ".tcard.danger{border-color:#da363355}.tcard.danger .tcard-pressure{color:#f85149}"
    ".tcard.info{border-color:#1f6feb55}.tcard.info .tcard-pressure{color:#58a6ff}"
    ".tcard.idle{opacity:.5}.tcard.idle .tcard-pressure{color:#6e7681}"
    ".tcard.warn{border-color:#d29922aa;animation:pulseWarn 2.2s ease-in-out infinite}"
    ".tcard.warn .tcard-pressure{color:#d29922}"
    "@keyframes pulseWarn{0%,100%{box-shadow:0 0 0 0 rgba(210,153,34,.35)}50%{box-shadow:0 0 0 6px rgba(210,153,34,0)}}"
    ".banner{margin:0 0 8px 0;padding:8px 10px;border-radius:8px;font-size:12px;font-weight:700}"
    ".banner.warn{background:#d29922;color:#1b1400}"
    ".banner.danger{background:#da3633;color:#fff;animation:pulseBg 1.4s ease-in-out infinite}"
    "@keyframes pulseBg{0%,100%{filter:brightness(1)}50%{filter:brightness(1.3)}}"
    "table{width:100%;border-collapse:collapse;font-size:13px}"
    "th,td{padding:10px 8px;border-bottom:1px solid #21262d;text-align:right}"
    "th{color:#58a6ff;font-weight:600}"
    ".btn{display:inline-flex;align-items:center;justify-content:center;padding:8px 14px;background:#238636;color:#fff;border-radius:6px;text-decoration:none;border:none;cursor:pointer;font-size:13px;font-weight:500}"
    ".btn.blue{background:#1f6feb}"
    ".btn.r{background:#da3633;color:#fff}"
    ".btn.on{background:#238636}.btn.off{background:#21262d;color:#8b949e;border:1px solid #30363d}"
    ".row{display:flex;align-items:center;justify-content:space-between;margin-bottom:10px;flex-wrap:wrap;gap:8px}"
    ".tabs-container{display:flex;background:#0d1117;border-radius:6px;padding:3px;margin-bottom:14px;border:1px solid #30363d;gap:4px;overflow-x:auto}"
    ".tabs-container a{flex:1;text-align:center;padding:8px 4px;border-radius:5px;color:#8b949e;text-decoration:none;font-size:13px;font-weight:500;white-space:nowrap;transition:0.2s}"
    ".tabs-container a.sel{background:#1f6feb;color:#ffffff;box-shadow:0 1px 3px rgba(0,0,0,0.3)}"
    "input[type='text'],input[type='password'],input[type='datetime-local']{width:100%;padding:9px;border-radius:6px;border:1px solid #30363d;background:#0d1117;color:#fff}"
    "small{color:#8b949e}"
    "</style></head><body>"
    "<header><h1>نظام TPMS الاحترافي V4</h1></header>"
    "<nav><a href='/'>الرئيسية</a><a href='/scanner'>الماسح</a><a href='/tech'>🔧 فني الإطارات</a><a href='/chart'>📈 المنحنيات</a><a href='/history'>السجل</a><a href='/settings'>الإعدادات</a></nav>"
    "<div class='wrap'>"
  );
}

void streamPageFooter() {
  webSend("</div></body></html>");
  webSend("");
}

void handleRoot() {
  uint32_t nowSec = epochNow();
  DateTime dt(nowSec);
  char clockBuf[24];
  if (rtcOk) {
    snprintf(clockBuf, sizeof(clockBuf), "%04d-%02d-%02d %02d:%02d:%02d",
             dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second());
  } else {
    snprintf(clockBuf, sizeof(clockBuf), "RTC غير متصل");
  }
  unsigned long upSec = millis() / 1000UL;
  char upBuf[16];
  snprintf(upBuf, sizeof(upBuf), "%02lu:%02lu:%02lu", upSec / 3600, (upSec % 3600) / 60, upSec % 60);

  streamPageHeader("TPMS - الرئيسية", 10);

  webSend("<div class='card row'>"
                     "<div><b>الوقت الحالي (RTC):</b> " + String(clockBuf) + "</div>"
                     "<div><b>مدة التشغيل:</b> " + String(upBuf) + "</div></div>");

  webSend("<div class='card row'>"
                     "<div><b>CC1101:</b> " + String(isCC1101Ok ? "متصل ✅" : "غير متصل ❌") +
                     " | الانحراف: <b style='color:#58a6ff'><bdi dir='ltr'>" + String(freqDevCurrent) + " kHz</bdi></b>" +
                     " | حزم الراديو: <bdi dir='ltr'>" + String(rfPacketCounter) + "</bdi></div>"
                     "<div><b>FRAM:</b> " + String(framOk ? "متصلة ✅" : "غير متصلة ❌") +
                     " | <b>المعالج:</b> <bdi dir='ltr'>" + String(getCpuFrequencyMhz()) + " MHz</bdi></div></div>");

  webSend("<div class='card row'><div><b>🚗 السيارة النشطة:</b> " + htmlEscape(activeProfileName()) +
                     "</div><a class='btn off' style='padding:4px 10px;font-size:12px' href='/settings?view=profiles'>تبديل</a></div>");
  webSend("<div class='card'><div class='row'><b>عرض الشاشة: "
                     + String(screenMode == 0 ? "واجهة السيارة (تويوتا فقط)" : screenMode == 2 ? "فني الإطارات (الحساس الأقرب)" : screenMode == 3 ? "تشخيص الراديو (توجيه الهوائي)" : "قائمة الماسح (كافة الديكودرات)") + "</b></div>"
                     "<div style='display:flex;gap:8px;flex-wrap:wrap'>"
                     "<a class='btn " + String(screenMode == 0 ? "on" : "blue") + "' style='flex:1' href='" + actionUrl("do=screen&mode=cards") + "'>واجهة السيارة</a>"
                     "<a class='btn " + String(screenMode == 1 ? "on" : "blue") + "' style='flex:1' href='" + actionUrl("do=screen&mode=scanner") + "'>قائمة الماسح</a>"
                     "<a class='btn " + String(screenMode == 2 ? "on" : "blue") + "' style='flex:1' href='" + actionUrl("do=screen&mode=tech") + "'>فني الإطارات</a>"
                     "<a class='btn " + String(screenMode == 3 ? "on" : "blue") + "' style='flex:1' href='" + actionUrl("do=screen&mode=radio") + "'>تشخيص الراديو</a>"
                     "</div></div>");
  if (learnActive || learnDone) {
    webSend("<div class='card' style='border:1px solid #2ea043'><b>🧭 تعلّم الإطارات " + String(learnActive ? "جارٍ الآن — استمر بالقيادة" : "انتهى") +
                       "</b> <a class='btn' style='margin-right:8px' href='/learn'>عرض النتائج</a></div>");
  }
  if (smartProposal.valid) {
    webSend("<div class='card' style='border:1px solid #d29922'><b>🧠 اكتشف الجهاز حساساً جديداً لسيارتك</b> "
                       "<a class='btn' style='margin-right:8px' href='/settings?view=smart'>مراجعة</a></div>");
  }
  if (alarmActive) {
    webSend(String("<div class='card' style='background:#da3633;color:#fff;font-weight:700'>⚠️ إنذار إطار نشط") +
                       (alarmAcked ? " (تم الإسكات)" : "") + "</div>");
  }

  webSend("<div class='grid'>");
  portENTER_CRITICAL(&tireDataMux);
  TireState localTires[5];
  for (int i = 0; i < 5; i++) localTires[i] = tireState[i];
  portEXIT_CRITICAL(&tireDataMux);

  // كان buf[400] كافياً قبل إضافة وسوم <bdi dir='ltr'> أدناه لعزل الأرقام
  // عن اتجاه الفقرة العربية. أضافت هذه الوسوم (فتح+إغلاق × مرتين) نحو 42
  // بايت إضافية للحالة الأسوأ (إطار قديم/Stale + بطارية LOW + متحرك/Rolling
  // + رقم عمر كبير)، فرفعت الطول الأسوأ من ~378 إلى ~420 بايت — أي فوق حجم
  // 400 المتاح، مما يجعل snprintf يقتطع نهاية السلسلة (غالباً وسم </small>
  // أو ما قبله) ويكسر بنية HTML لبقية البطاقة. رفعناه إلى 480 (هامش أمان
  // كافٍ لأي تركيبة نصية مستقبلية مشابهة).
  char buf[480];
  for (int i = 0; i < 5; i++) {
    uint32_t age = 0;
    bool isStale = false;
    const char* statusClass = "idle";
    if (localTires[i].hasData) {
      age = nowSec - localTires[i].lastSeenEpoch;
      // إصلاح أمان: لا تُعرض هذه البطاقة كأنها "سليمة" أبداً إن كانت آخر
      // إشارة قديمة جداً — راجع تعليق staleSignalTimeoutSec أعلى الملف.
      isStale = (age > staleSignalTimeoutSec);
      statusClass = isStale ? "warn" :
                    localTires[i].isLeaking ? "danger" :
                    localTires[i].lvl == 4 ? "warn" :
                    (i < 4 && localTires[i].lvl == 1) ? "danger" :
                    (i < 4 && localTires[i].lvl == 2) ? "info" : "ok";
    }

    // إصلاح أمان (XSS): label كان يُدرَج مباشرة بلا هروب HTML هنا. خلافاً
    // لـshortId، الليبل نص حر يسمح بالعربية عمداً (لا نُقيّده بأبجدية رقمية)،
    // فالهروب عند العرض هو خط الدفاع الصحيح له، وأيضاً درعاً إضافياً لأي
    // shortId قديم مخزَّن في FRAM من قبل تنقية الإدخال المضافة في do=tires_config.
    String safeLabel = htmlEscape(myTires[i].label);
    String safeShortId = htmlEscape(myTires[i].shortId);
    snprintf(buf, sizeof(buf),
             "<div class='tcard %s'><div class='tcard-head'>"
             "<span class='tcard-label'>%s</span>"
             "<span class='tcard-sub'>[%s] &middot; #%u</span></div><div class='tcard-body'>",
             statusClass, safeLabel.c_str(), safeShortId.c_str(), (unsigned int)localTires[i].packetCount);
    webSend(buf);

    if (!localTires[i].hasData) {
      webSend("<small>⏳ في انتظار البيانات...</small>");
    } else {
      if (isStale) {
        snprintf(buf, sizeof(buf),
                 "<div class='banner warn'>⚠️ لا توجد إشارة منذ <bdi dir='ltr'>%lu</bdi> ثانية "
                 "— تحقق من الحساس (بطارية/هوائي)</div>",
                 (unsigned long)age);
        webSend(buf);
      }
      if (localTires[i].isLeaking) {
        webSend("<div class='banner danger'>⚠️ تنبيه: تسريب نشط!</div>");
      }
      if (localTires[i].lvl == 4) {
        webSend("<div class='banner warn'>⚠️ أقل من ضغطه الطبيعي المعتاد — افحص الإطار</div>");
      }

      const char* bBadge = localTires[i].batteryOk ?
        "<span style='color:#3fb950;font-weight:bold'>🔋 OK</span>" :
        "<span style='color:#f85149;font-weight:bold'>🔋 LOW ⚠️</span>";

      // إصلاح كفاءة (محتفظ به من التعديل السابق): مؤشرات const char* ثابتة
      // بدل كائنات String متكررة التخصيص على صفحة تتحدّث تلقائياً كل 10 ثوانٍ.
      const char* motionStr = localTires[i].isMoving ?
        "<span style='color:#79c0ff;font-weight:600'>🚗 متحرك (Rolling)</span>" :
        "<span style='color:#8b949e'>🅿️ متوقف (Parked)</span>";

      // عزل الأرقام/الوحدات الإنجليزية داخل <bdi dir='ltr'> يمنع خوارزمية
      // Unicode Bidi من إعادة ترتيبها بصرياً وسط الفقرة العربية المحيطة
      // (نفس إصلاح الاتجاه السابق، مطبَّق هنا على تصميم البطاقة الجديد).
      float coldB = coldPressureBar(localTires[i].bar, localTires[i].temp);
      char coldStr[16];
      if (isnan(coldB)) snprintf(coldStr, sizeof(coldStr), "--");
      else snprintf(coldStr, sizeof(coldStr), "%.*f", pDecimals(), pToUnit(coldB));
      snprintf(buf, sizeof(buf),
               "<div class='tcard-pressure'><bdi dir='ltr'>%.*f</bdi><small>%s</small></div>"
               "<div class='tcard-sub2'><bdi dir='ltr'>%.0f&deg;%s</bdi> &middot; بارد @20&deg;: <bdi dir='ltr'>%s</bdi></div>",
               pDecimals(), pToUnit(localTires[i].bar), pUnitAr(), tToUnit(localTires[i].temp), tUnit(), coldStr);
      webSend(buf);

      snprintf(buf, sizeof(buf),
               "<div class='tcard-stats'>"
               "<div class='tcard-stat'>%s</div>"
               "<div class='tcard-stat'>📶 <bdi dir='ltr'>%d dBm</bdi></div>"
               "<div class='tcard-stat' style='grid-column:1/3'>%s</div></div>",
               bBadge, localTires[i].rssi, motionStr);
      webSend(buf);

      snprintf(buf, sizeof(buf),
               "<div class='tcard-meta'><small>الحالة: %d &middot; %sمنذ <bdi dir='ltr'>%lu</bdi> ثانية</small></div>",
               localTires[i].status,
               isStale ? "(آخر قراءة معروفة) " : "", (unsigned long)age);
      webSend(buf);
      webSend("<div style='margin-top:6px'><a href='/chart?tire=" + String(i) + "' style='color:#58a6ff;font-size:12px'>📈 منحنى الضغط والحرارة</a></div>");
    }
    webSend("</div></div>");
  }
  webSend("</div>");
  streamPageFooter();
}

void handleScanner() {
  streamPageHeader("TPMS - الماسح", 10);
  webSend("<div class='card'><table><tr><th>المعرف</th><th>الموديل</th><th>ضغط</th><th>حرارة</th><th>البطارية</th><th>الحركة</th><th>RSSI</th><th>حزم</th><th>اقتران</th></tr>");
  
  ScannedSensor localList[MAX_SCANNED];
  int count = 0;
  portENTER_CRITICAL(&tireDataMux);
  count = scannedCount;
  for (int i = 0; i < count; i++) localList[i] = scannedList[i];
  portEXIT_CRITICAL(&tireDataMux);

  // كان row[300] غير كافٍ للحالة الأسوأ: id[24] (حتى 23 حرفاً) + model[48]
  // (حتى 47 حرفاً) + وسوم HTML الثابتة + "دوران" العربية قد يصل إجمالياً
  // إلى ~381 بايت، فيقتطع snprintf الصف في منتصف وسم HTML ويكسر شكل
  // الجدول/التنسيق لبقية الصفحة تحت الصف المقتطَع. رفعناه إلى 420 بايت
  // (هامش أمان) بدلاً من تتبّع الحالات الحدّية بدقة عند كل تعديل مستقبلي.
  char row[420];
  for (int i = 0; i < count; i++) {
    const char* bBadge = localList[i].batteryOk ? "<span style='color:#3fb950'>OK</span>" : "<span style='color:#f85149'>LOW</span>";
    const char* mStr = localList[i].isMoving ? "دوران" : "توقف";

    snprintf(row, sizeof(row),
             "<tr><td><b>%s</b></td><td>%s</td><td><bdi dir='ltr'>%.*f %s</bdi></td><td><bdi dir='ltr'>%.0f&deg;%s</bdi></td><td>%s</td><td>%s</td><td>%ddBm</td><td>#%u</td>"
             "<td><a class='btn blue' style='padding:3px 8px;font-size:11px' href='/settings?view=tires&assignId=%s'>تعيين</a></td></tr>",
             localList[i].id, localList[i].model, pDecimals(), pToUnit(localList[i].bar), pUnit(), tToUnit(localList[i].temp), tUnit(), bBadge, mStr,
             localList[i].rssi, (unsigned int)localList[i].packetCount, localList[i].id);
    webSend(row);
  }
  if (count == 0) webSend("<tr><td colspan='9'>لا توجد حساسات مكتشفة بعد...</td></tr>");
  webSend("</table></div>");
  streamPageFooter();
}

// كل التقاطات الجلسة (الحالي أولاً) في مصفوفة واحدة
static int techSessionSnapshot(TechCapture* out, int maxN) {
  int n = 0;
  portENTER_CRITICAL(&tireDataMux);
  if (techCap.valid && n < maxN) out[n++] = techCap;
  for (int i = 0; i < techHistCount && n < maxN; i++) out[n++] = techHist[i];
  portEXIT_CRITICAL(&tireDataMux);
  return n;
}

void handleTechCsv() {
  static TechCapture all[TECH_HIST + 1];
  int n = techSessionSnapshot(all, TECH_HIST + 1);
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.sendHeader("Content-Disposition", "attachment; filename=tire_inspection.csv");
  server.send(200, "text/csv", "");
  webSend("time,sensor_id,model,pressure_bar,cold_pressure_bar,temp_c,battery,best_rssi_dbm,packets,verdict\n");
  for (int i = 0; i < n; i++) {
    const TechCapture& c = all[i];
    DateTime dt(c.epoch);
    char row[220];
    float cold = coldPressureBar(c.bar, c.temp);
    snprintf(row, sizeof(row), "%04d-%02d-%02d %02d:%02d:%02d,%s,%s,%s,%s,%s,%s,%d,%lu,%s\n",
             dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second(),
             csvSafeField(c.id).c_str(), csvSafeField(c.model).c_str(),
             isnan(c.bar) ? "" : String(c.bar, 2).c_str(), isnan(cold) ? "" : String(cold, 2).c_str(),
             isnan(c.temp) ? "" : String(c.temp, 0).c_str(),
             c.battery == 1 ? "ok" : c.battery == 0 ? "low" : "n/a", c.bestRssi, (unsigned long)c.count,
             techVerdict(c, nullptr, nullptr));
    webSend(row);
  }
  webSend("");
}

void handleTechReport() {
  static TechCapture all[TECH_HIST + 1];
  int n = techSessionSnapshot(all, TECH_HIST + 1);
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html; charset=utf-8", "");
  DateTime now(epochNow());
  char dateBuf[32];
  snprintf(dateBuf, sizeof(dateBuf), "%04d-%02d-%02d %02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute());
  webSend("<!DOCTYPE html><html lang='ar' dir='rtl'><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
                     "<title>تقرير فحص حساسات الإطارات</title><style>body{font-family:system-ui,sans-serif;color:#111;background:#fff;margin:24px}"
                     "h1{font-size:20px;margin:0 0 4px}table{width:100%;border-collapse:collapse;margin-top:14px;font-size:13px}"
                     "th,td{border:1px solid #999;padding:6px;text-align:center}th{background:#eee}.ok{color:#137333}.bad{color:#b3261e;font-weight:700}"
                     ".warn{color:#a15c00;font-weight:700}@media print{.np{display:none}}</style></head><body>"
                     "<h1>تقرير فحص حساسات ضغط الإطارات (TPMS)</h1><div>التاريخ: <bdi dir='ltr'>" + String(dateBuf) +
                     "</bdi> | عدد الحساسات المفحوصة: " + String(n) + "</div>"
                     "<div class='np' style='margin-top:10px'><button onclick='print()'>🖨 طباعة / حفظ PDF</button> <a href='/tech'>رجوع</a></div>"
                     "<table><tr><th>#</th><th>الوقت</th><th>معرف الحساس</th><th>الموديل</th><th>الضغط</th><th>الضغط البارد @20&deg;</th><th>الحرارة</th><th>البطارية</th><th>الإشارة</th><th>الحكم</th></tr>");
  for (int i = 0; i < n; i++) {
    const TechCapture& c = all[n - 1 - i];  // الترتيب الزمني: الأقدم أولاً
    const char* ar;
    uint16_t col;
    techVerdict(c, &col, &ar);
    DateTime dt(c.epoch);
    char tbuf[12];
    snprintf(tbuf, sizeof(tbuf), "%02d:%02d", dt.hour(), dt.minute());
    float cold = coldPressureBar(c.bar, c.temp);
    String row = "<tr><td>" + String(i + 1) + "</td><td dir='ltr'>" + String(tbuf) + "</td><td dir='ltr'><b>" + htmlEscape(c.id) + "</b></td><td>" + htmlEscape(c.model) +
                 "</td><td dir='ltr'>" + (isnan(c.bar) ? String("--") : String(pToUnit(c.bar), pDecimals()) + " " + pUnit()) +
                 "</td><td dir='ltr'>" + (isnan(cold) ? String("--") : String(pToUnit(cold), pDecimals()) + " " + pUnit()) +
                 "</td><td dir='ltr'>" + (isnan(c.temp) ? String("--") : String(tToUnit(c.temp), 0) + "&deg;" + tUnit()) +
                 "</td><td>" + (c.battery == 1 ? "سليمة" : c.battery == 0 ? "<span class='bad'>ضعيفة</span>" : "—") +
                 "</td><td dir='ltr'>" + String(c.bestRssi) + " dBm</td><td class='" + (col == ST77XX_GREEN ? "ok" : col == ST77XX_RED ? "bad" : "warn") + "'>" + ar + "</td></tr>";
    webSend(row);
  }
  if (n == 0) webSend("<tr><td colspan='10'>لم يُفحص أي حساس في هذه الجلسة</td></tr>");
  webSend("</table><p style='font-size:11px;color:#555;margin-top:16px'>الضغط البارد: ما يعادل الضغط المقاس عند 20&deg;C. "
                     "الإشارة: أقوى إشارة التُقطت من الحساس.</p></body></html>");
  webSend("");
}

void handleTech() {
  streamPageHeader("TPMS - فني الإطارات", 3);
  TechCapture c;
  static TechCapture hist[TECH_HIST];
  int histCount;
  portENTER_CRITICAL(&tireDataMux);
  c = techCap;
  histCount = techHistCount;
  for (int i = 0; i < histCount; i++) hist[i] = techHist[i];
  portEXIT_CRITICAL(&tireDataMux);
  uint32_t nowMs = millis();

  webSend("<div class='card'><div class='row'><div><b>وضع فني الإطارات</b> "
                     + String(screenMode == 2 ? "<span style='color:#3fb950'>مفعّل ✅</span>" : "<span style='color:#f85149'>غير مفعّل</span>") +
                     "<br><small>يلتقط الحساس الأقرب فقط (إشارة ≥ <bdi dir='ltr'>" + String(techThresholdDbm) + " dBm</bdi>) ويتجاهل السيارات البعيدة."
                     " الإشارة الحية الآن: <bdi dir='ltr'>" + String((int)rtl_433_ESP::currentRssi) + " dBm</bdi></small></div></div>"
                     "<div style='display:flex;gap:6px;flex-wrap:wrap'>"
                     "<a class='btn on' style='flex:1' href='" + actionUrl("do=screen&mode=tech") + "'>تفعيل</a>"
                     "<a class='btn off' style='flex:1' href='" + actionUrl("do=tech_thr&dir=down") + "'>-5 أبعد</a>"
                     "<a class='btn off' style='flex:1' href='" + actionUrl("do=tech_thr&dir=up") + "'>+5 أقرب</a>"
                     "<a class='btn r' style='flex:1' href='" + actionUrl("do=tech_reset") + "'>التقاط جديد</a></div></div>");

  if (!c.valid) {
    webSend("<div class='card' style='text-align:center;padding:24px'><div style='font-size:22px;font-weight:700'>قرّب الجهاز من الإطار</div>"
                       "<small>أيقظ الحساس: نفّس الإطار قليلاً، أو دوّر العجلة، أو استخدم أداة تنشيط TPMS (LF 125kHz).</small></div>");
  } else {
    const char* ar;
    uint16_t col;
    techVerdict(c, &col, &ar);
    const char* css = (col == ST77XX_GREEN) ? "#3fb950" : (col == ST77XX_RED ? "#f85149" : "#d29922");
    char buf[700];
    char age[12];
    formatAge((nowMs - c.lastSeenMs) / 1000, age, sizeof(age));
    String safeId = htmlEscape(c.id);
    String safeModel = htmlEscape(c.model);
    snprintf(buf, sizeof(buf),
             "<div class='card'><div class='row'><div><small>معرف الحساس</small>"
             "<div style='font-size:30px;font-weight:800;font-family:monospace' dir='ltr'>%s</div><small>%s%s%s</small></div>"
             "<div style='text-align:center'><div style='font-size:42px;font-weight:800;color:%s'><bdi dir='ltr'>%s</bdi> <small>%s</small></div>"
             "<small>بارد @20&deg;: <bdi dir='ltr'>%s</bdi> &middot; <bdi dir='ltr'>%s&deg;%s</bdi></small></div></div>"
             "<div style='margin:12px 0;padding:12px;border-radius:8px;border:2px solid %s;color:%s;font-size:20px;font-weight:800;text-align:center'>%s</div>",
             safeId.c_str(), safeModel.c_str(), c.tire >= 0 ? " — <b style='color:#3fb950'>إطار سيارتي " : "",
             c.tire >= 0 ? htmlEscape(myTires[c.tire].label).c_str() : "", css,
             isnan(c.bar) ? "--" : String(pToUnit(c.bar), pDecimals()).c_str(), pUnitAr(),
             isnan(coldPressureBar(c.bar, c.temp)) ? "--" : String(pToUnit(coldPressureBar(c.bar, c.temp)), pDecimals()).c_str(),
             isnan(c.temp) ? "--" : String(tToUnit(c.temp), 0).c_str(), tUnit(), css, css, ar);
    webSend(buf);
    snprintf(buf, sizeof(buf),
             "<small>الإشارة: <bdi dir='ltr'>%d dBm</bdi> (الأقوى <bdi dir='ltr'>%d</bdi>) | البطارية: %s | الحزم: %lu | منذ <bdi dir='ltr'>%s</bdi> | التحقق: %s</small>"
             "<br><br><a class='btn blue' href='/settings?view=tires&assignId=%s'>تعيين لإطار من سيارتي</a></div>",
             c.rssi, c.bestRssi, c.battery == 1 ? "سليمة" : c.battery == 0 ? "ضعيفة" : "غير متاحة",
             (unsigned long)c.count, age, c.mic[0] ? c.mic : "-", safeId.c_str());
    webSend(buf);
  }

  webSend("<div class='card'><div class='row'><b>سجل الفحص</b><div style='display:flex;gap:6px;flex-wrap:wrap'>"
                     "<a class='btn blue' style='padding:5px 10px;font-size:12px' href='/tech/report'>📄 تقرير للطباعة</a>"
                     "<a class='btn blue' style='padding:5px 10px;font-size:12px' href='/tech/csv'>⬇ CSV</a>"
                     "<a class='btn off' style='padding:5px 10px;font-size:12px' href='" + actionUrl("do=tech_clear") + "'>🧹 جلسة جديدة</a></div></div><table><tr><th>المعرف</th><th>الموديل</th><th>ضغط</th><th>حرارة</th><th>البطارية</th><th>أقوى إشارة</th><th>الحكم</th><th>تعيين</th></tr>");
  for (int i = 0; i < histCount; i++) {
    const char* ar;
    uint16_t col;
    techVerdict(hist[i], &col, &ar);
    char row[420];
    String safeId = htmlEscape(hist[i].id);
    snprintf(row, sizeof(row),
             "<tr><td><b>%s</b></td><td>%s</td><td><bdi dir='ltr'>%s</bdi></td><td><bdi dir='ltr'>%s</bdi></td><td>%s</td><td><bdi dir='ltr'>%d</bdi></td><td>%s</td>"
             "<td><a class='btn blue' style='padding:3px 8px;font-size:11px' href='/settings?view=tires&assignId=%s'>تعيين</a></td></tr>",
             safeId.c_str(), htmlEscape(hist[i].model).c_str(),
             isnan(hist[i].bar) ? "--" : String(pToUnit(hist[i].bar), pDecimals()).c_str(),
             isnan(hist[i].temp) ? "--" : String(tToUnit(hist[i].temp), 0).c_str(),
             hist[i].battery == 1 ? "سليمة" : hist[i].battery == 0 ? "ضعيفة" : "—", hist[i].bestRssi, ar, safeId.c_str());
    webSend(row);
  }
  if (histCount == 0) webSend("<tr><td colspan='8'>لا يوجد</td></tr>");
  webSend("</table></div>");
  streamPageFooter();
}

void handleChart() {
  int tire = server.hasArg("tire") ? constrain((int)server.arg("tire").toInt(), 0, 4) : 0;
  int hours = server.hasArg("hours") ? server.arg("hours").toInt() : 24;
  if (hours != 6 && hours != 24 && hours != 72 && hours != 168) hours = 24;
  streamPageHeader("TPMS - المنحنيات", 0);

  String tabs = "<div class='card'><div class='tabs-container'>";
  for (int i = 0; i < 5; i++)
    tabs += "<a class='" + String(i == tire ? "sel" : "") + "' href='/chart?tire=" + String(i) + "&hours=" + String(hours) + "'>" + htmlEscape(myTires[i].label) + "</a>";
  tabs += "</div><div class='tabs-container'>";
  const int hs[4] = {6, 24, 72, 168};
  const char* hl[4] = {"6 ساعات", "يوم", "3 أيام", "أسبوع"};
  for (int k = 0; k < 4; k++)
    tabs += "<a class='" + String(hs[k] == hours ? "sel" : "") + "' href='/chart?tire=" + String(tire) + "&hours=" + String(hs[k]) + "'>" + hl[k] + "</a>";
  tabs += "</div>";
  webSend(tabs);

  if (!framOk) {
    webSend("<small>ذاكرة FRAM غير متصلة.</small></div>");
    streamPageFooter();
    return;
  }

  // قراءة السجل من الأقدم للأحدث مع تقليل النقاط إلى 360 كحد أقصى
  const int MAXP = 360;
  static uint32_t ts[MAXP];
  static float pb[MAXP], tc[MAXP];
  int np = 0, stride = 1, seen = 0;
  uint32_t total = 0, widx = 0;
  if (logMutex && xSemaphoreTake(logMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    total = logHeader.count;
    widx = logHeader.writeIndex;
    xSemaphoreGive(logMutex);
  }
  uint32_t nowSec = epochNow();
  uint32_t fromSec = nowSec > (uint32_t)hours * 3600UL ? nowSec - (uint32_t)hours * 3600UL : 0;
  const int CHUNK = 20;
  LogRecord chunk[CHUNK];
  uint32_t k = 0;
  while (k < total) {
    uint32_t start = (widx + FRAM_LOG_MAX_RECORDS - total + k) % FRAM_LOG_MAX_RECORDS;
    uint32_t n = min((uint32_t)CHUNK, total - k);
    n = min(n, FRAM_LOG_MAX_RECORDS - start); // لا نعبر نهاية الحلقة في قراءة واحدة
    if (!framRead((uint16_t)(FRAM_LOG_DATA_ADDR + start * FRAM_LOG_RECORD_SIZE), (uint8_t*)chunk, n * FRAM_LOG_RECORD_SIZE)) break;
    for (uint32_t j = 0; j < n; j++) {
      const LogRecord& r = chunk[j];
      if (r.tireIdx != tire || r.timestamp < fromSec || r.timestamp > nowSec + 120) continue;
      if ((seen++ % stride) != 0) continue;
      if (np == MAXP) {
        for (int q = 0; q < MAXP / 2; q++) { ts[q] = ts[2 * q]; pb[q] = pb[2 * q]; tc[q] = tc[2 * q]; }
        np = MAXP / 2;
        stride *= 2;
      }
      ts[np] = r.timestamp;
      pb[np] = r.pressureRaw / 50.0f;
      tc[np] = r.tempC;
      np++;
    }
    k += n;
    if ((k % 400) < CHUNK) esp_task_wdt_reset();
  }

  if (np < 2) {
    webSend("<br><small>لا توجد قراءات كافية لهذا الإطار في هذه الفترة.</small></div>");
    streamPageFooter();
    return;
  }

  // إحصائيات + معدل تغيّر الضغط البارد (انحدار خطي) لكشف التسريب البطيء
  float pMin = 1e9, pMax = -1e9, pSum = 0, tMin = 1e9, tMax = -1e9;
  double sx = 0, sy = 0, sxx = 0, sxy = 0;
  int nc = 0;
  for (int i = 0; i < np; i++) {
    pMin = min(pMin, pb[i]); pMax = max(pMax, pb[i]); pSum += pb[i];
    tMin = min(tMin, tc[i]); tMax = max(tMax, tc[i]);
    float cold = coldPressureBar(pb[i], tc[i]);
    if (!isnan(cold)) {
      double x = (double)(ts[i] - ts[0]) / 86400.0;
      sx += x; sy += cold; sxx += x * x; sxy += x * cold; nc++;
    }
  }
  double spanDays = (double)(ts[np - 1] - ts[0]) / 86400.0;
  double slope = 0;
  if (nc >= 3 && spanDays > 0.01) {
    double den = nc * sxx - sx * sx;
    if (fabs(den) > 1e-12) slope = (nc * sxy - sx * sy) / den;
  }

  // الرسم: SVG مباشر بدون مكتبات خارجية
  const int W = 640, H = 240, ML = 44, MR = 40, MT = 12, MB = 26;
  float lo = pMin, hi = pMax;
  if (tire < 4) { lo = min(lo, tireThresh[tire].pMin); hi = max(hi, tireThresh[tire].pMax); }
  lo -= 0.1f; hi += 0.1f;
  float tlo = tMin - 2, thi = tMax + 2;
  uint32_t t0 = ts[0], t1 = ts[np - 1] > ts[0] ? ts[np - 1] : ts[0] + 1;
  auto X = [&](uint32_t t) { return ML + (float)(t - t0) * (W - ML - MR) / (float)(t1 - t0); };
  auto YP = [&](float b) { return MT + (hi - b) * (H - MT - MB) / (hi - lo); };
  auto YT = [&](float c) { return MT + (thi - c) * (H - MT - MB) / (thi - tlo); };

  String svg = "<svg viewBox='0 0 " + String(W) + " " + String(H) + "' style='width:100%;height:auto;background:#0d1117;border-radius:8px;margin-top:10px' dir='ltr'>";
  for (int g = 0; g <= 4; g++) {
    float b = lo + (hi - lo) * g / 4.0f;
    int y = (int)YP(b);
    svg += "<line x1='" + String(ML) + "' x2='" + String(W - MR) + "' y1='" + String(y) + "' y2='" + String(y) + "' stroke='#21262d'/>"
           "<text x='4' y='" + String(y + 4) + "' fill='#8b949e' font-size='10'>" + String(pToUnit(b), pDecimals()) + "</text>";
  }
  if (tire < 4) {
    svg += "<line x1='" + String(ML) + "' x2='" + String(W - MR) + "' y1='" + String((int)YP(tireThresh[tire].pMin)) + "' y2='" + String((int)YP(tireThresh[tire].pMin)) + "' stroke='#f85149' stroke-dasharray='4 4'/>";
    svg += "<line x1='" + String(ML) + "' x2='" + String(W - MR) + "' y1='" + String((int)YP(tireThresh[tire].pMax)) + "' y2='" + String((int)YP(tireThresh[tire].pMax)) + "' stroke='#d29922' stroke-dasharray='4 4'/>";
  }
  webSend(svg);
  svg = "<polyline fill='none' stroke='#f0883e' stroke-width='1' opacity='0.7' points='";
  for (int i = 0; i < np; i++) svg += String((int)X(ts[i])) + "," + String((int)YT(tc[i])) + " ";
  svg += "'/><polyline fill='none' stroke='#3fb950' stroke-width='1.5' stroke-dasharray='3 3' points='";
  for (int i = 0; i < np; i++) { float c = coldPressureBar(pb[i], tc[i]); if (!isnan(c)) svg += String((int)X(ts[i])) + "," + String((int)YP(c)) + " "; }
  svg += "'/><polyline fill='none' stroke='#58a6ff' stroke-width='2' points='";
  for (int i = 0; i < np; i++) svg += String((int)X(ts[i])) + "," + String((int)YP(pb[i])) + " ";
  svg += "'/>";
  char lbl[64];
  DateTime d0(t0), d1(t1);
  snprintf(lbl, sizeof(lbl), "%02d/%02d %02d:%02d", d0.day(), d0.month(), d0.hour(), d0.minute());
  svg += "<text x='" + String(ML) + "' y='" + String(H - 8) + "' fill='#8b949e' font-size='10'>" + String(lbl) + "</text>";
  snprintf(lbl, sizeof(lbl), "%02d/%02d %02d:%02d", d1.day(), d1.month(), d1.hour(), d1.minute());
  svg += "<text x='" + String(W - MR - 70) + "' y='" + String(H - 8) + "' fill='#8b949e' font-size='10'>" + String(lbl) + "</text>";
  svg += "<text x='" + String(W - MR + 4) + "' y='" + String(MT + 10) + "' fill='#f0883e' font-size='10'>" + String(tToUnit(thi), 0) + "&#176;</text>";
  svg += "<text x='" + String(W - MR + 4) + "' y='" + String(H - MB) + "' fill='#f0883e' font-size='10'>" + String(tToUnit(tlo), 0) + "&#176;</text></svg>";
  webSend(svg);

  String stats = "<small style='display:block;margin-top:8px;line-height:1.8'>"
                 "<span style='color:#58a6ff'>━ الضغط</span> &nbsp; <span style='color:#3fb950'>┅ الضغط البارد @20&deg;</span> &nbsp; "
                 "<span style='color:#f0883e'>━ الحرارة</span> &nbsp; <span style='color:#f85149'>┅ الحد الأدنى</span><br>"
                 "عدد القراءات: " + String(seen) + " | الضغط: أدنى <bdi dir='ltr'>" + String(pToUnit(pMin), pDecimals()) + "</bdi> أعلى <bdi dir='ltr'>" +
                 String(pToUnit(pMax), pDecimals()) + "</bdi> متوسط <bdi dir='ltr'>" + String(pToUnit(pSum / np), pDecimals()) + " " + pUnit() +
                 "</bdi> | الحرارة <bdi dir='ltr'>" + String(tToUnit(tMin), 0) + "–" + String(tToUnit(tMax), 0) + "&deg;" + tUnit() + "</bdi></small>";
  webSend(stats);
  if (spanDays >= 0.25 && nc >= 3) {
    float perDay = (float)slope;
    bool leak = perDay < -0.03f;
    webSend(String("<div class='banner ") + (leak ? "warn" : "") + "' style='margin-top:10px;" + (leak ? "" : "background:#1b4722;color:#fff") + "'>"
                       "معدل تغيّر الضغط البارد: <bdi dir='ltr'>" + String(pToUnit(perDay), unitPressure == 0 ? 3 : 2) + " " + pUnit() + "/يوم</bdi> — " +
                       (leak ? "⚠️ تسريب بطيء محتمل، افحص الإطار والصمام" : "✅ الضغط مستقر (لا تسريب بطيء ظاهر)") + "</div>");
  } else {
    webSend("<small>تقدير التسريب البطيء يحتاج 6 ساعات قراءات على الأقل.</small>");
  }
  webSend("</div>");
  streamPageFooter();
}

void handleLearn() {
  streamPageHeader("TPMS - تعلّم الإطارات", learnActive ? 5 : 0);
  static LearnCandidate local[LEARN_MAX_CAND]; // static: ثقيلة على مكدس مهمة الويب
  int n;
  portENTER_CRITICAL(&tireDataMux);
  if (!learnActive) learnSortLocked();
  n = learnCandCount;
  for (int i = 0; i < n; i++) local[i] = learnCand[i];
  portEXIT_CRITICAL(&tireDataMux);
  if (learnActive) {
    for (int i = 1; i < n; i++) { LearnCandidate k = local[i]; int j = i - 1; while (j >= 0 && learnScore(local[j]) < learnScore(k)) { local[j + 1] = local[j]; j--; } local[j + 1] = k; }
  }

  webSend("<div class='card'><b>🧭 تعلّم إطارات السيارة تلقائياً</b><br><small style='line-height:1.7'>"
                     "اضغط «بدء» ثم قُد السيارة 3 دقائق (أو أمسك زر BOOT خمس ثوانٍ). حساسات سيارتك ترسل مراراً بإشارة قوية، "
                     "أما السيارات المارّة فتظهر مرة أو مرتين. الحساسات لا ترسل موقعها، لذا اختر موقع كل حساس بنفسك "
                     "(مثلاً: نفّس أحد الإطارات قليلاً وراقب أي معرّف يتغير ضغطه).</small><br><br><div style='display:flex;gap:8px'>");
  if (learnActive) {
    uint32_t left = (LEARN_DURATION_MS - min(LEARN_DURATION_MS, millis() - learnStartMs)) / 1000;
    webSend("<span style='flex:1;color:#3fb950;font-weight:700'>جارٍ التعلّم... متبقٍ " + String(left / 60) + ":" + (left % 60 < 10 ? "0" : "") + String(left % 60) + "</span>"
                       "<a class='btn r' href='" + actionUrl("do=learn_stop") + "'>إنهاء الآن</a>");
  } else {
    webSend("<a class='btn' style='flex:1' href='" + actionUrl("do=learn_start") + "'>بدء التعلّم (3 دقائق)</a>");
  }
  webSend("</div></div>");

  if (n > 0) {
    webSend("<div class='card'><form action='/action' method='POST'><input type='hidden' name='do' value='learn_apply'>" + csrfHiddenField() +
                       "<table><tr><th>المعرف</th><th>الموديل</th><th>دفعات</th><th>متوسط الإشارة</th><th>الضغط</th><th>الموقع</th></tr>");
    for (int i = 0; i < n && i < 12; i++) {
      int avg = local[i].packets ? (int)(local[i].rssiSum / (int32_t)local[i].packets) : 0;
      String safeId = htmlEscape(local[i].id);
      String row = "<tr" + String(local[i].bursts >= 3 ? " style='color:#3fb950'" : "") + "><td><b>" + safeId + "</b><input type='hidden' name='id_" + String(i) + "' value='" + safeId + "'></td>"
                   "<td>" + htmlEscape(local[i].model) + "</td><td>" + String(local[i].bursts) + "</td><td><bdi dir='ltr'>" + String(avg) + " dBm</bdi></td>"
                   "<td><bdi dir='ltr'>" + String(pToUnit(local[i].bar), pDecimals()) + " " + pUnit() + "</bdi></td><td><select name='pos_" + String(i) + "'><option value=''>—</option>";
      for (int p = 0; p < 5; p++) row += "<option value='" + String(p) + "'>" + htmlEscape(myTires[p].label) + "</option>";
      row += "</select></td></tr>";
      webSend(row);
    }
    webSend("</table><br><button type='submit' class='btn' style='width:100%'>حفظ المواقع المختارة</button></form>"
                       "<small>المرشحون بالأخضر ظهروا 3 دفعات أو أكثر (الأرجح أنها إطارات سيارتك).</small>"
                       "<br><br><a class='btn off' href='" + actionUrl("do=learn_close") + "'>إغلاق النتائج دون حفظ</a></div>");
  } else if (!learnActive) {
    webSend("<div class='card'><small>لا توجد نتائج بعد.</small></div>");
  }
  streamPageFooter();
}

void handleHistory() {
  streamPageHeader("TPMS - السجل التاريخي", 0);

  uint32_t total = 0;
  // إصلاح تزامن حقيقي: كان الكود يأخذ القفل بلا التحقق من نجاح ذلك (نتيجة
  // xSemaphoreTake غير مفحوصة)، ثم يستدعي xSemaphoreGive بلا شرط. لو انتهت
  // المهلة (تعارض مع framWriterTask أثناء كتابة سجل) دون أخذ القفل فعلياً:
  // (1) القراءة كانت ستحدث بلا حماية فعلية رغم أنها تبدو محمية ظاهرياً،
  // و(2) الأخطر: xSemaphoreGive كانت ستُحرّر قفلاً لم نأخذه أصلاً بينما
  // المهمة الأخرى (framWriterTask) لا تزال تملكه فعلياً — يُفسد ملكية القفل
  // ويسمح لمهمتين بالاعتقاد أنهما تملكانه في آن واحد. الإصلاح: القراءة تتم
  // فقط داخل نطاق نجاح الأخذ الفعلي (نمط مطابق للمستخدم أصلاً في مواضع
  // أخرى من الملف)، بدل الافتراض الدائم أن الأخذ نجح.
  if (logMutex && xSemaphoreTake(logMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    total = logHeader.count;
    xSemaphoreGive(logMutex);
  }
  webSend("<div class='card row'><b>السجلات في FRAM:</b><span>"
                     + String(total) + " / " + String((unsigned long)FRAM_LOG_MAX_RECORDS) + "</span></div>");

  if (!framOk) {
    webSend("<div class='card' style='color:#da3633'>تنبيه: ذاكرة FRAM غير متصلة.</div>");
  } else {
    webSend("<div class='card'><a class='btn' style='width:100%' href='/history/csv'>⬇ تحميل السجل الكامل (CSV)</a></div>");
  }

  webSend("<div class='card'><small>آخر 50 سجلاً:</small><br><br>"
                     "<table><tr><th>الوقت</th><th>الإطار</th><th>ضغط</th><th>حرارة</th><th>البطارية</th><th>الحالة</th><th>الحركة</th><th>تسرب</th><th>RSSI</th></tr>");

  uint32_t totalLogCount = 0, logWriteIndex = 0;
  // نفس إصلاح ملكية القفل أعلاه: لا نقرأ logHeader إلا داخل نطاق نجاح
  // xSemaphoreTake فعلياً، ولا نستدعي Give إلا إن كنا أخذنا القفل بالفعل.
  if (logMutex && xSemaphoreTake(logMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    totalLogCount = logHeader.count;
    logWriteIndex = logHeader.writeIndex;
    xSemaphoreGive(logMutex);
  }

  // كان بت التسرب (motionAndLeak & 0x02) يُسجَّل في FRAM ويظهر في تصدير
  // CSV (عمود leak) لكنه لم يُقرأ ولم يُعرض إطلاقاً في هذه الصفحة (لا عمود
  // له في الرأس ولا في الصف) — فيبدو سجل الويب خالياً من أي إشارة تسرب
  // سابقة رغم أن البيانات محفوظة وسليمة فعلياً. أضفنا عمود "تسرب" هنا
  // ورفعنا حجم rowBuf ليستوعب وسم <span> الإضافي بهامش أمان مماثل لتعليق
  // row[420] أعلاه.
  int rows = (int)((totalLogCount < 50UL) ? totalLogCount : 50UL);
  char rowBuf[360];
  for (int i = 0; i < rows; i++) {
    uint32_t idx = (uint32_t)((logWriteIndex + FRAM_LOG_MAX_RECORDS - 1 - i) % FRAM_LOG_MAX_RECORDS);
    LogRecord rec;
    if (!framGetLogRecord((uint16_t)(FRAM_LOG_DATA_ADDR + idx * FRAM_LOG_RECORD_SIZE), rec)) continue;
    DateTime dt(rec.timestamp);
    char tsBuf[24];
    snprintf(tsBuf, sizeof(tsBuf), "%04d-%02d-%02d %02d:%02d:%02d",
             dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second());
    const char* label = (rec.tireIdx < 5) ? myTires[rec.tireIdx].label : "?";
    const char* bBadge = (rec.batteryOk != 0) ? "<span style='color:#3fb950'>OK</span>" : "<span style='color:#f85149'>LOW</span>";
    const char* mStr = (rec.motionAndLeak & 0x01) ? "دوران" : "توقف";
    const char* lBadge = (rec.motionAndLeak & 0x02) ? "<span style='color:#f85149'><b>تسرب</b></span>" : "<span style='color:#3fb950'>-</span>";

    // إصلاح أمان (XSS): موضع رابع فات إصلاح الرسالة السابقة — نفس الحقل
    // label (نص حر يدخله المستخدم عبر صفحة "تخصيص الإطارات") كان يُدرَج هنا
    // في صفحة السجل التاريخي HTML بلا أي هروب، خلافاً لبقية المواضع الثلاثة
    // التي أُصلحت سابقاً (بطاقة الويب الرئيسية، نموذج التخصيص، تبويبات الحدود).
    String safeLabelHist = htmlEscape(label);

    snprintf(rowBuf, sizeof(rowBuf),
             "<tr><td>%s</td><td>%s</td><td>%.2f bar</td><td>%d&deg;C</td><td>%s</td><td>%d</td><td>%s</td><td>%s</td><td>%d dBm</td></tr>",
             tsBuf, safeLabelHist.c_str(), rec.pressureRaw / 50.0f, (int)rec.tempC, bBadge, (int)rec.status, mStr, lBadge, (int)rec.rssi);
    webSend(rowBuf);
  }
  if (rows == 0) webSend("<tr><td colspan='9'>لا توجد سجلات محفوظة بعد</td></tr>");
  webSend("</table></div>");
  streamPageFooter();
}

void handleHistoryCSV() {
  if (!framOk) {
    server.send(503, "text/plain; charset=utf-8", "FRAM غير متصلة");
    return;
  }

  server.sendHeader("Content-Disposition", "attachment; filename=tpms_history.csv");
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/csv; charset=utf-8", "");
  webSend("timestamp,datetime,tire,bar,psi,temp_c,battery,status,motion,leak,rssi_dbm\r\n");

  uint32_t total = 0;
  uint32_t writeIndex = 0;
  // نفس إصلاح ملكية القفل المطبَّق في handleHistory() أعلاه.
  if (logMutex && xSemaphoreTake(logMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    total = logHeader.count;
    writeIndex = logHeader.writeIndex;
    xSemaphoreGive(logMutex);
  }
  
  uint32_t startIdx = (uint32_t)((writeIndex + FRAM_LOG_MAX_RECORDS - total) % FRAM_LOG_MAX_RECORDS);

  char rowBuf[128];
  LogRecord rec;
  for (uint32_t i = 0; i < total; i++) {
    uint32_t idx = (startIdx + i) % FRAM_LOG_MAX_RECORDS;
    uint16_t addr = (uint16_t)(FRAM_LOG_DATA_ADDR + idx * FRAM_LOG_RECORD_SIZE);
    if (!framGetLogRecord(addr, rec)) continue;

    float bar = rec.pressureRaw / 50.0f;
    float psi = bar * 14.5038f;
    const char* label = (rec.tireIdx < 5) ? myTires[rec.tireIdx].label : "?";
    const char* bStr = (rec.batteryOk != 0) ? "OK" : "LOW";
    const char* mStr = (rec.motionAndLeak & 0x01) ? "ROLLING" : "PARKED";
    const char* lStr = (rec.motionAndLeak & 0x02) ? "LEAK" : "NONE";
    DateTime dt(rec.timestamp);

    String safeLabelCsv = csvSafeField(label);
    snprintf(rowBuf, sizeof(rowBuf), "%lu,%04d-%02d-%02d %02d:%02d:%02d,%s,%.2f,%.1f,%d,%s,%u,%s,%s,%d\r\n",
             (unsigned long)rec.timestamp, dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second(),
             safeLabelCsv.c_str(), bar, psi, (int)rec.tempC, bStr, (unsigned int)rec.status, mStr, lStr, (int)rec.rssi);
    webSend(rowBuf);

    if ((i & 0x1F) == 0) yield();
  }
  webSend("");
}

// ---------------- 11. صفحة الإعدادات وتخصيص الحساسات ----------------
void handleSettings() {
  String viewStr = server.hasArg("view") ? server.arg("view") : "general";
  int tab = server.hasArg("tab") ? server.arg("tab").toInt() : 0;
  if (tab < 0 || tab > 3) tab = 0;

  streamPageHeader("TPMS - الإعدادات");

  if (server.hasArg("msg") && server.arg("msg") == "wiped") {
    webSend("<div class='card' style='background:#1b4722;border:1px solid #2ea043;color:#fff;text-align:center;font-weight:600'>"
                       "✅ تم مسح وتهيئة ذاكرة FRAM بنجاح واستعادة الإعدادات المصنعية!"
                       "</div>");
  }
  
  webSend("<div class='tabs-container'>"
                     "<a class='" + String(viewStr == "general" ? "sel" : "") + "' href='/settings?view=general'>عام</a>"
                     "<a class='" + String(viewStr == "tires" ? "sel" : "") + "' href='/settings?view=tires'>حساسات الإطارات</a>"
                     "<a class='" + String(viewStr == "profiles" ? "sel" : "") + "' href='/settings?view=profiles'>ملفات السيارات</a>"
                     "<a class='" + String(viewStr == "smart" ? "sel" : "") + "' href='/settings?view=smart'>🧠 التعلّم الذاتي</a>"
                     "<a class='" + String(viewStr == "radio" ? "sel" : "") + "' href='/settings?view=radio'>الراديو CC1101</a>"
                     "<a class='" + String(viewStr == "limits" ? "sel" : "") + "' href='/settings?view=limits&tab=" + String(tab) + "'>حدود التنبيه</a>"
                     "<a class='" + String(viewStr == "decoders" ? "sel" : "") + "' href='/settings?view=decoders'>ديكودرات السيارات</a>"
                     "<a class='" + String(viewStr == "rtc" ? "sel" : "") + "' href='/settings?view=rtc'>الوقت (RTC)</a>"
                     "</div>");

  if (viewStr == "smart") {
    // ---------- (1) اكتشاف الحساسات ----------
    SmartHeader hdr; static SmartSensor tab[SMART_MAX_SENSORS]; SmartProposal prop;
    portENTER_CRITICAL(&tireDataMux);
    hdr = smartHdr; memcpy(tab, smartTab, sizeof(tab)); prop = smartProposal;
    portEXIT_CRITICAL(&tireDataMux);
    webSend("<div class='card'><div class='row'><b>1) اكتشاف حساسات سيارتك تلقائياً</b></div>"
                       "<div style='display:flex;gap:6px;margin-bottom:8px'>"
                       "<a class='btn " + String(autoSensorsMode == 1 ? "on" : "off") + "' style='flex:1' href='" + actionUrl("do=smart_mode&m=1") + "'>معطّل</a>"
                       "<a class='btn " + String(autoSensorsMode == 2 ? "on" : "off") + "' style='flex:1' href='" + actionUrl("do=smart_mode&m=2") + "'>بتأكيد زر BOOT</a>"
                       "<a class='btn " + String(autoSensorsMode == 3 ? "on" : "off") + "' style='flex:1' href='" + actionUrl("do=smart_mode&m=3") + "'>تلقائي</a></div>"
                       "<small style='display:block;line-height:1.7'>حساسات سيارتك تظهر بإشارة قوية في كل قيادة، والسيارات المارّة لا تتكرر. "
                       "بعد 3 قيادات يعرفها الجهاز، ويكتشف أيضاً تبديل إطار أو حساس. <b>تأكيد بالزر:</b> تظهر شاشة لمدة دقيقة وتضغط BOOT للتطبيق. "
                       "<b>تلقائي:</b> يُطبَّق مباشرة مع صفيرتين. المواقع في أول إعداد تقديرية — صحّحها من «حساسات الإطارات».</small>"
                       "<br><small>القيادات (الجلسات): " + String(hdr.sessionCounter) + " | جلسات بحركة فعلية: " + String(hdr.activeSessions) + "</small>");
    if (smartLastAction.length()) webSend("<div class='banner' style='background:#1b4722;color:#fff;margin-top:8px'>" + htmlEscape(smartLastAction) + "</div>");
    if (prop.valid) {
      String pr = "<div style='border:1px solid #d29922;border-radius:8px;padding:10px;margin-top:10px'><b>اقتراح بانتظار التأكيد:</b><br>";
      for (int i = 0; i < prop.count; i++) pr += htmlEscape(myTires[prop.slot[i]].label) + ": <bdi dir='ltr'>" + htmlEscape(myTires[prop.slot[i]].shortId[0] ? myTires[prop.slot[i]].shortId : "—") +
                                                  " → <b>" + htmlEscape(prop.id[i]) + "</b></bdi><br>";
      pr += "<div style='display:flex;gap:6px;margin-top:8px'><a class='btn' style='flex:1' href='" + actionUrl("do=smart_apply") + "'>تطبيق</a>"
            "<a class='btn off' style='flex:1' href='" + actionUrl("do=smart_dismiss") + "'>تجاهل</a></div></div>";
      webSend(pr);
    }
    String tb = "<table style='margin-top:10px'><tr><th>المعرف</th><th>الموديل</th><th>قيادات</th><th>النسبة</th><th>الحالة</th></tr>";
    int shown = 0;
    for (int i = 0; i < SMART_MAX_SENSORS; i++) {
      if (!tab[i].id[0]) continue;
      uint16_t span = hdr.activeSessions - tab[i].activeAtFirst + 1;
      int slot = smartAssignedSlot(tab[i].id);
      String st = slot >= 0 ? String("مربوط: ") + htmlEscape(myTires[slot].label) : String(smartConfirmed(tab[i]) ? "مؤكَّد (غير مربوط)" : "قيد المراقبة");
      tb += "<tr><td dir='ltr'><b>" + htmlEscape(tab[i].id) + "</b></td><td>" + htmlEscape(tab[i].model) + "</td><td>" + String(tab[i].qualSessions) +
            "</td><td>" + String(span ? tab[i].qualSessions * 100 / span : 0) + "%</td><td>" + st + "</td></tr>";
      shown++;
    }
    if (!shown) tb += "<tr><td colspan='5'>لا توجد بيانات بعد — قُد السيارة والجهاز يعمل</td></tr>";
    tb += "</table><br><a class='btn off' href='" + actionUrl("do=smart_reset") + "' onclick=\"return confirm('مسح بيانات اكتشاف الحساسات؟')\">مسح بيانات الاكتشاف</a></div>";
    webSend(tb);

    // ---------- (2) الضغط الطبيعي ----------
    TireBaseline bl[5]; TireState ts[5];
    portENTER_CRITICAL(&tireDataMux);
    memcpy(bl, tireBaseline, sizeof(bl)); for (int i = 0; i < 5; i++) ts[i] = tireState[i];
    portEXIT_CRITICAL(&tireDataMux);
    String b2 = "<div class='card'><div class='row'><b>2) تعلّم الضغط الطبيعي لكل إطار</b><span>" + String(baselineEnabled ? "مفعّل ✅" : "معطّل") + "</span></div>"
                "<div style='display:flex;gap:6px;margin-bottom:8px'>"
                "<a class='btn " + String(baselineEnabled ? "on" : "off") + "' style='flex:1' href='" + actionUrl("do=base_toggle&state=on") + "'>تفعيل</a>"
                "<a class='btn " + String(!baselineEnabled ? "on" : "off") + "' style='flex:1' href='" + actionUrl("do=base_toggle&state=off") + "'>تعطيل</a></div>"
                "<small style='display:block;line-height:1.7'>يتعلّم الضغط البارد المعتاد لكل إطار (عينة كل 5 دقائق، يُفعَّل بعد يوم و20 عينة). "
                "ينبّه إذا نزل الإطار <bdi dir='ltr'>" + String(pToUnit(BASE_DEV_BAR), pDecimals() + 1) + " " + pUnit() + "</bdi> عن طبيعيه حتى لو بقي ضمن الحدود.</small>"
                "<table style='margin-top:10px'><tr><th>الإطار</th><th>الطبيعي @20&deg;</th><th>الحالي @20&deg;</th><th>عينات</th><th>الحالة</th></tr>";
    for (int i = 0; i < 5; i++) {
      float cur = ts[i].hasData ? coldPressureBar(ts[i].bar, ts[i].temp) : NAN;
      String status = bl[i].samples == 0 ? "لا بيانات" : baselineReady(bl[i]) ? (!isnan(cur) && cur < bl[i].coldBar - BASE_DEV_BAR ? "<span style='color:#d29922'>أقل من الطبيعي</span>" : "<span style='color:#3fb950'>نشط</span>") : "يتعلّم";
      b2 += "<tr><td>" + htmlEscape(myTires[i].label) + "</td><td dir='ltr'>" + (bl[i].samples ? String(pToUnit(bl[i].coldBar), pDecimals()) : String("--")) +
            "</td><td dir='ltr'>" + (isnan(cur) ? String("--") : String(pToUnit(cur), pDecimals())) + "</td><td>" + String(bl[i].samples) + "</td><td>" + status + "</td></tr>";
    }
    b2 += "</table><div style='display:flex;gap:6px;margin-top:10px;flex-wrap:wrap'>"
          "<a class='btn blue' style='flex:1' href='" + actionUrl("do=base_limits") + "'>تطبيق حدود مقترحة (الطبيعي −0.30 / +0.50 بار)</a>"
          "<a class='btn off' style='flex:1' href='" + actionUrl("do=base_reset") + "' onclick=\"return confirm('إعادة تعلّم الضغط الطبيعي لكل الإطارات؟')\">إعادة التعلّم</a></div></div>";
    webSend(b2);

    // ---------- (3) المعايرة الذاتية ----------
    long nextMin = tuneRunning ? 0 : (long)((int32_t)(tuneNextMs - millis()) / 60000L);
    webSend("<div class='card'><div class='row'><b>3) المعايرة الذاتية لتردد الراديو</b><span>" + String(autoTuneEnabled ? "مفعّلة ✅" : "معطّلة") + "</span></div>"
                       "<div style='display:flex;gap:6px;margin-bottom:8px'>"
                       "<a class='btn " + String(autoTuneEnabled ? "on" : "off") + "' style='flex:1' href='" + actionUrl("do=tune_toggle&state=on") + "'>تفعيل</a>"
                       "<a class='btn " + String(!autoTuneEnabled ? "on" : "off") + "' style='flex:1' href='" + actionUrl("do=tune_toggle&state=off") + "'>تعطيل</a>"
                       "<a class='btn blue' style='flex:1' href='" + actionUrl("do=tune_now") + "'>تشغيل الآن</a></div>"
                       "<small style='display:block;line-height:1.7'>كل 3 ساعات يقارن الإزاحة الحالية بإزاحة مجاورة (±5 kHz) لمدة 20 دقيقة بالتناوب، "
                       "ويعتمدها فقط إذا فكّت 15% حزماً أكثر. يتوقف أثناء وضع الفني والتعلّم.</small><br>"
                       "<small>الإزاحة الحالية: <bdi dir='ltr'>" + String(freqOffsetKhz > 0 ? "+" : "") + String(freqOffsetKhz) + " kHz</bdi> | " +
                       (tuneRunning ? String("قيد التشغيل (") + String(tuneSlice + 1) + "/" + String(TUNE_SLICES) + ")" : String("التالية بعد ") + String(max(0L, nextMin)) + " دقيقة") +
                       "<br>آخر نتيجة: " + htmlEscape(tuneLastResult) + "</small></div>");
  } else if (viewStr == "profiles") {
    webSend("<div class='card'><b>ملفات السيارات</b><br><small>كل ملف يحفظ معرّفات حساسات سيارة وحدود إنذارها. "
                       "التبديل يحفظ السيارة الحالية تلقائياً ثم يحمّل المختارة.</small><br><br>");
    for (int slot = 0; slot < PROFILE_COUNT; slot++) {
      VehicleProfile p;
      bool ok = profileLoad(slot, p);
      String name = ok && p.name[0] ? String(p.name) : ("سيارة " + String(slot + 1));
      String ids;
      if (ok) for (int i = 0; i < 5; i++) if (p.tires[i].shortId[0]) ids += String(p.tires[i].shortId) + " ";
      bool active = slot == activeProfile;
      webSend("<div style='border:1px solid " + String(active ? "#2ea043" : "#30363d") + ";border-radius:8px;padding:10px;margin-bottom:10px'>"
                         "<div class='row'><b>" + htmlEscape(name) + (active ? " <span style='color:#3fb950'>(النشطة)</span>" : "") + "</b>" +
                         (active ? String("") : "<a class='btn blue' href='" + actionUrl("do=profile_activate&slot=" + String(slot)) + "'>تفعيل</a>") + "</div>"
                         "<small dir='ltr'>" + (ids.length() ? htmlEscape(ids) : String(ok ? "—" : "فارغ")) + "</small>"
                         "<form action='/action' method='POST' style='display:flex;gap:6px;margin-top:8px'>"
                         "<input type='hidden' name='do' value='profile_rename'><input type='hidden' name='slot' value='" + String(slot) + "'>" + csrfHiddenField() +
                         "<input type='text' name='name' maxlength='22' value='" + htmlEscape(name) + "'><button class='btn off' type='submit'>تسمية</button></form></div>");
    }
    webSend("</div>");
  } else if (viewStr == "tires") {
    String prefillId = server.hasArg("assignId") ? server.arg("assignId") : "";
    webSend("<div class='card'><b>إدارة وتخصيص معرّفات الحساسات</b><br><small>قم بتعيين معرّف الحساس (Sensor ID) لكل إطار. يتم حفظ التعديلات تلقائياً في FRAM دون الحاجة لإعادة رفع الكود.</small><br><br>");
    
    if (prefillId.length() > 0) {
      webSend("<div style='background:#1f6feb22;border:1px solid #1f6feb;padding:8px;border-radius:6px;margin-bottom:12px'>"
                         "المعرّف المختار من الماسح: <b style='color:#58a6ff'>" + htmlEscape(prefillId) + "</b> - اختر الإطار المطلوب واضغط حفظ.</div>");
    }

    webSend("<form action='/action' method='POST'><input type='hidden' name='do' value='tires_config'>" + csrfHiddenField());
    for (int i = 0; i < 5; i++) {
      webSend("<div class='row' style='border-bottom:1px solid #21262d;padding-bottom:8px'>"
                         "<div style='width:120px'><b>" + htmlEscape(myTires[i].label) + ":</b></div>"
                         "<div style='flex:1;display:flex;gap:6px'>"
                         "<input type='text' name='id_" + String(i) + "' value='" + htmlEscape(myTires[i].shortId) + "' placeholder='مثال: 78D8483' maxlength='11'>"
                         "<input type='text' style='width:90px' name='lbl_" + String(i) + "' value='" + htmlEscape(myTires[i].label) + "' placeholder='الاسم' maxlength='7'>"
                         "</div></div>");
    }
    webSend("<br><button type='submit' class='btn' style='width:100%'>حفظ إعدادات الحساسات</button></form>"
                       "<br><a class='btn blue' style='width:100%' href='/learn'>🧭 تعلّم الإطارات تلقائياً بالقيادة</a></div>");
    {
      String oc = "<div class='card' style='border:1px solid #2ea043'><b>🔒 الحساسات الأصلية لسيارتك</b><br>"
                  "<small>مرجع ثابت محفوظ في FRAM. التعلّم الذاتي لا يستبدلها تلقائياً أبداً — يطلب تأكيدك دائماً.</small>"
                  "<table style='margin-top:8px'><tr><th>الإطار</th><th>الأصلي</th><th>الحالي</th></tr>";
      for (int i = 0; i < 5; i++) {
        bool same = slotHasOriginal(i);
        oc += "<tr><td>" + htmlEscape(myTires[i].label) + "</td><td dir='ltr'><b>" + htmlEscape(originals.ids[i]) + "</b></td><td dir='ltr' style='color:" +
              (same ? "#3fb950'>✓ " : "#d29922'>") + htmlEscape(myTires[i].shortId[0] ? myTires[i].shortId : "—") + "</td></tr>";
      }
      oc += "</table><div style='display:flex;gap:6px;margin-top:10px;flex-wrap:wrap'>"
            "<a class='btn' style='flex:1' href='" + actionUrl("do=orig_restore") + "' onclick=\"return confirm('استعادة الحساسات الأصلية لكل الإطارات؟')\">↩ استعادة الحساسات الأصلية</a>"
            "<a class='btn off' style='flex:1' href='" + actionUrl("do=orig_save") + "' onclick=\"return confirm('اعتماد الحساسات الحالية كأصلية جديدة؟')\">حفظ الحالية كأصلية</a></div></div>";
      webSend(oc);
    }

  } else if (viewStr == "radio") {
    webSend("<div class='card'>"
                       "<div class='row'><b>انحراف تردد الراديو (Frequency Deviation)</b><span style='color:#58a6ff;font-weight:700;font-size:16px'>"
                       + String(freqDevCurrent) + " kHz</span></div>"
                       "<div style='display:grid;grid-template-columns:repeat(5, 1fr);gap:4px;margin-bottom:10px'>");
    int presets[] = {25, 35, 40, 47, 50};
    for (int p : presets) {
      String activeClass = (freqDevCurrent == p) ? "on" : "off";
      webSend("<a class='btn " + activeClass + "' style='font-size:11px;padding:6px 2px;text-align:center' href='" + actionUrl("do=freqdev&set=" + String(p) + "&view=radio") + "'>" + String(p) + "k</a>");
    }
    webSend("</div>"
                       "<div style='display:flex;gap:8px'>"
                       "<a class='btn off' style='flex:1' href='" + actionUrl("do=freqdev&dir=down&view=radio") + "'>- 5 kHz تقليل</a>"
                       "<a class='btn off' style='flex:1' href='" + actionUrl("do=freqdev&dir=up&view=radio") + "'>+ 5 kHz زيادة</a>"
                       "</div>"
                       "<small style='display:block;margin-top:10px;line-height:1.5'>"
                       "ℹ️ <b>توصية:</b> حساسات تويوتا ولكزس (Pacific) تعمل بكفاءة قصوى عند <b>40 kHz</b> أو <b>35 kHz</b>."
                       "</small></div>");

    {
      webSend("<div class='card'><div class='row'><b>عرض نطاق الاستقبال</b><span style='color:#58a6ff;font-weight:700;font-size:16px'>"
                         + String((int)currentRxBwKhz()) + " kHz</span></div>"
                         "<div style='display:grid;grid-template-columns:repeat(4, 1fr);gap:4px;margin-bottom:10px'>");
      const int bwPresets[] = {2, 3, 4, 6, 7, 9};   // 81, 102, 116, 162, 203, 270
      for (int bi : bwPresets) {
        webSend("<a class='btn " + String(rxBwIndex == bi ? "on" : "off") + "' style='font-size:11px;padding:6px 2px;text-align:center' href='"
                           + actionUrl("do=rxbw&idx=" + String(bi) + "&view=radio") + "'>" + String((int)CC1101_BW_LIST[bi]) + "</a>");
      }
      webSend("</div><small style='display:block;line-height:1.5'>ℹ️ <b>102 kHz</b> أعطى أبعد مدى في الاختبار الميداني "
                         "(فك إشارات حتى -98 dBm مقابل -93 عند 270). الأضيق قد يفقد حساسات منحرفة التردد.</small></div>");

      webSend("<div class='card'><div class='row'><b>معايرة التردد</b><span style='color:#58a6ff;font-weight:700'>"
                         + String(currentFreqMhz(), 3) + " MHz (" + String(freqOffsetKhz > 0 ? "+" : "") + String(freqOffsetKhz) + " kHz)</span></div>"
                         "<div style='display:flex;gap:6px'>"
                         "<a class='btn off' style='flex:1' href='" + actionUrl("do=foff&set=" + String(freqOffsetKhz - 5) + "&view=radio") + "'>-5 kHz</a>"
                         "<a class='btn off' style='flex:1' href='" + actionUrl("do=foff&set=0&view=radio") + "'>صفر</a>"
                         "<a class='btn off' style='flex:1' href='" + actionUrl("do=foff&set=" + String(freqOffsetKhz + 5) + "&view=radio") + "'>+5 kHz</a></div>"
                         "<small style='display:block;margin-top:8px;line-height:1.5'>بلورات CC1101 الرخيصة تنحرف 20-50 kHz. غيّر بخطوات 5 وراقب عدد الحزم المفكوكة.</small></div>");

      rtl_433_ESPStatus st = rtl_433_ESP::statusSnapshot();
      webSend("<div class='card'><b>تشخيص الراديو</b><br><small style='line-height:1.8'>"
                         "أرضية الضجيج: " + String(st.averageRssi) + " dBm | العتبة: " + String(st.rssiThreshold) + " dBm | الحالية: " + String(st.currentRssi) + " dBm<br>"
                         "إشارات ملتقطة: " + String(st.capturedSignals) + " | مفكوكة: " + String(st.decodedMessages) + " | غير معروفة: " + String(st.zeroDecodedSignals) + "<br>"
                         "مفقودة: " + String(st.droppedCaptureBuffers) + "/" + String(st.droppedDecoderQueue) + "/" + String(rfQueueDropCounter)
                         + " | مرفوضة (قيم مستحيلة): " + String(rfRejectedCounter) + " | إعادات تهيئة الراديو: " + String(ccRecoveryCounter) + "</small></div>");
    }

    webSend("<div class='card'><div class='row'><b>حساسية الاستقبال (RSSI Delta)</b><span>"
                       + String(rssiDeltaCurrent) + "</span></div>"
                       "<div style='display:flex;gap:8px'>"
                       "<a class='btn off' style='flex:1' href='" + actionUrl("do=rssi&dir=down&view=radio") + "'>- أبعد مدى (أكثر حساسية)</a>"
                       "<a class='btn off' style='flex:1' href='" + actionUrl("do=rssi&dir=up&view=radio") + "'>+ ضجيج أقل</a></div>"
                       "<small style='display:block;margin-top:8px'>الرقم = كم dB فوق أرضية الضجيج يجب أن تكون الإشارة لالتقاطها. الأقل = مدى أبعد. الافتراضي 9.</small></div>");

  } else if (viewStr == "decoders") {
    webSend("<div class='card'>"
                       "<div class='row'><b>إدارة ديكودرات السيارات</b>"
                       "<div><a class='btn blue' style='padding:5px 10px;font-size:12px;margin-left:4px' href='" + actionUrl("do=decoders_all&state=on") + "'>تفعيل الكل</a>"
                       "<a class='btn off' style='padding:5px 10px;font-size:12px' href='" + actionUrl("do=decoders_all&state=off") + "'>تعطيل الكل</a></div></div>"
                       "<table><tr><th>السيارة / البروتوكول</th><th>النوع</th><th>الحالة</th><th>إجراء</th></tr>");
    int decCount = rtl_433_ESP::getTpmsDecoderCount();
    for (int i = 0; i < decCount; i++) {
      bool isToy = rtl_433_ESP::isTpmsDecoderToyota(i);
      bool isEn  = (scannerDecodersMask & (1UL << i)) != 0;
      webSend("<tr><td><b>" + String(rtl_433_ESP::getTpmsDecoderLabel(i)) + "</b><br><small>" + String(rtl_433_ESP::getTpmsDecoderName(i)) + "</small></td>"
                         "<td>" + String(isToy ? "<span style='color:#f0883e;font-weight:bold'>تويوتا</span>" : "<span style='color:#8b949e'>أخرى</span>") + "</td>"
                         "<td>" + String(isEn ? "<span style='color:#3fb950;font-weight:600'>مُفعّل</span>" : "<span style='color:#8b949e'>معطّل</span>") + "</td>"
                         "<td><a class='btn " + String(isEn ? "off" : "on") + "' style='padding:4px 12px;font-size:12px' href='" + actionUrl("do=toggle_decoder&idx=" + String(i)) + "'>" + String(isEn ? "إيقاف" : "تشغيل") + "</a></td></tr>");
    }
    webSend("</table></div>");

  } else if (viewStr == "limits") {
    webSend("<div class='card'><div class='tabs-container'>");
    for (int i = 0; i < 4; i++) {
      webSend("<a class='" + String(i == tab ? "sel" : "") + "' href='/settings?view=limits&tab=" + String(i) + "'>" + htmlEscape(myTires[i].label) + "</a>");
    }
    webSend("</div>");
    
    TireThreshold& t = tireThresh[tab];
    float vals[4] = {t.pMin, t.pMax, t.tMin, t.tMax};
    const char* fields[4] = {"pmin", "pmax", "tmin", "tmax"};
    for (int r = 0; r < 4; r++) {
      webSend("<div class='row'><b>" + String(THR_FIELD_LABEL[r]) + "</b>"
                         "<div><span style='margin-left:12px;font-weight:600'>" + String(vals[r], (r < 2) ? 2 : 0) + "</span>"
                         "<a class='btn off' style='padding:4px 12px;margin-left:4px' href='" + actionUrl("do=thr&idx=" + String(tab) + "&field=" + fields[r] + "&dir=-1&view=limits&tab=" + String(tab)) + "'>-</a>"
                         "<a class='btn off' style='padding:4px 12px' href='" + actionUrl("do=thr&idx=" + String(tab) + "&field=" + fields[r] + "&dir=1&view=limits&tab=" + String(tab)) + "'>+</a></div></div>");
    }
    webSend("</div>");

  } else if (viewStr == "rtc") {
    uint32_t nowSec = epochNow();
    DateTime dt(nowSec);
    char nowBuf[24];
    snprintf(nowBuf, sizeof(nowBuf), "%04d-%02d-%02d %02d:%02d:%02d",
             dt.year(), dt.month(), dt.day(), dt.hour(), dt.minute(), dt.second());

    webSend("<div class='card'>"
                       "<div class='row'><b>حالة الساعة (DS3231)</b><span>" + String(rtcOk ? "متصلة ✅" : "غير متصلة ❌") + "</span></div>"
                       "<div class='row'><b>الوقت الحالي بالجهاز</b><span>" + String(rtcOk ? nowBuf : "--") + "</span></div>"
                       "<button class='btn' style='width:100%;margin-top:10px' onclick='syncNow()' " + String(rtcOk ? "" : "disabled") + ">مزامنة فورية مع توقيت هاتفي</button>"
                       "<script>function syncNow(){var epoch=Math.floor(Date.now()/1000);var xhr=new XMLHttpRequest();xhr.open('POST','/action',true);xhr.setRequestHeader('Content-Type','application/x-www-form-urlencoded');xhr.onload=function(){location.href='/settings?view=rtc';};xhr.send('do=rtc_sync&epoch='+epoch+'&csrf=" + csrfToken + "');}</script>"
                       "<hr style='border-color:#30363d;margin:16px 0'>"
                       "<form action='/action' method='POST'>"
                       "<input type='hidden' name='do' value='rtc_sync'>"
                       "<input type='hidden' name='view' value='rtc'>" + csrfHiddenField() +
                       "<label>ضبط يدوي للتاريخ والساعة:</label>"
                       "<input type='datetime-local' name='datetime' required style='margin-top:6px'><br><br>"
                       "<button type='submit' class='btn blue' style='width:100%'" + String(rtcOk ? "" : " disabled") + ">تطبيق الضبط اليدوي</button></form></div>");
  } else {
    webSend("<div class='card'><div class='row'><b>تردد المعالج (CPU Clock)</b><span>"
                       + String(getCpuFrequencyMhz()) + " MHz</span></div>"
                       "<div style='display:flex;gap:6px'>"
                       "<a class='btn " + String(cpuFreqCurrent == 240 ? "on" : "off") + "' style='flex:1;font-size:12px;padding:8px 4px' href='" + actionUrl("do=cpufreq&freq=240") + "'>240 MHz (أقصى سرعة)</a>"
                       "<a class='btn " + String(cpuFreqCurrent == 160 ? "on" : "off") + "' style='flex:1;font-size:12px;padding:8px 4px' href='" + actionUrl("do=cpufreq&freq=160") + "'>160 MHz (متوازن)</a>"
                       "<a class='btn " + String(cpuFreqCurrent == 80  ? "on" : "off") + "' style='flex:1;font-size:12px;padding:8px 4px' href='" + actionUrl("do=cpufreq&freq=80") + "'>80 MHz (اقتصادي)</a>"
                       "</div></div>");

    {
      int curMin = (int)(staleSignalTimeoutSec / 60UL);
      webSend("<div class='card'><div class='row'><b>مهلة كشف انقطاع إشارة الحساس</b><span>"
                         + String(curMin) + " دقيقة</span></div>"
                         "<div style='color:#8b949e;font-size:12px;margin-bottom:8px'>"
                         "إن لم تصل إشارة من أي إطار لهذه المدة، تُعرض بطاقته كـ«لا توجد إشارة» بدل عرض آخر قراءة قديمة كأنها سليمة."
                         "</div>"
                         "<div style='display:flex;gap:6px;flex-wrap:wrap'>");
      const int presets[] = {5, 10, 20, 30, 60};
      for (int p : presets) {
        webSend("<a class='btn " + String(curMin == p ? "on" : "off") + "' style='flex:1;min-width:60px;font-size:12px;padding:8px 4px' href='" + actionUrl("do=stale_timeout&minutes=" + String(p)) + "'>" + String(p) + " د</a>");
      }
      webSend("</div></div>");
    }

    webSend("<div class='card'><div class='row'><b>وحدات العرض</b></div>"
                       "<div style='display:flex;gap:6px;margin-bottom:8px'>"
                       "<a class='btn " + String(unitPressure == 0 ? "on" : "off") + "' style='flex:1' href='" + actionUrl("do=units&p=0") + "'>بار</a>"
                       "<a class='btn " + String(unitPressure == 1 ? "on" : "off") + "' style='flex:1' href='" + actionUrl("do=units&p=1") + "'>PSI</a>"
                       "<a class='btn " + String(unitPressure == 2 ? "on" : "off") + "' style='flex:1' href='" + actionUrl("do=units&p=2") + "'>kPa</a></div>"
                       "<div style='display:flex;gap:6px'>"
                       "<a class='btn " + String(unitTemp == 0 ? "on" : "off") + "' style='flex:1' href='" + actionUrl("do=units&t=0") + "'>مئوية &deg;C</a>"
                       "<a class='btn " + String(unitTemp == 1 ? "on" : "off") + "' style='flex:1' href='" + actionUrl("do=units&t=1") + "'>فهرنهايت &deg;F</a></div>"
                       "<small style='display:block;margin-top:8px'>حدود الإنذار تبقى بالبار داخلياً. الضغط البارد = ما يعادل الضغط الحالي عند 20&deg;C.</small></div>");

    webSend("<div class='card'><div class='row'><b>الإنذار الصوتي (طنان GPIO13) ولمبة GPIO2</b><span>"
                       + String(buzzerEnabled ? "مفعّل ✅" : "معطّل") + "</span></div>"
                       "<div style='display:flex;gap:6px'>"
                       "<a class='btn " + String(buzzerEnabled ? "on" : "off") + "' style='flex:1' href='" + actionUrl("do=buzzer&state=on") + "'>تفعيل</a>"
                       "<a class='btn " + String(!buzzerEnabled ? "on" : "off") + "' style='flex:1' href='" + actionUrl("do=buzzer&state=off") + "'>تعطيل</a>"
                       "<a class='btn blue' style='flex:1' href='" + actionUrl("do=beeptest") + "'>اختبار الصوت</a></div>"
                       "<small style='display:block;margin-top:8px'>عند خطر في أي إطار: 3 صفارات تتكرر كل دقيقة واللمبة تومض. ضغطة قصيرة على BOOT تُسكت الإنذار.</small></div>");

    webSend("<div class='card'><div class='row'><b>قوة إشارة الواي فاي</b><span>"
                       + String(WIFI_POWER_DBM[wifiPowerIndex], 1) + " dBm</span></div>"
                       "<div style='display:flex;gap:8px'>"
                       "<a class='btn off' style='flex:1' href='" + actionUrl("do=wifipower&dir=down") + "'>- تقليل القوة</a>"
                       "<a class='btn off' style='flex:1' href='" + actionUrl("do=wifipower&dir=up") + "'>+ زيادة القوة</a></div></div>");

    webSend("<div class='card'><b>إعدادات نقطة الوصول (Wi-Fi AP)</b><br><br>"
                       "<form action='/action' method='POST'><input type='hidden' name='do' value='ap_config'>" + csrfHiddenField() +
                       "<div style='margin-bottom:10px'><label>اسم الشبكة:</label><input type='text' name='ssid' value='" + htmlEscape(apSSID) + "' required maxlength='32' style='margin-top:4px'></div>"
                       "<div style='margin-bottom:14px'><label>كلمة المرور:</label><input type='password' name='pass' value='' minlength='8' maxlength='64' placeholder='اتركه فارغاً للاحتفاظ بكلمة المرور الحالية' style='margin-top:4px'></div>"
                       "<button type='submit' class='btn' style='width:100%'>حفظ وتطبيق</button></form></div>");

    webSend("<div class='card' style='border:1px solid #da3633'>"
                       "<b style='color:#f85149'>تهيئة ومسح ذاكرة FRAM بالكامل</b><br>"
                       "<small style='display:block;margin:6px 0 14px 0;line-height:1.5'>"
                       "سيؤدي هذا الإجراء لمسح كافة السجلات التاريخية وبيانات الإطارات وإعادة الضبط للوضع المصنعي."
                       "</small>"
                       "<button class='btn r' style='width:100%' onclick='confirmWipe()'>⚠️ مسح وتهيئة الذاكرة بالكامل</button></div>"
                       // تحويل من GET (location.href، كان يمكن تنفيذه عرضياً بمجرد
                       // فتح الرابط عبر أي وسيلة: prefetch للمتصفح، إعادة تشغيل
                       // تبويب/سجل تصفح، أو رابط <img>/<a> خبيث من صفحة أخرى) إلى
                       // POST حقيقي عبر fetch — الآن هذا الإجراء المدمّر لا يمكن
                       // تنفيذه بمجرد فتح/زيارة الرابط، بل يتطلب طلب POST صريح،
                       // وhandleAction() ترفض أي طلب غير POST لـ do=wipe_fram (انظر أدناه).
                       "<script>function confirmWipe(){"
                       "if(!confirm('تحذير شديد:\\nهل أنت متأكد تماماً من مسح ذاكرة FRAM بالكامل؟\\nسيتم حذف جميع السجلات والإعدادات فوراً!'))return;"
                       "var xhr=new XMLHttpRequest();xhr.open('POST','/action',true);"
                       "xhr.setRequestHeader('Content-Type','application/x-www-form-urlencoded');"
                       "xhr.onload=function(){location.href='/settings?view=general'+(xhr.status<400?'&msg=wiped':'');};"
                       "xhr.onerror=function(){alert('فشل الاتصال بالجهاز');};"
                       "xhr.send('do=wipe_fram&csrf=" + csrfToken + "');"
                       "}</script>");
  }
  streamPageFooter();
}

void handleAction() {
  if (!server.hasArg("csrf") || server.arg("csrf") != csrfToken) {
    server.send(403, "text/plain; charset=utf-8", "رفض الطلب: توكن CSRF مفقود أو غير صالح");
    return;
  }

  String doWhat = server.arg("do");

  if (doWhat == "freqdev") {
    if (server.hasArg("set")) {
      int val = server.arg("set").toInt();
      applyAndSaveFreqDev(val);
    } else if (server.hasArg("dir")) {
      int dir = (server.arg("dir") == "up") ? FREQ_DEV_STEP : -FREQ_DEV_STEP;
      applyAndSaveFreqDev(freqDevCurrent + dir);
    }
  } else if (doWhat == "orig_restore") {
    originalsRestore();
  } else if (doWhat == "orig_save") {
    originalsSaveCurrent();
  } else if (doWhat == "smart_mode") {
    autoSensorsMode = constrain((int)server.arg("m").toInt(), 1, 3);
    saveConfigToFram();
  } else if (doWhat == "smart_apply") {
    smartApplyProposal(false);
  } else if (doWhat == "smart_dismiss") {
    smartDismissProposal();
  } else if (doWhat == "smart_reset") {
    smartReset();
  } else if (doWhat == "base_toggle") {
    baselineEnabled = server.arg("state") == "on";
    saveConfigToFram();
  } else if (doWhat == "base_reset") {
    baselineReset(-1);
  } else if (doWhat == "base_limits") {
    portENTER_CRITICAL(&tireDataMux);
    for (int i = 0; i < 4; i++) {
      if (!baselineReady(tireBaseline[i])) continue;
      float b = tireBaseline[i].coldBar;
      tireThresh[i].pMin = constrain(roundf((b - 0.30f) * 20.0f) / 20.0f, P_ABS_MIN, P_ABS_MAX - P_MIN_GAP);
      tireThresh[i].pMax = constrain(roundf((b + 0.50f) * 20.0f) / 20.0f, tireThresh[i].pMin + P_MIN_GAP, P_ABS_MAX);
    }
    portEXIT_CRITICAL(&tireDataMux);
    saveConfigToFram();
  } else if (doWhat == "tune_toggle") {
    autoTuneEnabled = server.arg("state") == "on";
    saveConfigToFram();
  } else if (doWhat == "tune_now") {
    if (autoTuneEnabled && !tuneRunning) tuneNextMs = millis();
  } else if (doWhat == "profile_activate") {
    profileActivate(server.arg("slot").toInt());
    lastDrawnScreenMode = -1;
  } else if (doWhat == "profile_rename") {
    String nm = server.arg("name");
    nm.trim();
    if (nm.length() > 0) profileRename(server.arg("slot").toInt(), nm);
  } else if (doWhat == "units") {
    if (server.hasArg("p")) unitPressure = constrain((int)server.arg("p").toInt(), 0, 2);
    if (server.hasArg("t")) unitTemp = constrain((int)server.arg("t").toInt(), 0, 1);
    saveConfigToFram();
    lastDrawnScreenMode = -1; // إعادة رسم الشاشة بالوحدة الجديدة
  } else if (doWhat == "buzzer") {
    buzzerEnabled = server.arg("state") == "on";
    if (!buzzerEnabled) { beepRemaining = 0; digitalWrite(BUZZER_PIN, LOW); }
    saveConfigToFram();
  } else if (doWhat == "beeptest") {
    bool was = buzzerEnabled;
    buzzerEnabled = true;
    beep(2, 150, 120);
    buzzerEnabled = was;
  } else if (doWhat == "learn_start") {
    learnStart();
    lastDrawnScreenMode = -1;
  } else if (doWhat == "learn_stop") {
    if (learnActive) learnFinish();
  } else if (doWhat == "learn_close") {
    learnDone = false;
    lastDrawnScreenMode = -1;
  } else if (doWhat == "learn_apply") {
    // ربط المرشحين المختارين بمواقع الإطارات
    char ids[5][sizeof(myTires[0].shortId)];
    bool set[5] = {false, false, false, false, false};
    for (int k = 0; k < LEARN_MAX_CAND; k++) {
      String pos = server.arg("pos_" + String(k));
      String id  = server.arg("id_" + String(k));
      if (pos.length() == 0 || id.length() == 0) continue;
      int p = pos.toInt();
      if (p < 0 || p > 4) continue;
      String clean;
      for (unsigned int q = 0; q < id.length(); q++) if (isalnum((unsigned char)id[q])) clean += (char)toupper((unsigned char)id[q]);
      strncpy(ids[p], clean.c_str(), sizeof(ids[p]) - 1);
      ids[p][sizeof(ids[p]) - 1] = '\0';
      set[p] = true;
    }
    portENTER_CRITICAL(&tireDataMux);
    for (int p = 0; p < 5; p++) {
      if (!set[p]) continue;
      memcpy(myTires[p].shortId, ids[p], sizeof(myTires[p].shortId));
      tireState[p].hasData = false;
      tireState[p].packetCount = 0;
    }
    learnDone = false;
    portEXIT_CRITICAL(&tireDataMux);
    saveConfigToFram();
    tireDataDirty = true;
    lastDrawnScreenMode = -1;
  } else if (doWhat == "rxbw") {
    applyAndSaveRxBw(server.arg("idx").toInt());
  } else if (doWhat == "foff") {
    applyAndSaveFreqOffset(server.arg("set").toInt());
  } else if (doWhat == "cpufreq") {
    int freq = server.arg("freq").toInt();
    applyAndSaveCpuFreq(freq);
  } else if (doWhat == "stale_timeout") {
    int minutes = server.arg("minutes").toInt();
    applyAndSaveStaleTimeoutMinutes(minutes);
  } else if (doWhat == "tires_config") {
    // إصلاح تزامن (Race Condition): كانت الكتابة هنا تُعدِّل myTires[] مباشرة
    // بلا أي قفل، بينما storageTask() تقرأ نفس المصفوفة مع كل حزمة راديو
    // واردة (على نفس النواة لكن من مهمة FreeRTOS مختلفة) بلا قفل أيضاً على
    // جانب القراءة — قراءة ممزوجة جزئياً (torn read) نظرياً ممكنة لو بادل
    // المُجدوِل المهمتين أثناء strncpy. نجهّز القيم في مخازن محلية أولاً (لا
    // يصح استدعاء String::c_str() أو أي تخصيص Heap داخل قسم حرج يعطّل
    // المقاطعات)، ثم ننسخها للمصفوفة الفعلية داخل قفل قصير جداً (نسخ بايتات
    // فقط، بلا أي استدعاء آخر بداخله).
    char newIds[5][sizeof(myTires[0].shortId)];
    char newLbls[5][sizeof(myTires[0].label)];
    bool hasLbl[5];
    for (int i = 0; i < 5; i++) {
      String idArg  = server.arg("id_" + String(i));
      String lblArg = server.arg("lbl_" + String(i));
      idArg.trim();
      idArg.toUpperCase(); // توحيد الحالة: idBuf في storageTask() دائماً حروف كبيرة،
                           // فيجب أن يكون shortId المخزَّن كذلك لتنجح المطابقة الدقيقة (tail match)
      // إصلاح أمان (XSS): shortId يُعرض لاحقاً بلا هروب HTML في أكثر من مكان
      // (بطاقة السيارة، الإعدادات). معرّفات الحساسات الحقيقية أبجدية رقمية
      // فقط أصلاً (idBuf المستخرَج من إشارات الراديو يُنقّى بنفس الطريقة
      // تماماً)، فتصفية أي حرف آخر هنا تمنع حقن HTML/JS من الجذر بدل
      // الاعتماد فقط على الهروب عند العرض، وتحسّن جودة البيانات أيضاً —
      // فمعرّف يحوي رموزاً غريبة لن يطابق أي حساس حقيقي بأي حال.
      {
        String cleaned;
        for (unsigned int k = 0; k < idArg.length(); k++) {
          if (isalnum((unsigned char)idArg[k])) cleaned += idArg[k];
        }
        idArg = cleaned;
      }
      lblArg.trim();
      strncpy(newIds[i], idArg.c_str(), sizeof(newIds[i]) - 1);
      newIds[i][sizeof(newIds[i]) - 1] = '\0';
      hasLbl[i] = lblArg.length() > 0;
      if (hasLbl[i]) {
        strncpy(newLbls[i], lblArg.c_str(), sizeof(newLbls[i]) - 1);
        newLbls[i][sizeof(newLbls[i]) - 1] = '\0';
      }
    }

    portENTER_CRITICAL(&tireDataMux);
    for (int i = 0; i < 5; i++) {
      memcpy(myTires[i].shortId, newIds[i], sizeof(myTires[i].shortId));
      if (hasLbl[i]) memcpy(myTires[i].label, newLbls[i], sizeof(myTires[i].label));
    }
    portEXIT_CRITICAL(&tireDataMux);

    saveConfigToFram();
    tireDataDirty = true;
  } else if (doWhat == "toggle_decoder") {
    int idx = server.arg("idx").toInt();
    if (idx >= 0 && idx < rtl_433_ESP::getTpmsDecoderCount()) {
      scannerDecodersMask ^= (1UL << idx);
      saveConfigToFram();
      syncDecoderEngineWithScreenMode();
    }
  } else if (doWhat == "decoders_all") {
    String state = server.arg("state");
    scannerDecodersMask = (state == "on") ? 0x01FFFFFFUL : 0x00000000UL;
    saveConfigToFram();
    syncDecoderEngineWithScreenMode();
  } else if (doWhat == "wipe_fram") {
    // إجراء مدمّر (يمسح كل بيانات FRAM) — لا يُقبل إلا عبر POST فعلي (طلب
    // fetch من confirmWipe() في صفحة الإعدادات)، وليس عبر GET/رابط عادي،
    // لمنع تنفيذه عرضياً بمجرد فتح/زيارة الرابط بأي وسيلة.
    if (server.method() != HTTP_POST) {
      server.send(405, "text/plain; charset=utf-8", "مرفوض: يتطلب هذا الإجراء طلب POST");
      return;
    }
    formatAndResetFRAM();
    server.send(200, "text/plain; charset=utf-8", "تم مسح ذاكرة FRAM بنجاح");
    return;
  } else if (doWhat == "rssi") {
    int dir = (server.arg("dir") == "up") ? RSSI_DELTA_STEP : -RSSI_DELTA_STEP;
    applyAndSaveRssiDelta(rssiDeltaCurrent + dir);
  } else if (doWhat == "wifipower") {
    int dir = (server.arg("dir") == "up") ? -1 : 1;
    wifiPowerIndex = clampWifiPowerIndex(wifiPowerIndex + dir);
    applyWifiPower();
    saveWifiPower();
  } else if (doWhat == "thr") {
    int idx = server.arg("idx").toInt();
    String field = server.arg("field");
    int dir = server.arg("dir").toInt();
    if (idx >= 0 && idx < 4 && dir != 0) {
      // إصلاح تزامن (Race Condition — قفل غير متماثل): كانت الكتابة هنا على
      // tireThresh[idx] تتم مباشرة بلا قفل، بينما storageTask() تقرأها عبر
      // computeLevel() من داخل قسم حرج قائم أصلاً (لتحديث tireState[])—
      // أي أن طرف القراءة يحترم القفل وطرف الكتابة لا، وهو تناقض يُبطل
      // فائدة القفل من الأساس (الحماية المتبادلة تتطلب التزام الطرفين معاً).
      portENTER_CRITICAL(&tireDataMux);
      TireThreshold& t = tireThresh[idx];
      if (field == "pmin") t.pMin = constrain(t.pMin + dir * P_STEP, P_ABS_MIN, t.pMax - P_MIN_GAP);
      else if (field == "pmax") t.pMax = constrain(t.pMax + dir * P_STEP, t.pMin + P_MIN_GAP, P_ABS_MAX);
      else if (field == "tmin") t.tMin = constrain(t.tMin + dir * T_STEP, T_ABS_MIN, t.tMax - T_MIN_GAP);
      else if (field == "tmax") t.tMax = constrain(t.tMax + dir * T_STEP, t.tMin + T_MIN_GAP, T_ABS_MAX);
      if (tireState[idx].hasData) tireState[idx].lvl = computeLevel(idx, tireState[idx].bar, tireState[idx].temp);
      if (tireState[idx].isLeaking) tireState[idx].lvl = 3;
      portEXIT_CRITICAL(&tireDataMux);
      saveThreshold(idx);
      tireDataDirty = true;
    }
  } else if (doWhat == "screen") {
    String m = server.arg("mode");
    screenMode = (m == "scanner") ? 1 : (m == "tech") ? 2 : (m == "radio") ? 3 : 0;
    syncDecoderEngineWithScreenMode();
  } else if (doWhat == "tech_clear") {
    portENTER_CRITICAL(&tireDataMux);
    memset(&techCap, 0, sizeof(techCap));
    techHistCount = 0;
    techRevision++;
    portEXIT_CRITICAL(&tireDataMux);
  } else if (doWhat == "tech_reset") {
    techReset();
  } else if (doWhat == "tech_thr") {
    int dir = (server.arg("dir") == "up") ? 5 : -5;
    techThresholdDbm = constrain(techThresholdDbm + dir, -100, -20);
    saveConfigToFram();
    if (screenMode == 2) lastDrawnScreenMode = -1; // إعادة رسم العنوان بالعتبة الجديدة
  } else if (doWhat == "rtc_sync") {
    if (server.hasArg("epoch")) {
      uint32_t epoch = strtoul(server.arg("epoch").c_str(), nullptr, 10);
      if (epoch > 1600000000UL && epoch < 4102444800UL) {
        rtcSetEpoch(epoch);
      }
    } else if (server.hasArg("datetime")) {
      String dtStr = server.arg("datetime");
      int y = 0, mo = 0, d = 0, hh = 0, mm = 0;
      if (sscanf(dtStr.c_str(), "%d-%d-%dT%d:%d", &y, &mo, &d, &hh, &mm) == 5) {
        rtcSetDateTime(y, mo, d, hh, mm, 0);
      }
    }
  } else if (doWhat == "ap_config") {
    String newSSID = server.arg("ssid");
    String newPass = server.arg("pass");
    // إصلاح: توحيد الطول مع حدود التخزين الفعلية في FRAM (32/64 حرفاً حسب
    // sizeof(cfg.apSSID)/sizeof(cfg.apPassword) في saveConfigToFram أدناه).
    // كان القيمتان تُستخدَمان بلا أي حد هنا مباشرة في WiFi.softAP()، بينما
    // FRAM كانت تقتطعهما عبر strncpy — فيعمل الجهاز فوراً بقيمة كاملة غير
    // مقتطَعة، ثم يُحمَّل بعد أي إعادة تشغيل بقيمة مقتطَعة مختلفة، فيفاجأ
    // المستخدم بعدم قدرته على الاتصال بنفس الاسم/كلمة المرور التي ضبطها
    // للتو. الاقتطاع هنا يضمن تطابق السلوك دائماً. علماً أن سمة maxlength
    // في نموذج HTML لا تحمي شيئاً فعلياً — أي طلب POST خام يتجاوزها بسهولة.
    if (newSSID.length() > 32) newSSID = newSSID.substring(0, 32);
    if (newPass.length() > 64) newPass = newPass.substring(0, 64);
    bool passProvided = newPass.length() > 0;
    bool passOk = !passProvided || newPass.length() >= 8;
    if (newSSID.length() > 0 && passOk) {
      apSSID = newSSID;
      if (passProvided) apPassword = newPass;
      saveConfigToFram();
      WiFi.softAP(apSSID.c_str(), apPassword.c_str());
      applyWifiPower();
      server.sendHeader("Location", "/settings?view=general");
      server.send(303);
      return;
    }
  }

  String view = server.hasArg("view") ? server.arg("view") : "";
  String tab  = server.hasArg("tab") ? server.arg("tab") : "";
  // إصلاح أمان (HTTP Header/Response Splitting): view/tab تأتي مباشرة من
  // مدخلات المستخدم (بعد فك ترميز URL، فقد تحوي %0D%0A كأحرف CR/LF فعلية)
  // وتُستخدَم مباشرة في قيمة ترويسة Location أدناه. لا يصح الاعتماد على أن
  // مكتبة WebServer ستُصفّي هذه الأحرف تلقائياً — نزيلها هنا صراحة كخط دفاع
  // مستقل بغض النظر عن سلوك المكتبة الداخلي.
  view.replace("\r", ""); view.replace("\n", "");
  tab.replace("\r", "");  tab.replace("\n", "");
  String backTo = "/";
  if (doWhat == "tech_clear" || doWhat == "tech_reset" || doWhat == "tech_thr" || (doWhat == "screen" && server.arg("mode") == "tech")) backTo = "/tech";
  if (doWhat.startsWith("profile_")) backTo = "/settings?view=profiles";
  if (doWhat.startsWith("orig_")) backTo = "/settings?view=tires";
  if (doWhat.startsWith("smart_") || doWhat.startsWith("base_") || doWhat.startsWith("tune_")) backTo = "/settings?view=smart";
  if (doWhat.startsWith("learn_")) backTo = (doWhat == "learn_apply" || doWhat == "learn_close") ? "/" : "/learn";
  if (doWhat == "toggle_decoder" || doWhat == "decoders_all") {
    backTo = "/settings?view=decoders";
  } else if (doWhat == "tires_config") {
    backTo = "/settings?view=tires";
  } else if (doWhat == "units" || doWhat == "buzzer" || doWhat == "beeptest" ||
             doWhat == "rxbw" || doWhat == "foff" || doWhat == "cpufreq" || doWhat == "rssi" || doWhat == "wifipower" || doWhat == "thr" || doWhat == "rtc_sync" || doWhat == "freqdev" || doWhat == "stale_timeout") {
    backTo = "/settings";
    if (view.length()) backTo += "?view=" + view + (tab.length() ? "&tab=" + tab : "");
  }
  server.sendHeader("Location", backTo);
  server.send(303);
}

// ---------------- 12. مهمة الراديو على Core 0 ----------------
void rfLoopTask(void* pvParameters) {

  pinMode(CC1101_CSN, OUTPUT);
  pinMode(CC1101_SCK, OUTPUT);
  pinMode(CC1101_MOSI, OUTPUT);
  pinMode(CC1101_MISO, INPUT);
  digitalWrite(CC1101_CSN, HIGH);
  delay(100);

  rf.setCallback(logSignal, messageBuffer, sizeof(messageBuffer));

  esp_task_wdt_add(NULL);
  bool firstStart = true;
  uint32_t lastAttempt = 0;
  uint32_t lastHealth = 0;

  for (;;) {
    esp_task_wdt_reset();
    if (!isCC1101Ok) {
      // تهيئة (أو إعادة تهيئة بعد فشل فحص الصحة) كل 3 ثوانٍ حتى ينجح الراديو
      if (firstStart || millis() - lastAttempt >= 3000) {
        lastAttempt = millis();
        if (!firstStart) { rf.end(); ccRecoveryCounter++; }
        firstStart = false;
        rtl_433_ESP::setRxBandwidth(currentRxBwKhz()); // تُطبَّق داخل begin
        rf.setRSSIThreshold(rssiDeltaCurrent);
        rtl_433_ESP::setPins(CC1101_SCK, CC1101_MISO, CC1101_MOSI, CC1101_CSN, CC1101_GDO0, CC1101_GDO2);
        // مخزن الإشارات: DEV_CAPTURE_SLOTS + DEV_DECODE_SLOTS خانة (≈4.9KB لكل خانة من الذاكرة الداخلية)
        rtl_433_ESP::setBufferSlots(DEV_CAPTURE_SLOTS, DEV_DECODE_SLOTS);
        bool radioOk = rf.begin(CC1101_GDO0, currentFreqMhz());
        if (radioOk) {
          rf.setNotifyTask(xTaskGetCurrentTaskHandle());
          rf.enableReceiver();
          rf.setFrequencyDeviation((float)freqDevCurrent);
          isCC1101Ok = true;
          cc1101ErrCode = 0;
          lastHealth = millis();
          syncDecoderEngineWithScreenMode();
          Serial.printf("[RF] CC1101 ready @ %.3f MHz, BW %d kHz, delta %d dB\n",
                        currentFreqMhz(), (int)currentRxBwKhz(), rssiDeltaCurrent);
        } else {
          cc1101ErrCode = (int)rf.lastError();
          Serial.printf("[RF] CC1101 init failed (err %d), retrying\n", cc1101ErrCode);
        }
        statusBarDirty = true;
      }
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
    rf.loop(); // يُستدعى دائماً حتى لو فات إشعار، فلا تتراكم الإشارات في الحلقة

    if (millis() - lastHealth >= 5000) {
      lastHealth = millis();
      if (!rtl_433_ESP::isRadioAlive()) {
        isCC1101Ok = false;
        cc1101ErrCode = -99;
        statusBarDirty = true;
        Serial.println("[RF] CC1101 health check failed, reinitialising");
      }
    }
  }
}

// ---------------- 12.5 مراقب استقرار النظام (Task Watchdog) ----------------
// نظام حياتي يجب ألا يتجمّد بصمت أبداً. هذه الدالة تُهيّئ Task Watchdog Timer
// (TWDT) بطريقة متوافقة تلقائياً مع كلا إصداري ESP32 Arduino Core الظاهرين
// في Boards Manager لديك (2.0.18 يبني على ESP-IDF v4.x، و4.0.0-alpha يبني
// على ESP-IDF v5.x) — الفحص يتم وقت الترجمة عبر ESP_IDF_VERSION_MAJOR، فلا
// حاجة لمعرفة أيهما مُختار فعلياً في Tools->Board؛ الكود يصحّح نفسه تلقائياً.
// ⚠️ تنبيه مهم منفصل: النسخة الثانية المثبَّتة لديك (4.0.0-alpha1) هي نسخة
// "ألفا" تجريبية غير مستقرة من مطوّرها الرسمي (Espressif) — غير مناسبة
// إطلاقاً لمشروع تعتمد عليه للسلامة. استخدم فقط "Arduino ESP32 Boards"
// (2.0.18) المستقرة عند اختيار اللوحة من Tools->Board، وأزل نسخة الألفا من
// Boards Manager لتفادي اختيارها بالخطأ لاحقاً.
#define APP_WDT_TIMEOUT_S 30

void setupWatchdog() {
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_config_t wdtConfig = {};
  wdtConfig.timeout_ms = APP_WDT_TIMEOUT_S * 1000;
  wdtConfig.idle_core_mask = 0; // لا نراقب مهام الخمول، فقط مهامنا المسجَّلة صراحة أدناه
  wdtConfig.trigger_panic = true; // عند انتهاء المهلة: إعادة إقلاع فورية بدل تجمّد صامت
  esp_err_t err = esp_task_wdt_init(&wdtConfig);
  if (err == ESP_ERR_INVALID_STATE) {
    esp_task_wdt_reconfigure(&wdtConfig);
  }
#else
  esp_task_wdt_init(APP_WDT_TIMEOUT_S, true);
#endif
}

// ---------------- 13. الإعداد العام (SETUP على Core 1) ----------------
void setup() {
  setCpuFrequencyMhz(240);

  // يُهيَّأ الـWatchdog أولاً قبل أي شيء آخر، وتُسجَّل مهمة الحلقة الرئيسية
  // (setup+loop) فوراً — بما يغطي حتى مسار الفشل المبكر أدناه (توقف صامت
  // كامل عند فشل إنشاء الطوابير/الأقفال) بدل تجمّد أبدي دون أي استشفاء.
  setupWatchdog();
  esp_task_wdt_add(NULL);

  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  Serial.setRxBufferSize(1024);
  Serial.begin(115200);
  delay(200);
  Serial.printf("\n[SYSTEM] TPMS Professional V4 (%s %s) | heap %u KB | chip rev %d\n",
                __DATE__, __TIME__, ESP.getFreeHeap() / 1024, ESP.getChipRevision());

  csrfToken = generateCsrfToken();

  // طابور راديو محسن بحجم 32 عنصراً (يوفر 25KB من الـ RAM الداخلية)
  rfQueue = xQueueCreate(32, sizeof(RawRfPacket));
  framQueue = xQueueCreate(32, sizeof(FramJob));
  i2cMutex = xSemaphoreCreateMutex();
  logMutex = xSemaphoreCreateMutex();
  tftMutex = xSemaphoreCreateMutex();
  
  if (!rfQueue || !framQueue || !i2cMutex || !logMutex || !tftMutex) {
    // إصلاح أمان: كانت هذه حلقة تجمّد أبدي صامت تمامًا (بلا شاشة، بلا
    // استرداد) عند فشل التخصيص المبكر. بما أن مهمة الحلقة الرئيسية مسجَّلة
    // الآن في الـWatchdog، عمداً لا نُطعمه هنا (لا استدعاء esp_task_wdt_reset)
    // — فيُعيد النظام إقلاعه تلقائيًا خلال APP_WDT_TIMEOUT_S ثانية بدل البقاء
    // معطلاً للأبد، مانحًا فرصة حقيقية للاستشفاء من فشل عابر بدل توقف دائم.
    while (true) { delay(1000); }
  }

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000);

  framOk = framDetect();
  rtcOk  = rtc.begin(&Wire);
  if (rtcOk) {
    if (rtc.lostPower()) {
      rtc.adjust(DateTime(__DATE__, __TIME__));
    }
    syncRtcTime();
  }

  loadConfigFromFram();
  loadPersistedTireStates();
  loadLogHeader();
  smartLoad();
  baselineLoad();
  originalsLoad();
  Serial.printf("[SYSTEM] FRAM %s | RTC %s | BW %d kHz | delta %d | freq %.3f MHz\n",
                framOk ? "OK" : "MISSING", rtcOk ? "OK" : "MISSING", (int)currentRxBwKhz(), rssiDeltaCurrent, currentFreqMhz());
  if (framOk) saveConfigToFram(); // حفظ أي ترقية إعدادات تمت أثناء التحميل

  pinMode(TFT_CS, OUTPUT);
  digitalWrite(TFT_CS, HIGH);
  tftSPI.begin(TFT_SCK, TFT_MISO, TFT_MOSI, TFT_CS);
  tft.init(SCREEN_WIDTH, SCREEN_HEIGHT);
  tft.setSPISpeed(40000000);
  tft.setRotation(0);
  safeTftFillScreen(ST77XX_BLACK);
  digitalWrite(TFT_CS, HIGH);

  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  WiFi.mode(WIFI_OFF);

  server.on("/", handleRoot);
  server.on("/scanner", handleScanner);
  server.on("/tech", handleTech);
  server.on("/tech/csv", handleTechCsv);
  server.on("/tech/report", handleTechReport);
  server.on("/learn", handleLearn);
  server.on("/chart", handleChart);
  server.on("/history", handleHistory);
  server.on("/history/csv", handleHistoryCSV);
  server.on("/settings", handleSettings);
  server.on("/action", handleAction);
  httpUpdater.setup(&server, "/update", "admin", apPassword.c_str());
  // فحص الاتصال في Android/iOS/Windows وأي مسار مجهول: تحويل للصفحة الرئيسية (بوابة تلقائية)
  auto captive = []() {
    server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
    server.send(302, "text/plain", "");
  };
  server.on("/generate_204", captive);
  server.on("/gen_204", captive);
  server.on("/hotspot-detect.html", captive);
  server.on("/library/test/success.html", captive);
  server.on("/connecttest.txt", captive);
  server.on("/ncsi.txt", captive);
  server.on("/redirect", captive);
  server.onNotFound(captive);

  xTaskCreatePinnedToCore(rfLoopTask, "rfLoopTask", 8192, NULL, 3, NULL, 1);   // النواة 1: مقاطعة الراديو تُربط هنا بعيداً عن الواي فاي
  xTaskCreatePinnedToCore(storageTask, "storageTask", 8192, NULL, 3, NULL, 1);
  xTaskCreatePinnedToCore(framWriterTask, "framWriter", 4096, NULL, 1, NULL, 1);

  screenMode = 0;
  syncDecoderEngineWithScreenMode();

  topBarDirty = true;
  drawBaseCarLayout();
  updateTireReadouts();
  updateTopBar();
  updateStatusBar();
  lastDrawnScreenMode = screenMode;
}

// ---------------- 13.5 أوامر وتشخيص المنفذ التسلسلي ----------------
void printStatusLine() {
  rtl_433_ESPStatus st = rtl_433_ESP::statusSnapshot();
  Serial.printf("[STAT] up %lus | mode %d | pkts %lu rej %lu | captured %u decoded %u lost %u/%u/%lu | noise %d thr %d | "
                "heap %u KB (min %u) | radio %s resets %lu | fram %d rtc %d | cpu %u MHz wifi %.1f dBm\n",
                (unsigned long)(millis() / 1000), screenMode, (unsigned long)rfPacketCounter, (unsigned long)rfRejectedCounter,
                st.capturedSignals, st.decodedMessages, st.droppedCaptureBuffers, st.droppedDecoderQueue,
                (unsigned long)rfQueueDropCounter, st.averageRssi, st.rssiThreshold,
                ESP.getFreeHeap() / 1024, ESP.getMinFreeHeap() / 1024, isCC1101Ok ? "OK" : "FAIL",
                (unsigned long)ccRecoveryCounter, framOk, rtcOk, (unsigned)getCpuFrequencyMhz(), WIFI_POWER_DBM[wifiPowerIndex]);
  uint32_t dn = rtl_433_ESP::decodeCount;
  Serial.printf("[PERF] decode avg %.1f ms max %.1f ms | queue peak %lu/%u | slots %u+%u | lost capture %u queue %u\n",
                dn ? rtl_433_ESP::decodeUsTotal / 1000.0 / dn : 0.0, rtl_433_ESP::decodeUsMax / 1000.0,
                (unsigned long)rtl_433_ESP::queuePeak, rtl_433_ESP::decodeSlots(), rtl_433_ESP::captureSlots(),
                rtl_433_ESP::decodeSlots(), rtl_433_ESP::droppedCaptureBuffers, rtl_433_ESP::droppedDecoderQueue);
}

// اختبار ذاتي لصفحات الويب: مهمة منفصلة تطلب الصفحة من خادم الجهاز نفسه
static char webTestPath[96];
static volatile bool webTestBusy = false;
static void webTestTask(void* arg) {
  WiFiClient c;
  uint32_t t0 = millis();
  if (!c.connect(IPAddress(127, 0, 0, 1), 80, 3000)) {
    Serial.printf("[WEBTEST] %s -> connect failed\n", webTestPath);
    vTaskDelete(NULL);
    return;
  }
  c.printf("GET %s HTTP/1.1\r\nHost: 192.168.4.1\r\nConnection: close\r\n\r\n", webTestPath);
  char status[48] = "";
  char tail[64] = "";
  size_t bytes = 0, sl = 0;
  bool inStatus = true;
  uint32_t last = millis();
  while ((c.connected() || c.available()) && millis() - last < 8000) {
    int ch = c.read();
    if (ch < 0) { delay(2); continue; }
    last = millis();
    bytes++;
    if (inStatus) {
      if (ch == '\r' || ch == '\n') inStatus = false;
      else if (sl < sizeof(status) - 1) status[sl++] = (char)ch;
    }
    memmove(tail, tail + 1, sizeof(tail) - 2);
    tail[sizeof(tail) - 2] = (ch == '\n' || ch == '\r') ? ' ' : (char)ch;
  }
  c.stop();
  status[sl] = 0;
  Serial.printf("[WEBTEST] %s -> \"%s\" %u bytes in %lu ms | heap %u KB (largest %u KB) | tail: %s\n", webTestPath, status,
                (unsigned)bytes, (unsigned long)(millis() - t0), ESP.getFreeHeap() / 1024, heap_caps_get_largest_free_block(MALLOC_CAP_8BIT) / 1024, tail);
  webTestBusy = false;
  vTaskDelete(NULL);
}

void handleSerialCommands() {
  static char line[320];
  static int len = 0;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch == '\n' || ch == '\r') {
      line[len] = '\0';
      if (len > 0) {
        if (strcmp(line, "stat") == 0) printStatusLine();
        else if (strncmp(line, "mode ", 5) == 0) { screenMode = constrain(atoi(line + 5), 0, SCREEN_MODE_COUNT - 1); Serial.printf("[CMD] mode %d\n", screenMode); }
        else if (strncmp(line, "bw ", 3) == 0) {
          float want = atof(line + 3);
          int best = 0;
          for (int i = 1; i < CC1101_BW_COUNT; i++) if (fabsf(CC1101_BW_LIST[i] - want) < fabsf(CC1101_BW_LIST[best] - want)) best = i;
          applyAndSaveRxBw(best);
          Serial.printf("[CMD] bw %d kHz\n", (int)currentRxBwKhz());
        }
        else if (strncmp(line, "rssi ", 5) == 0) { applyAndSaveRssiDelta(atoi(line + 5)); Serial.printf("[CMD] rssi delta %d\n", rssiDeltaCurrent); }
        else if (strncmp(line, "foff ", 5) == 0) { applyAndSaveFreqOffset(atoi(line + 5)); Serial.printf("[CMD] freq %.3f MHz\n", currentFreqMhz()); }
        else if (strncmp(line, "techthr ", 8) == 0) { techThresholdDbm = constrain(atoi(line + 8), -100, -20); saveConfigToFram(); lastDrawnScreenMode = -1; Serial.printf("[CMD] tech threshold %d\n", techThresholdDbm); }
        else if (strcmp(line, "techreset") == 0) { techReset(); Serial.println("[CMD] tech reset"); }
        else if (strcmp(line, "tech") == 0) {
          TechCapture c; int hc; char prevIds[TECH_HIST][24];
          portENTER_CRITICAL(&tireDataMux);
          c = techCap; hc = techHistCount;
          for (int i = 0; i < hc; i++) memcpy(prevIds[i], techHist[i].id, sizeof(prevIds[i]));
          portEXIT_CRITICAL(&tireDataMux);
          if (c.valid) Serial.printf("[TECH] %s %s bar=%.2f rssi=%d best=%d pkts=%lu verdict=%s\n", c.id, c.model,
                                     c.bar, c.rssi, c.bestRssi, (unsigned long)c.count, techVerdict(c, nullptr, nullptr));
          else Serial.println("[TECH] no capture");
          for (int i = 0; i < hc; i++) Serial.printf("[TECH] prev%d %s\n", i, prevIds[i]);
        }
        else if (strcmp(line, "smart") == 0) {
          Serial.printf("[SMART] mode %d sessions %u active %u proposal %d | baseline %s | tune %s next %ldmin last: %s\n",
                        autoSensorsMode, smartHdr.sessionCounter, smartHdr.activeSessions, smartProposal.valid,
                        baselineEnabled ? "on" : "off", autoTuneEnabled ? (tuneRunning ? "running" : "on") : "off",
                        (long)((int32_t)(tuneNextMs - millis()) / 60000L), tuneLastResult.c_str());
          for (int i = 0; i < SMART_MAX_SENSORS; i++) if (smartTab[i].id[0])
            Serial.printf("[SMART] %s %s qual=%u lastAct=%u first=%u confirmed=%d slot=%d\n", smartTab[i].id, smartTab[i].model,
                          smartTab[i].qualSessions, smartTab[i].lastQualActive, smartTab[i].activeAtFirst, smartConfirmed(smartTab[i]), smartAssignedSlot(smartTab[i].id));
          for (int i = 0; i < 5; i++) Serial.printf("[BASE] %d %.2f bar n=%lu ready=%d\n", i, tireBaseline[i].coldBar, (unsigned long)tireBaseline[i].samples, baselineReady(tireBaseline[i]));
        }
        else if (strncmp(line, "cpu ", 4) == 0) { applyAndSaveCpuFreq(atoi(line + 4)); Serial.printf("[CMD] cpu %u MHz\n", (unsigned)getCpuFrequencyMhz()); }
        else if (strncmp(line, "wifipower ", 10) == 0) {
          float want = atof(line + 10);
          int best = 0;
          for (int i = 1; i < WIFI_POWER_LEVELS_COUNT; i++) if (fabsf(WIFI_POWER_DBM[i] - want) < fabsf(WIFI_POWER_DBM[best] - want)) best = i;
          wifiPowerIndex = clampWifiPowerIndex(best);
          if (wifiEnabled) applyWifiPower();
          saveWifiPower();
          Serial.printf("[CMD] wifi power %.1f dBm\n", WIFI_POWER_DBM[wifiPowerIndex]);
        }
        else if (strncmp(line, "inject ", 7) == 0) { logSignal(line + 7); }
        else if (strcmp(line, "newsession") == 0) { smartNewSession("manual"); }
        else if (strcmp(line, "smartreset") == 0) { smartReset(); Serial.println("[CMD] smart data cleared"); }
        else if (strcmp(line, "basereset") == 0) { baselineReset(-1); Serial.println("[CMD] baselines cleared"); }
        else if (strcmp(line, "smartapply") == 0) { smartApplyProposal(false); }
        else if (strncmp(line, "settire ", 8) == 0) {
          int sl = line[8] - '0';
          const char* id = line + 10;
          if (sl >= 0 && sl <= 4 && strlen(line) > 10) {
            portENTER_CRITICAL(&tireDataMux);
            memset(myTires[sl].shortId, 0, sizeof(myTires[sl].shortId));
            strncpy(myTires[sl].shortId, id, sizeof(myTires[sl].shortId) - 1);
            portEXIT_CRITICAL(&tireDataMux);
            saveConfigToFram();
            Serial.printf("[CMD] tire %d = %s\n", sl, myTires[sl].shortId);
          }
        }
        else if (strcmp(line, "restore") == 0) { originalsRestore(); }
        else if (strcmp(line, "originals") == 0) { for (int i = 0; i < 5; i++) Serial.printf("[ORIG] %d %s current=%s same=%d\n", i, originals.ids[i], myTires[i].shortId, slotHasOriginal(i)); }
        else if (strcmp(line, "tires") == 0) { for (int i = 0; i < 5; i++) Serial.printf("[TIRE] %d %s %s\n", i, myTires[i].label, myTires[i].shortId); }
        else if (strncmp(line, "autosens ", 9) == 0) { autoSensorsMode = constrain(atoi(line + 9), 1, 3); saveConfigToFram(); Serial.printf("[CMD] auto sensors mode %d\n", autoSensorsMode); }
        else if (strcmp(line, "tune now") == 0) { tuneNextMs = millis(); Serial.println("[CMD] tune scheduled"); }
        else if (strcmp(line, "learn") == 0) { learnStart(); lastDrawnScreenMode = -1; }
        else if (strcmp(line, "learnstop") == 0) { if (learnActive) learnFinish(); }
        else if (strcmp(line, "beep") == 0) { beep(2, 120, 120); Serial.println("[CMD] beep"); }
        else if (strncmp(line, "unit ", 5) == 0) { unitPressure = constrain(atoi(line + 5), 0, 2); saveConfigToFram(); lastDrawnScreenMode = -1; Serial.printf("[CMD] pressure unit %s\n", pUnit()); }
        else if (strncmp(line, "webtest ", 8) == 0) {
          if (!wifiEnabled) Serial.println("[WEBTEST] wifi is off (use: wifi on)");
          else if (webTestBusy) Serial.println("[WEBTEST] busy");
          else {
            webTestBusy = true;
            strncpy(webTestPath, line + 8, sizeof(webTestPath) - 1);
            webTestPath[sizeof(webTestPath) - 1] = 0;
            xTaskCreatePinnedToCore(webTestTask, "webTest", 6144, NULL, 5, NULL, 0);
          }
        }
        else if (strcmp(line, "wifi on") == 0)  { if (!wifiEnabled) startWifiAP(); Serial.println("[CMD] wifi on"); }
        else if (strcmp(line, "wifi off") == 0) { if (wifiEnabled) stopWifiAP(); Serial.println("[CMD] wifi off"); }
        else if (strcmp(line, "log off") == 0) { serialPacketLog = false; Serial.println("[CMD] packet log off"); }
        else if (strcmp(line, "log on") == 0)  { serialPacketLog = true;  Serial.println("[CMD] packet log on"); }
        else if (strcmp(line, "reboot") == 0) ESP.restart();
        else Serial.printf("[CMD] unknown: %s (stat | mode N | bw N | rssi N | foff N | techthr N | techreset | tech | learn | learnstop | beep | unit N | wifi on/off | log on/off | reboot)\n", line);
      }
      len = 0;
    } else if (len < (int)sizeof(line) - 1) {
      line[len++] = ch;
    }
  }
}

// ---------------- 14. الحلقة الرئيسية (Core 1) ----------------
unsigned long lastMinuteTick = 0;
unsigned long lastScreenRefresh = 0;
static const uint32_t SCREEN_REFRESH_MS = 150;

void loop() {
  // إطعام Watchdog النظام: هذه المهمة (الحلقة الرئيسية) مسجَّلة الآن عبر
  // esp_task_wdt_add(NULL) في setup(). loop() تتكرر بلا أي انتظار محظور
  // (server.handleClient غير محجوبة، وكل الفحوصات أدناه فورية)، فإطعامه
  // هنا مرة كل تكرار آمن تماماً ولا يخاطر بإنذار كاذب أثناء التشغيل الطبيعي.
  esp_task_wdt_reset();

  handleSmartButton();
  handleSerialCommands();

  confirmOtaImageIfHealthy(isCC1101Ok);
  serviceBuzzerAndLed();

  // مراقبة الإنذار: إطار في خطر (ضغط منخفض/مرتفع/تسريب) وإشارته حديثة
  static uint32_t lastAlarmCheck = 0, lastAlarmBeep = 0;
  static bool prevAlarm = false;
  if (millis() - lastAlarmCheck >= 500) {
    lastAlarmCheck = millis();
    bool active = false;
    uint32_t nowSec = epochNow();
    portENTER_CRITICAL(&tireDataMux);
    for (int i = 0; i < 5; i++) {
      const TireState& t = tireState[i];
      if (!t.hasData || (nowSec - t.lastSeenEpoch) > staleSignalTimeoutSec) continue;
      if (t.lvl >= 1 && t.lvl <= 4) active = true;
    }
    portEXIT_CRITICAL(&tireDataMux);
    if (active && !prevAlarm) {
      alarmAcked = false;
      beep(3, 350, 200);
      lastAlarmBeep = millis();
    } else if (active && !alarmAcked && millis() - lastAlarmBeep >= 60000UL) {
      beep(3, 350, 200);
      lastAlarmBeep = millis();
    }
    if (!active) alarmAcked = false;
    prevAlarm = active;
    alarmActive = active;
  }

  smartService();

  // شاشة اقتراح الحساسات (وضع التأكيد بالزر): تظهر 60 ثانية
  static bool smartScreenShown = false;
  static uint32_t lastSmartDraw = 0;
  if (smartProposal.valid && (int32_t)(smartPromptUntil - millis()) > 0 && !learnActive) {
    if (!smartScreenShown) { drawSmartPrompt(true); smartScreenShown = true; }
    else if (millis() - lastSmartDraw >= 1000) { lastSmartDraw = millis(); drawSmartPrompt(false); }
    delay(1);
    if (wifiEnabled) { dnsServer.processNextRequest(); server.handleClient(); }
    return;
  } else if (smartScreenShown) {
    smartScreenShown = false;
    lastDrawnScreenMode = -1;
  }

  // التعلّم: انتهاء المدة + تحديث الشاشة
  if (learnActive && millis() - learnStartMs >= LEARN_DURATION_MS) learnFinish();
  static bool learnScreenShown = false;
  static uint32_t lastLearnDraw = 0;
  if (learnActive || learnDone) {
    if (!learnScreenShown) { drawLearnScreen(true); learnScreenShown = true; lastLearnDraw = millis(); }
    else if (millis() - lastLearnDraw >= 1000) { lastLearnDraw = millis(); drawLearnScreen(!learnActive && lastLearnDraw - learnStartMs < LEARN_DURATION_MS + 1500); }
    delay(1);
    if (wifiEnabled) { dnsServer.processNextRequest(); server.handleClient(); }
    return;
  } else if (learnScreenShown) {
    learnScreenShown = false;
    lastDrawnScreenMode = -1; // العودة للشاشة السابقة
  }

  static uint32_t lastStat = 0;
  if (millis() - lastStat >= 60000) {
    lastStat = millis();
    printStatusLine();
  }

  if (wifiEnabled) { dnsServer.processNextRequest(); server.handleClient(); }

  if (screenMode != lastDrawnScreenMode) {
    syncDecoderEngineWithScreenMode();
    if (screenMode == 0) {
      drawBaseCarLayout();
      updateTireReadouts();
    } else if (screenMode == 2) {
      drawTechLayout();
      updateTechValues(true);
    } else if (screenMode == 3) {
      drawRadioLayout();
    } else {
      drawScannerLayout();
      updateScannerValues();
    }
    lastDrawnScreenMode = screenMode;
    topBarDirty = true;
    tireDataDirty = true;
    statusBarDirty = true;
  }

  if (topBarDirty && screenMode == 0) updateTopBar();

  static int radioPeak = -128;
  if (rtl_433_ESP::currentRssi > radioPeak) radioPeak = rtl_433_ESP::currentRssi;
  if (screenMode == 3) {
    static uint32_t lastRadioDraw = 0;
    if (millis() - lastRadioDraw >= 150) {
      lastRadioDraw = millis();
      updateRadioValues(radioPeak);
      radioPeak = -128;
    }
    tireDataDirty = false;
  } else if (screenMode == 2) {
    // وضع الفني: رسم كامل عند تغيّر الالتقاط، وتحديث المقياس/العمر دورياً
    static uint32_t drawnRevision = 0xFFFFFFFF;
    static uint32_t lastTechTick = 0;
    static bool flashShown = false;
    bool flashing = (int32_t)(techFlashUntil - millis()) > 0;
    if (techRevision != drawnRevision || (flashShown && !flashing)) {
      drawnRevision = techRevision;
      flashShown = flashing;
      updateTechValues(true);
      lastTechTick = millis();
    } else if (millis() - lastTechTick >= (techCap.valid ? 1000UL : 200UL)) {
      lastTechTick = millis();
      updateTechValues(false);
    }
    tireDataDirty = false;
  } else if (tireDataDirty && (millis() - lastScreenRefresh >= SCREEN_REFRESH_MS)) {
    tireDataDirty = false;
    lastScreenRefresh = millis();
    if (screenMode == 0) updateTireReadouts();
    else updateScannerValues();
  }
  else if (millis() - lastMinuteTick >= 60000) {
    lastMinuteTick = millis();
    syncRtcTime();
    if (screenMode == 0) updateTireReadouts();
    else if (screenMode == 1) updateScannerValues();
  }

  static uint32_t lastStatusRefresh = 0;
  if (statusBarDirty && (millis() - lastStatusRefresh >= 500)) {
    statusBarDirty = false;
    lastStatusRefresh = millis();
    updateStatusBar();
  }

  delay(1);
}