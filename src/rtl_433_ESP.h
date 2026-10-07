/*
  TPMS_RTL433 — مكتبة موحّدة لاستقبال حساسات ضغط الإطارات (TPMS) على 433.92MHz
  ESP32 / ESP32-S3 + CC1101 (FSK). مبنية على rtl_433_ESP (NorthernMan54) و rtl_433 (merbanan).

  المنافذ قابلة للضبط وقت التشغيل عبر rtl_433_ESP::setPins() قبل begin().
  الافتراضي حسب نوع الشريحة:
    ESP32-S3 (R3):  SCK=12 MISO=13 MOSI=11 CS=10 GDO0=4  GDO2=5   (FSPI)
    ESP32   (DEV):  SCK=22 MISO=21 MOSI=25 CS=26 GDO0=32 GDO2=33  (VSPI)
*/

#ifndef rtl_433_ESP_H
#define rtl_433_ESP_H

#define TPMS_RTL433_VERSION "1.1.0"

#ifndef RF_CC1101
#  define RF_CC1101
#endif
#if defined(CONFIG_IDF_TARGET_ESP32S3) || defined(CONFIG_IDF_TARGET_ESP32C3)
#  define TPMS_DEFAULT_PIN_SCK  12
#  define TPMS_DEFAULT_PIN_MISO 13
#  define TPMS_DEFAULT_PIN_MOSI 11
#  define TPMS_DEFAULT_PIN_CS   10
#  define TPMS_DEFAULT_PIN_GDO0 4
#  define TPMS_DEFAULT_PIN_GDO2 5
#else
#  define TPMS_DEFAULT_PIN_SCK  22
#  define TPMS_DEFAULT_PIN_MISO 21
#  define TPMS_DEFAULT_PIN_MOSI 25
#  define TPMS_DEFAULT_PIN_CS   26
#  define TPMS_DEFAULT_PIN_GDO0 32
#  define TPMS_DEFAULT_PIN_GDO2 33
#endif
// للتوافق مع الكود القديم
#ifndef RF_MODULE_CS
#  define RF_MODULE_CS   TPMS_DEFAULT_PIN_CS
#endif
#ifndef RF_MODULE_GDO0
#  define RF_MODULE_GDO0 TPMS_DEFAULT_PIN_GDO0
#endif
#ifndef RF_MODULE_GDO2
#  define RF_MODULE_GDO2 TPMS_DEFAULT_PIN_GDO2
#endif
#ifndef RF_MODULE_SCK
#  define RF_MODULE_SCK  TPMS_DEFAULT_PIN_SCK
#endif
#ifndef RF_MODULE_MISO
#  define RF_MODULE_MISO TPMS_DEFAULT_PIN_MISO
#endif
#ifndef RF_MODULE_MOSI
#  define RF_MODULE_MOSI TPMS_DEFAULT_PIN_MOSI
#endif
#ifndef RF_MODULE_FREQUENCY
#  define RF_MODULE_FREQUENCY 433.92
#endif
#ifndef OOK_MODULATION
#  define OOK_MODULATION false
#endif

#include <Arduino.h>
#include <SPI.h>
#include <functional>
#include "log.h"
#include "tools/aprintf.h"

#ifndef ICACHE_RAM_ATTR
#  define ICACHE_RAM_ATTR IRAM_ATTR
#endif

#define RADIOLIB_LOW_LEVEL 1
#include <RadioLib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#ifndef MINRSSI
#  define MINRSSI -82
#endif

// خفض العينات إلى 5000 للتكيف الفوري مع ضوضاء الزحام
#ifndef RSSI_SAMPLES
#  define RSSI_SAMPLES 5000
#endif

#ifndef RSSI_THRESHOLD
#  define RSSI_THRESHOLD 9
#endif

// أثناء تتبع نهاية إشارة فعلية (receiveMode == true): الفجوة بالميكروثانية
// بين قراءات RSSI المتتالية بدل تأخير تِك كامل (1ms). كل ما قلّت الرقم زادت
// دقة اكتشاف نهاية الإشارة، لكن زاد استهلاك معالج core0 في تلك اللحظات
// القصيرة فقط (مدة الإشارة الواحدة عادة مايكروثواني إلى بضع مللي ثانية).
#ifndef REALTIME_POLL_GAP_US
#  define REALTIME_POLL_GAP_US 20
#endif

// صمام أمان: أقصى عدد دورات فحص سريع متتالية أثناء receiveMode قبل إجبار
// المهمة على تنفيذ vTaskDelay(1) حقيقي مرة واحدة، لضمان عدم تجويع مهمة
// الخمول على Core 0 مهما طالت مدة الإشارة (تحسباً لضجيج RF غير طبيعي).
// القيمة الافتراضية 250 × 20us ≈ 5ms فحص سريع متواصل كحد أقصى قبل كل تِك
// ضمان إجباري، أي أقل بكثير جداً من مهلة Task Watchdog الافتراضية (~5s).
#ifndef REALTIME_POLL_WATCHDOG_GUARD
#  define REALTIME_POLL_WATCHDOG_GUARD 250
#endif

// أثناء وضع الخمول (receiveMode == false، بانتظار بداية أي إشارة): كانت
// النسخة السابقة تنام تِك كامل (vTaskDelay(1) ≈ 1ms) بين كل قراءة RSSI
// وأخرى. إشارات TPMS القصيرة جداً قد "تفوت" فرصة اكتشافها إن جاءت بالكامل
// بين تكتين متتاليين (حتى 1ms فجوة عمياء)، فيضطر المستشعر لإعادة الإرسال
// (كل بضع ثوانٍ عادة) قبل أن يُلتقط فعلياً — يُحسّ هذا كبطء في الاكتشاف.
// خفضنا الفجوة هنا لزيادة معدل الفحص أثناء الخمول أيضاً (وليس فقط أثناء
// تتبع نهاية إشارة قائمة فعلاً)، بتكلفة زيادة طفيفة جداً في استهلاك
// معالج/طاقة Core 0 المستمر (قابلة للضبط أو التراجع عنها إن كانت الحرارة
// أهم من سرعة الاكتشاف في حالتك).
#ifndef IDLE_POLL_GAP_US
#  define IDLE_POLL_GAP_US 250
#endif

// صمام أمان مماثل لوضع الخمول: 24 × 250us = 6ms فحص سريع متواصل كحد أقصى
// قبل كل تِك ضمان إجباري — أقل بكثير من مهلة Task Watchdog (~5s).
#ifndef IDLE_POLL_WATCHDOG_GUARD
#  define IDLE_POLL_WATCHDOG_GUARD 24
#endif

#ifndef CC1101_RX_BANDWIDTH
#  define CC1101_RX_BANDWIDTH 812.0f
#endif

#define RECEIVER_BUFFER_SIZE 4

// مدة السماح بانخفاض RSSI داخل الإشارة الواحدة قبل اعتبارها انتهت (FSK).
// حساسات TPMS ترسل حزماً مدتها 10-15ms، وأي هبوط لحظي في RSSI كان يقطعها.
#ifndef GAP_RETENTION_US
#  define GAP_RETENTION_US 3000
#endif

// عرض نطاق الاستقبال الافتراضي: 102 kHz أعطى أفضل مدى في اختبار ميداني
// (فك إشارات حتى -98 dBm مقابل -93 dBm عند 270 kHz).
#ifndef CC1101_FSK_RX_BANDWIDTH
#  define CC1101_FSK_RX_BANDWIDTH 102.0f
#endif

#if OOK_MODULATION
#  ifndef MINIMUM_PULSE_LENGTH
#    define MINIMUM_PULSE_LENGTH 50
#  endif
#  ifndef MINIMUM_SIGNAL_LENGTH
#    define MINIMUM_SIGNAL_LENGTH 40000
#  endif
#else
#  ifndef MINIMUM_PULSE_LENGTH
#    define MINIMUM_PULSE_LENGTH 30
#  endif
#  ifndef MINIMUM_SIGNAL_LENGTH
#    define MINIMUM_SIGNAL_LENGTH 500
#  endif
#endif

#ifdef RF_CC1101
#  define RF_MODULE_RECEIVER_GPIO RF_MODULE_GDO0
#  define STR_MODULE              "CC1101"
  extern SPIClass newSPI;
#endif

// منافذ CC1101
struct rtl_433_ESPPins {
  int8_t sck, miso, mosi, cs, gdo0, gdo2;
};

extern volatile bool rtl_433_radio_config_failed;

#define RADIOLIB_STATE(STATEVAR, FUNCTION)                              \
  {                                                                     \
    if ((STATEVAR) != RADIOLIB_ERR_NONE) {                              \
      logprintfLn(LOG_ERR, STR_MODULE " " FUNCTION " failed, code: %d", \
                  STATEVAR);                                            \
      rtl_433_radio_config_failed = true;                               \
    }                                                                   \
  }

typedef void (*rtl_433_ESPCallBack)(char* message);
typedef void (*rtl_433_raw_pulse_cb)(const int* pulse_us, const int* gap_us,
                                     unsigned int num_pulses,
                                     unsigned long duration_us, int rssi);
typedef struct pulse_data pulse_data_t;

enum class rtl_433_ESPError : uint8_t {
  None = 0,
  OutOfMemory,
  RadioInitialization,
  QueueCreation,
  TaskCreation,
  InvalidArgument,
};

struct rtl_433_ESPStatus {
  unsigned int capturedSignals;
  unsigned int decodedSignals;
  unsigned int decodedMessages;
  unsigned int zeroDecodedSignals;
  unsigned int droppedCaptureBuffers;
  unsigned int droppedDecoderQueue;
  int currentRssi;
  int averageRssi;
  int rssiThreshold;
  bool receiverEnabled;
};

class rtl_433_ESP {
public:
  rtl_433_ESP();
  void loop();
  void setCallback(rtl_433_ESPCallBack callback, char* messageBuffer, int bufferSize);
  void setRawPulsesCallback(rtl_433_raw_pulse_cb callback);
  void setRSSIThreshold(int);

  static void initReceiver(byte inputPin, float receiveFrequency);
  static bool begin(byte inputPin, float receiveFrequency);
  static void end();

  static bool isInitialized();
  static rtl_433_ESPError lastError();
  static rtl_433_ESPStatus statusSnapshot();

  static void enableReceiver();
  static void disableReceiver();
  static void setNotifyTask(TaskHandle_t task);

  // منافذ CC1101 (تُستدعى قبل begin)
  static void setPins(int8_t sck, int8_t miso, int8_t mosi, int8_t cs, int8_t gdo0, int8_t gdo2);
  static rtl_433_ESPPins getPins();

  static int16_t setFrequencyDeviation(float value);
  static float getFrequencyDeviation();
  static bool setFskBandwidth(float khz) { return setRxBandwidth(khz); }  // اسم بديل (نسخة R3)

  // عرض نطاق الاستقبال (kHz) وتردد الاستقبال (MHz) — آمنة من أي مهمة
  static bool setRxBandwidth(float khz);
  static float getRxBandwidth();
  static bool setFrequency(float mhz);
  static float getFrequency();
  static bool isRadioAlive();

  // قفل ناقل SPI بين مهمة قراءة RSSI وأي مهمة أخرى تلمس سجلات الراديو
  static bool spiTake(uint32_t waitMs);
  static void spiGive();

  static void applyToyotaOnlyDecoders();
  static void applyTpmsMask(uint32_t mask);

  // واجهة موحّدة للمفككات (رقم المفكك = رقم البت في القناع)
  static void setDecoderMask(uint32_t mask);          // يُحفظ ويُطبَّق بعد begin أيضاً
  static uint32_t getDecoderMask();
  static int decoderCount() { return getTpmsDecoderCount(); }
  static const char* decoderName(int idx);            // اسم rtl_433 (حقل "protocol" في JSON)
  static const char* decoderShortName(int idx);       // اسم قصير للشاشات (10 أحرف)
  static uint8_t decoderPriority(int idx);            // 0 CRC، 1 Checksum، 2 تحقق ضعيف
  static bool isWeakDecoder(int idx) { return decoderPriority(idx) >= 2; }
  static int protocolIndex(const char* protocolName); // من حقل "protocol" إلى رقم المفكك
  static int activeDecoderCount();
  static float currentFrequency;                       // التردد الحالي (MHz)
  static int  getTpmsDecoderCount();
  static const char* getTpmsDecoderName(int idx);
  static const char* getTpmsDecoderLabel(int idx);
  static bool isTpmsDecoderToyota(int idx);

  static int messageCount;
  // volatile: يُكتب باستمرار من rtl_433_ReceiverTask (مهمة FreeRTOS مستقلة)
  // ويُقرأ من interruptHandler (ISR) ومن statusSnapshot()/loop() في سياقات
  // أخرى — بدون volatile قد يُخزّن المترجم القيمة في سجل ولا يعيد قراءتها
  // من الذاكرة داخل حلقة تلك السياقات، فيرى بعضها قيمة قديمة/ثابتة خطأً.
  static volatile int currentRssi;
  static int signalRssi;
  static int rssiThreshold;
  static int rssiThresholdDelta;
  static int averageRssi;

  static int rtlVerbose;
  static int totalSignals;
  static int ignoredSignals;
  static int unparsedSignals;

  static volatile unsigned int decoderSignals;
  static volatile unsigned int decodedMessages;
  static volatile unsigned int zeroDecodedSignals;
  static volatile unsigned int droppedCaptureBuffers;
  static volatile unsigned int droppedDecoderQueue;

  static bool ookModulation;

private:
  int8_t _outputPin;
  static void interruptHandler();
  static void resetReceiver();
  static int _getRSSI();
  static bool receivePulseTrain(pulse_data_t* destination);

  static bool _enabledReceiver;
  static volatile uint8_t _actualPulseTrain;
  static uint8_t _avaiablePulseTrain;
  static volatile unsigned long _lastChange;
  static volatile int16_t _nrpulses;
  static int16_t _interrupt;
  static volatile TaskHandle_t _notifyTask;
  static float _currentFreqDev;
  static float _currentRxBw;

  static void rtl_433_ReceiverTask(void* pvParameters);
};

#endif