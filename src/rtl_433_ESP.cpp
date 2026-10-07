/*
  rtl_433_ESP - 433.92 MHz protocols library for ESP32
  معالجة إشارات الراديو بنظام التخصيص الصفري (Zero Dynamic Allocation)
*/

#include <rtl_433_ESP.h>
#include "esp_task_wdt.h"
#include "capture_utils.h"
#include "signalDecoder.h"

extern "C" unsigned long rtl_433_millis(void) { return millis(); }

#if defined(RF_MODULE_SCK) && defined(RF_MODULE_MISO) && \
    defined(RF_MODULE_MOSI) && defined(RF_MODULE_CS)
#  include <SPI.h>
#  if defined(CONFIG_IDF_TARGET_ESP32C3) || \
      defined(CONFIG_IDF_TARGET_ESP32S3) || \
      defined(CONFIG_IDF_TARGET_ESP32C5)
SPIClass newSPI(FSPI);
#  else
SPIClass newSPI(VSPI);
#  endif
#endif

// كائن الراديو يُنشأ في begin() بالمنافذ المضبوطة (setPins)، فتعمل المكتبة على أي لوحة
static rtl_433_ESPPins _pins = {TPMS_DEFAULT_PIN_SCK, TPMS_DEFAULT_PIN_MISO, TPMS_DEFAULT_PIN_MOSI,
                                TPMS_DEFAULT_PIN_CS, TPMS_DEFAULT_PIN_GDO0, TPMS_DEFAULT_PIN_GDO2};
#ifdef RF_CC1101
static CC1101* radioPtr = nullptr;
#  define radio (*radioPtr)
#endif
static uint32_t _decoderMask = 0x01FFFFFFUL;
volatile bool rtl_433_radio_config_failed = false;

#define rtl_433_ReceiverTask_Stack    2048
#define rtl_433_ReceiverTask_Priority 2
#define rtl_433_ReceiverTask_Core     1   // بعيداً عن الواي فاي (النواة 0) الذي يؤخر مقاطعة GPIO ويشوّه توقيت النبضات

#ifndef PRETRIGGER_ENABLED
#  define PRETRIGGER_ENABLED 1
#endif
#define PRETRIGGER_WINDOW_US 4000
#define PRETRIGGER_PAIRS     32

static bool receiveMode = false;

#if PRETRIGGER_ENABLED
static volatile unsigned int _pretriggerPulse[PRETRIGGER_PAIRS];
static volatile unsigned int _pretriggerGap[PRETRIGGER_PAIRS];
static volatile uint8_t _pretriggerHead = 0;
static volatile uint8_t _pretriggerCount = 0;
static volatile unsigned int _pretriggerPendingPulse = 0;
#endif

static unsigned long signalStart = micros();
static unsigned long gapStart = micros();
static unsigned long signalEnd = micros();

pulse_data_t* _pulseTrains = nullptr;

int rtl_433_ESP::messageCount = 0;
volatile int rtl_433_ESP::currentRssi = 0;
int rtl_433_ESP::signalRssi = 0;
int rtl_433_ESP::rssiThreshold = MINRSSI;
bool rtl_433_ESP::_enabledReceiver = false;
volatile uint8_t rtl_433_ESP::_actualPulseTrain = 0;
uint8_t rtl_433_ESP::_avaiablePulseTrain = 0;
volatile unsigned long rtl_433_ESP::_lastChange = 0;
int rtl_433_ESP::rtlVerbose = 0;
volatile int16_t rtl_433_ESP::_nrpulses = 0;

int rtl_433_ESP::totalSignals = 0;
int rtl_433_ESP::ignoredSignals = 0;
int rtl_433_ESP::unparsedSignals = 0;
volatile unsigned int rtl_433_ESP::decoderSignals = 0;
volatile unsigned int rtl_433_ESP::decodedMessages = 0;
volatile unsigned int rtl_433_ESP::zeroDecodedSignals = 0;
volatile unsigned int rtl_433_ESP::droppedCaptureBuffers = 0;
volatile unsigned int rtl_433_ESP::droppedDecoderQueue = 0;

int rtl_433_ESP::averageRssi = 0;
int rtl_433_ESP::rssiThresholdDelta = RSSI_THRESHOLD;
static int _peakRssi = -256;
static int _aboveThreshold = 0;

bool rtl_433_ESP::ookModulation = OOK_MODULATION;

int _totalRssi = 0;
int _rssiCount = 0;
int _noiseCount = 0;

int16_t rtl_433_ESP::_interrupt = NOT_AN_INTERRUPT;
static int8_t receiverGpio = -1;

volatile TaskHandle_t rtl_433_ESP::_notifyTask = nullptr;
static TaskHandle_t rtl_433_ReceiverHandle = nullptr;
static portMUX_TYPE captureMux = portMUX_INITIALIZER_UNLOCKED;
static bool initialized = false;
static rtl_433_ESPError initializationError = rtl_433_ESPError::None;

float rtl_433_ESP::_currentFreqDev = 40.0f;
float rtl_433_ESP::_currentRxBw = CC1101_FSK_RX_BANDWIDTH;
float rtl_433_ESP::currentFrequency = RF_MODULE_FREQUENCY;

// قفل SPI: مهمة الاستقبال تقرأ RSSI كل ~250us، وصفحة الويب قد تغيّر إعدادات
// الراديو في نفس اللحظة من النواة الأخرى — بدون قفل تتداخل حركات SPI.
static SemaphoreHandle_t _spiMutex = NULL;

bool rtl_433_ESP::spiTake(uint32_t waitMs) {
  if (_spiMutex == NULL) return true;
  TickType_t ticks = (waitMs == UINT32_MAX) ? portMAX_DELAY : pdMS_TO_TICKS(waitMs);
  return xSemaphoreTake(_spiMutex, ticks) == pdTRUE;
}

void rtl_433_ESP::spiGive() {
  if (_spiMutex != NULL) xSemaphoreGive(_spiMutex);
}

#ifdef RF_CC1101
// إعدادات SmartRF Studio لتحسين الحساسية (مدى أبعد) عند عرض نطاق <= 325kHz.
// تُستدعى والراديو في IDLE ومع امتلاك قفل SPI.
static void applySensitivityRegs(float bandwidthKhz) {
  if (bandwidthKhz <= 325.0f) {
    radio.SPIsetRegValue(RADIOLIB_CC1101_REG_TEST2, 0x81);
    radio.SPIsetRegValue(RADIOLIB_CC1101_REG_TEST1, 0x35);
  } else {
    radio.SPIsetRegValue(RADIOLIB_CC1101_REG_TEST2, 0x88);
    radio.SPIsetRegValue(RADIOLIB_CC1101_REG_TEST1, 0x31);
  }
  radio.SPIsetRegValue(RADIOLIB_CC1101_REG_FIFOTHR, 0x40, 6, 6); // ADC_RETENTION
}
#endif

rtl_433_ESP::rtl_433_ESP() {}

void rtl_433_ESP::initReceiver(byte inputPin, float receiveFrequency) {
  begin(inputPin, receiveFrequency);
}

bool rtl_433_ESP::begin(byte inputPin, float receiveFrequency) {
  if (initialized) return true;
  if (inputPin == static_cast<byte>(NOT_AN_INTERRUPT)) {
    initializationError = rtl_433_ESPError::InvalidArgument;
    return false;
  }
  if (!_pulseTrains) {
    _pulseTrains = static_cast<pulse_data_t*>(heap_caps_calloc(
        RECEIVER_BUFFER_SIZE, sizeof(pulse_data_t), MALLOC_CAP_INTERNAL));
  }
  if (!_pulseTrains) {
    initializationError = rtl_433_ESPError::OutOfMemory;
    return false;
  }
  if (_spiMutex == NULL) _spiMutex = xSemaphoreCreateMutex();
  rtl_433_radio_config_failed = false;
  receiverGpio = digitalPinToInterrupt(inputPin);
  currentFrequency = receiveFrequency;

  if (!rtlSetup()) {
    initializationError = rtl_433_ESPError::QueueCreation;
    return false;
  }
  tpmsApplyMask(_decoderMask);
  if (!radioPtr) {
    radioPtr = new CC1101(new Module(_pins.cs, _pins.gdo0, RADIOLIB_NC, _pins.gdo2, newSPI));
  }

  spiTake(UINT32_MAX);
  newSPI.begin(_pins.sck, _pins.miso, _pins.mosi, _pins.cs);

#ifdef RF_CC1101
  int state = radio.begin();
#else
  int state = radio.beginFSK();
#endif
  RADIOLIB_STATE(state, "radio.begin()");

  state = radio.setFrequency(receiveFrequency);
  RADIOLIB_STATE(state, "setFrequency");
  resetReceiver();

  // ضبط الموديول مع معالجة الماكرو الصريحة لتفادي خطأ المترجم
  if (ookModulation) {
    state = radio.setOOK(true);
    RADIOLIB_STATE(state, "setOOK");
  } else {
    state = radio.setOOK(false);
    RADIOLIB_STATE(state, "setFSK");
  }

  state = radio.setCrcFiltering(false);
  RADIOLIB_STATE(state, "setCrcFiltering");

#ifdef RF_CC1101
  if (ookModulation) {
    radio.SPIsendCommand(RADIOLIB_CC1101_CMD_IDLE);
    radio.SPIsetRegValue(RADIOLIB_CC1101_REG_PKTLEN, 0);
    radio.setRxBandwidth(CC1101_RX_BANDWIDTH);
  } else {
    state = radio.setFrequencyDeviation(_currentFreqDev);
    RADIOLIB_STATE(state, "setFrequencyDeviation");
    state = radio.setBitRate(17.24);
    RADIOLIB_STATE(state, "setBitRate");
    state = radio.setRxBandwidth(_currentRxBw);
    RADIOLIB_STATE(state, "setRxBandwidth");
  }
  state = radio.disableSyncWordFiltering(false);
  RADIOLIB_STATE(state, "disableSyncWordFiltering");
  if (!ookModulation) {
    radio.SPIsendCommand(RADIOLIB_CC1101_CMD_IDLE);
    applySensitivityRegs(_currentRxBw);
  }
#endif

  state = radio.receiveDirectAsync();
  RADIOLIB_STATE(state, "receiveDirectAsync");
  spiGive();

  if (!rtl_433_ReceiverHandle) {
    BaseType_t taskCreated = xTaskCreatePinnedToCore(
        rtl_433_ESP::rtl_433_ReceiverTask,
        "rtl_433_ReceiverTask",
        rtl_433_ReceiverTask_Stack,
        NULL,
        rtl_433_ReceiverTask_Priority,
        &rtl_433_ReceiverHandle,
        rtl_433_ReceiverTask_Core);
    if (taskCreated != pdPASS) {
      initializationError = rtl_433_ESPError::TaskCreation;
      rtlShutdown();
      return false;
    }
  }

  if (rtl_433_radio_config_failed) {
    initializationError = rtl_433_ESPError::RadioInitialization;
    end();
    return false;
  }
  initialized = true;
  initializationError = rtl_433_ESPError::None;
  return true;
}

bool rtl_433_ESP::receivePulseTrain(pulse_data_t* destination) {
  if (!destination || !_pulseTrains) return false;
  uint8_t currentTrain = 0;
  bool available = false;

  // إصلاح: يُحجَز فهرس البافر الجاهز فقط داخل القسم الحرج (عملية سريعة
  // جداً، بضع مقارنات وزيادة عداد)، بدل حجز القسم الحرج طوال مدة نسخ/مسح
  // بنية pulse_data_t الكاملة كما كان سابقاً. تعطيل كل المقاطعات على
  // Core 0 (وهذا ما يفعله portENTER_CRITICAL) طوال مدة memcpy/memset لبنية
  // كبيرة نسبياً (قد تصل لعشرات الميكروثواني) كان يعني: لو بدأت إشارة
  // TPMS ثانية بالوصول بالضبط في تلك اللحظة (سيناريو متوقع تماماً عند
  // إرسال عدة حساسات إطارات متقاربة زمنياً)، فإن مقاطعة استقبالها
  // (interruptHandler) تتأخر أو تُفقد، فيُفسَد توقيت أول نبضاتها بدقة
  // الميكروثانية ويفشل فك تشفيرها. إخراج النسخ خارج القسم الحرج يحل هذا.
  //
  // هذا آمن دون أي شرط إضافي: بمجرد تقديم _avaiablePulseTrain هنا داخل
  // القسم الحرج، لا يمكن لمهمة الاستقبال (الكاتب) الكتابة فوق هذا البافر
  // تحديداً إلا بعد إكمال RECEIVER_BUFFER_SIZE-1 إشارات كاملة أخرى بالكامل
  // (كل واحدة تستغرق ميلي ثواني على الأقل) — أي أبطأ بمراحل من زمن هذا
  // النسخ (ميكروثواني فقط)، فلا خطر تضارب حقيقي على محتوى البافر نفسه.
  portENTER_CRITICAL(&captureMux);
  if (_pulseTrains[_avaiablePulseTrain].num_pulses > 0) {
    currentTrain = _avaiablePulseTrain;
    _avaiablePulseTrain = (_avaiablePulseTrain + 1) % RECEIVER_BUFFER_SIZE;
    available = true;
  }
  portEXIT_CRITICAL(&captureMux);

  if (available) {
    memcpy(destination, &_pulseTrains[currentTrain], sizeof(pulse_data_t));
    memset(&_pulseTrains[currentTrain], 0, sizeof(pulse_data_t));
  }

  return available;
}

void ICACHE_RAM_ATTR rtl_433_ESP::interruptHandler() {
  if (!_enabledReceiver) {
    _noiseCount++;
    return;
  }

  const unsigned long now = micros();
  const unsigned int duration = now - _lastChange;

  if (!receiveMode) {
#if PRETRIGGER_ENABLED
    bool level = digitalRead(receiverGpio);
    if (_lastChange == 0 || duration > PRETRIGGER_WINDOW_US) {
      _pretriggerCount = 0;
      _pretriggerPendingPulse = 0;
    }
    if (!level) {
      _pretriggerPendingPulse =
          duration > MINIMUM_PULSE_LENGTH && duration <= PRETRIGGER_WINDOW_US
              ? duration : 0;
    } else if (_pretriggerPendingPulse > 0 &&
               duration > MINIMUM_PULSE_LENGTH &&
               duration <= PRETRIGGER_WINDOW_US) {
      _pretriggerPulse[_pretriggerHead] = _pretriggerPendingPulse;
      _pretriggerGap[_pretriggerHead] = duration;
      _pretriggerHead = (_pretriggerHead + 1) % PRETRIGGER_PAIRS;
      if (_pretriggerCount < PRETRIGGER_PAIRS) _pretriggerCount++;
      _pretriggerPendingPulse = 0;
    }
    _lastChange = now;
#endif
    _noiseCount++;
    return;
  }

  volatile pulse_data_t& pulseTrain = _pulseTrains[_actualPulseTrain];
  volatile int* pulse = pulseTrain.pulse;
  volatile int* gap = pulseTrain.gap;

  // كل حافة أثناء الالتقاط تُسجَّل: شرط RSSI القديم هنا كان يُسقط حواف
  // الإشارات الضعيفة القريبة من العتبة (وهي بالضبط إشارات المدى البعيد).
  if (duration > MINIMUM_PULSE_LENGTH) {
    if (_nrpulses >= PD_MAX_PULSES - 1) { // البفر ممتلئ: لا التفاف فوق بداية الإشارة
      _lastChange = now;
      return;
    }
    if (!digitalRead(receiverGpio)) {
      pulse[_nrpulses] = duration;
    } else {
      if (pulse[_nrpulses] > 0) {
        gap[_nrpulses] = duration;
        _nrpulses++;
      } else if (_nrpulses > 1) {
        gap[_nrpulses - 1] += duration;
      } else {
        gap[_nrpulses] = duration;
        _nrpulses++;
      }
    }
    _lastChange = now;
  }
}

void rtl_433_ESP::resetReceiver() {
  if (!_pulseTrains) return;
  portENTER_CRITICAL(&captureMux);
  for (unsigned int i = 0; i < RECEIVER_BUFFER_SIZE; i++) {
    _pulseTrains[i].num_pulses = 0;
  }
  _avaiablePulseTrain = 0;
  _actualPulseTrain = 0;
  _nrpulses = 0;
  portEXIT_CRITICAL(&captureMux);
#if PRETRIGGER_ENABLED
  _pretriggerHead = 0;
  _pretriggerCount = 0;
  _pretriggerPendingPulse = 0;
#endif
  _totalRssi = 0;
  _rssiCount = 0;
  _peakRssi = -256;
  _aboveThreshold = 0;
  receiveMode = false;
  signalStart = micros();
}

void rtl_433_ESP::enableReceiver() {
  if (receiverGpio >= 0) {
    pinMode(receiverGpio, INPUT);
    attachInterrupt((uint8_t)receiverGpio, interruptHandler, CHANGE);
    _enabledReceiver = true;
  }
}

void rtl_433_ESP::disableReceiver() {
  _enabledReceiver = false;
  if (receiverGpio >= 0) {
    detachInterrupt((uint8_t)receiverGpio);
  }
}

void rtl_433_ESP::setNotifyTask(TaskHandle_t task) {
  _notifyTask = task;
}

void rtl_433_ESP::end() {
  disableReceiver();
  if (rtl_433_ReceiverHandle) {
    spiTake(UINT32_MAX);
    vTaskDelete(rtl_433_ReceiverHandle);
    rtl_433_ReceiverHandle = nullptr;
    spiGive();
  }
  rtlShutdown();
  if (_pulseTrains) {
    free(_pulseTrains);
    _pulseTrains = nullptr;
  }
  initialized = false;
  receiverGpio = -1;
}

bool rtl_433_ESP::isInitialized() { return initialized; }
rtl_433_ESPError rtl_433_ESP::lastError() { return initializationError; }

rtl_433_ESPStatus rtl_433_ESP::statusSnapshot() {
  rtl_433_ESPStatus snapshot{};
  portENTER_CRITICAL(&captureMux);
  snapshot.capturedSignals = totalSignals;
  snapshot.decodedSignals = decoderSignals;
  snapshot.decodedMessages = decodedMessages;
  snapshot.zeroDecodedSignals = zeroDecodedSignals;
  snapshot.droppedCaptureBuffers = droppedCaptureBuffers;
  snapshot.droppedDecoderQueue = droppedDecoderQueue;
  snapshot.currentRssi = currentRssi;
  snapshot.averageRssi = averageRssi;
  snapshot.rssiThreshold = rssiThreshold;
  snapshot.receiverEnabled = _enabledReceiver;
  portEXIT_CRITICAL(&captureMux);
  return snapshot;
}

/*----------------- حلقة المعالجة الخالية تماماً من تخصيص الذاكرة الديناميكي -----------------*/
void rtl_433_ESP::loop() {
  // ملاحظة أداء مهمة (بيئة مزدحمة بإشارات كثيرة/سيارات متعددة):
  // النسخة الأصلية كانت تسحب إشارة واحدة فقط من حلقة الالتقاط
  // (_pulseTrains، سعتها RECEIVER_BUFFER_SIZE) ثم تنام إجبارياً
  // vTaskDelay(1) بغض النظر عن وجود إشارات أخرى جاهزة بالفعل في الحلقة.
  // في بيئة فيها إشارات متلاحقة من عدة حساسات، هذا يعني أن معدل التفريغ
  // كان محدوداً بإشارة واحدة تقريباً لكل tick، فتمتلئ الحلقة الدائرية
  // ويبدأ إسقاط إشارات (droppedCaptureBuffers) قبل أن تصل حتى لطابور فك
  // التشفير. الآن نُفرّغ كل الإشارات الجاهزة دفعة واحدة في كل استدعاء
  // (محدودة طبيعياً بحجم RECEIVER_BUFFER_SIZE، عملية سريعة جداً: نسخ
  // ذاكرة + دفع لطابور، بدون أي عملية حظر/انتظار)، ولا ننام إلا إذا لم
  // يكن هناك أي عمل فعلي لتنفيذه، مما يقلل زمن الانتقال من الالتقاط
  // إلى فك التشفير إلى أدنى حد ممكن.
  if (_enabledReceiver && _pulseTrains) {
    while (_pulseTrains[_avaiablePulseTrain].num_pulses > 0) {
      // حجز فوري من المستودع الساكن بدون Heap Churn
      pulse_data_t* rtl_pulses = acquirePulseBuffer();
      if (!rtl_pulses) {
        // مستودع فك التشفير ممتلئ فعلاً (كل بافراته قيد الاستخدام) —
        // لا فائدة من محاولة المزيد الآن، نتوقف وننتظر الدورة القادمة
        droppedDecoderQueue++;
        break;
      }
      if (!receivePulseTrain(rtl_pulses)) {
        releasePulseBuffer(rtl_pulses);
        break;
      }
      if (rtl_pulses->num_pulses > PD_MIN_PULSES) {
        processSignal(rtl_pulses);
      } else {
        ignoredSignals++;
        releasePulseBuffer(rtl_pulses);
      }
    }
  }
  // لا يوجد أي نوم/تأخير هنا: عندما لا توجد بيانات جاهزة، تخرج الدالة فورًا
  // والمهمة المستدعية (rfLoopTask) تعود لتنام بالكامل على ulTaskNotifyTake(...,
  // portMAX_DELAY) بدون استهلاك أي دورة معالج، بدل النوم المصطنع القديم.
}

void rtl_433_ESP::rtl_433_ReceiverTask(void* pvParameters) {
  // إصلاح أمان: تسجيل مهمة الاستقبال (الأهم زمنياً في كامل النظام) في
  // Task Watchdog النظام. هذه المهمة لا تنتظر أبداً بلا مهلة (تستخدم
  // delayMicroseconds/vTaskDelay(1) الدورية فقط)، فإطعام الـWatchdog دورياً آمن
  // تماماً ولا يخاطر بإنذار كاذب.
  esp_task_wdt_add(NULL);
  // إصلاح كفاءة مهم: كان إطعام الـWatchdog (esp_task_wdt_reset) يُستدعى في
  // كل تكرار لهذه الحلقة — وهي أضيق وأكثر حلقة حساسة توقيتياً في كامل
  // النظام، تعمل حتى 50 ألف مرة/ثانية أثناء تتبع نهاية إشارة فعلية
  // (REALTIME_POLL_GAP_US=20us). esp_task_wdt_reset() تتطلب داخلياً في
  // ESP-IDF أخذ قفل لتحديث بيانات المهمة في جدول الـWatchdog بأمان — استدعاؤها
  // بهذا المعدل الهائل يُدخل تأخيراً/تنافساً غير ضروري إطلاقاً في المسار
  // الأكثر حساسية لدقة التوقيت (اكتشاف بداية/نهاية إشارة TPMS). بما أن مهلة
  // الـWatchdog كاملة 10 ثوانٍ (APP_WDT_TIMEOUT_S)، فإطعامه مرة كل ثانية
  // كافٍ بهامش أمان ضخم (10×) دون أي خطر إنذار كاذب، ويُلغي هذا التنافس تماماً
  // في المسار الساخن.
  uint32_t lastWdtFeedMs = 0;
  for (;;) {
    uint32_t nowMs = millis();
    if (nowMs - lastWdtFeedMs >= 1000) {
      esp_task_wdt_reset();
      lastWdtFeedMs = nowMs;
    }
    if (_enabledReceiver) {
      currentRssi = _getRSSI();
      _rssiCount++;
      _totalRssi += currentRssi;
      if (currentRssi > _peakRssi) _peakRssi = currentRssi;
      if (currentRssi > rssiThreshold) _aboveThreshold++;

      if (_rssiCount >= RSSI_SAMPLES) {
        averageRssi = _totalRssi / _rssiCount;
        rssiThreshold = averageRssi + rssiThresholdDelta;
        _peakRssi = -256;
        _aboveThreshold = 0;
        _totalRssi = 0;
        _rssiCount = 0;
      }

      if (currentRssi > rssiThreshold) {
        if (!receiveMode) {
          receiveMode = true;
          signalStart = micros();
          signalRssi = currentRssi;

#if PRETRIGGER_ENABLED
          noInterrupts();
          uint8_t selected = 0;
          unsigned int pretriggerDuration = 0;
          for (uint8_t i = 0; i < _pretriggerCount; ++i) {
            uint8_t index = (_pretriggerHead + PRETRIGGER_PAIRS - 1 - i) % PRETRIGGER_PAIRS;
            unsigned int pairDuration = _pretriggerPulse[index] + _pretriggerGap[index];
            if (pretriggerDuration + pairDuration > PRETRIGGER_WINDOW_US) break;
            pretriggerDuration += pairDuration;
            selected++;
          }
          uint8_t first = (_pretriggerHead + PRETRIGGER_PAIRS - selected) % PRETRIGGER_PAIRS;
          for (uint8_t i = 0; i < selected; ++i) {
            uint8_t index = (first + i) % PRETRIGGER_PAIRS;
            _pulseTrains[_actualPulseTrain].pulse[i] = _pretriggerPulse[index];
            _pulseTrains[_actualPulseTrain].gap[i] = _pretriggerGap[index];
          }
          _nrpulses = selected;
          if (!digitalRead(receiverGpio) && _pretriggerPendingPulse > 0 && _nrpulses < PD_MAX_PULSES) {
            _pulseTrains[_actualPulseTrain].pulse[_nrpulses] = _pretriggerPendingPulse;
            pretriggerDuration += _pretriggerPendingPulse;
          }
          _pretriggerCount = 0;
          _pretriggerPendingPulse = 0;
          interrupts();
          signalStart -= pretriggerDuration;
#else
          _lastChange = signalStart;
#endif
        }
        signalEnd = micros();
      } else if (receiveMode && (micros() - signalEnd < GAP_RETENTION_US)) {
        // هبوط لحظي في RSSI داخل الإشارة: نستمر في الالتقاط
      } else {
        if (receiveMode) {
          receiveMode = false;
          totalSignals++;
          if (rtl_433_capture::isCompleteSignal(_nrpulses, signalEnd - signalStart, PD_MIN_PULSES, MINIMUM_SIGNAL_LENGTH)) {
            uint8_t nextTrain = (_actualPulseTrain + 1) % RECEIVER_BUFFER_SIZE;
            portENTER_CRITICAL(&captureMux);
            if (!rtl_433_capture::canPublishToNextBuffer(_pulseTrains[nextTrain].num_pulses)) {
              // (حافة نادرة) البافر التالي ممتلئ فعلاً: نسقط الإشارة، لكن
              // بدون "continue" — نكمل بشكل طبيعي لمنطق التأخير المتكيف في
              // نهاية الحلقة بدل تخطيه بالكامل، حتى لا يتحول استمرار
              // امتلاء البافر لحلقة busy-loop بلا أي صمام أمان.
              memset(&_pulseTrains[_actualPulseTrain], 0, sizeof(pulse_data_t));
              droppedCaptureBuffers++;
              portEXIT_CRITICAL(&captureMux);
              _nrpulses = 0;
            } else {
              _pulseTrains[_actualPulseTrain].num_pulses = _nrpulses + 1;
              _pulseTrains[_actualPulseTrain].signalDuration = signalEnd - signalStart;
              _pulseTrains[_actualPulseTrain].signalRssi = signalRssi;
              messageCount++;
              gapStart = micros();
              _actualPulseTrain = nextTrain;
              portEXIT_CRITICAL(&captureMux);
              _nrpulses = 0;
              if (_notifyTask) {
                xTaskNotifyGive(_notifyTask);
              }
            }
          } else {
            ignoredSignals++;
            _nrpulses = 0;
          }
        }
      }
    }

    // ------------------- جدولة تأخير متكيفة حسب الحالة -------------------
    // أثناء تتبع نهاية إشارة فعلية (receiveMode == true) دقة توقيت النهاية
    // هي اللي فعلاً بتأثر على جودة فصل الإشارات المتلاحقة، فهنا نفحص RSSI
    // بأقصى معدل ممكن (فجوة ميكروثانية بسيطة فقط بدل تِك كامل 1ms) بدون أي
    // حظر حقيقي للمهمة. أما في وضع الانتظار (لا توجد إشارة قيد الالتقاط)
    // فلا داعي إطلاقاً للسرعة القصوى، فنرجع للتأخير الآمن التقليدي تِك واحد
    // (1ms) الذي يعطي فرصة كاملة لمهمة الخمول ولباقي مهام Core 0.
    //
    // صمام أمان إجباري: حتى لو ظل receiveMode == true لفترة طويلة جداً بشكل
    // غير طبيعي (مثلاً ضجيج RF مستمر يبقي RSSI فوق العتبة باستمرار)، فبعد
    // كل REALTIME_POLL_WATCHDOG_GUARD دورة فحص سريعة، نجبر المهمة على تنفيذ
    // vTaskDelay(1) حقيقي مرة واحدة على أي حال. هذا يضمن أن مهمة الخمول على
    // Core 0 لا يمكن تجويعها أبداً لدرجة إطلاق Task Watchdog Timer (فشل
    // قاتل/إعادة إقلاع)، بغض النظر عن أي سيناريو حافة.
    static uint16_t realtimePollGuard = 0;
    static uint16_t idlePollGuard = 0;
    if (receiveMode) {
      idlePollGuard = 0;
      if (++realtimePollGuard >= REALTIME_POLL_WATCHDOG_GUARD) {
        realtimePollGuard = 0;
        vTaskDelay(1);
      } else {
        delayMicroseconds(REALTIME_POLL_GAP_US);
      }
    } else {
      // فحص RSSI بمعدل أعلى أيضاً أثناء الخمول (بدل تِك كامل 1ms) لزيادة
      // احتمال اكتشاف إشارات TPMS القصيرة جداً من أول إرسال بدل انتظار
      // إعادة إرسال لاحقة. نفس منطق صمام الأمان لضمان عدم تجويع مهمة
      // الخمول على Core 0.
      realtimePollGuard = 0;
      if (++idlePollGuard >= IDLE_POLL_WATCHDOG_GUARD) {
        idlePollGuard = 0;
        vTaskDelay(1);
      } else {
        delayMicroseconds(IDLE_POLL_GAP_US);
      }
    }
  }
}

rtl_433_ESPCallBack _callback;
char* _messageBuffer;
int _bufferSize;

void rtl_433_ESP::setCallback(rtl_433_ESPCallBack callback, char* messageBuffer, int bufferSize) {
  if (!callback || !messageBuffer || bufferSize <= 0) {
    initializationError = rtl_433_ESPError::InvalidArgument;
    return;
  }
  _callback = callback;
  _messageBuffer = messageBuffer;
  _bufferSize = bufferSize;
  _setCallback(callback, messageBuffer, bufferSize);
}

void rtl_433_ESP::setRawPulsesCallback(rtl_433_raw_pulse_cb callback) {
  _setRawPulsesCallback(callback);
}

void rtl_433_ESP::setRSSIThreshold(int newRssi) {
  rssiThresholdDelta = newRssi;
}

int rtl_433_ESP::_getRSSI(void) {
  if (!spiTake(5)) return currentRssi; // الناقل مشغول لحظياً: آخر قيمة
  int r = radio.getRSSI();
  spiGive();
  return r;
}

int16_t rtl_433_ESP::setFrequencyDeviation(float value) {
  if (value < 5.0f || value > 150.0f) {
    return RADIOLIB_ERR_INVALID_FREQUENCY_DEVIATION;
  }
  _currentFreqDev = value;
#ifdef RF_CC1101
  if (!initialized) return RADIOLIB_ERR_NONE;
  if (!spiTake(500)) return RADIOLIB_ERR_SPI_CMD_TIMEOUT;
  radio.SPIsendCommand(RADIOLIB_CC1101_CMD_IDLE);
  int16_t state = radio.setFrequencyDeviation(value);
  radio.receiveDirectAsync();
  spiGive();
  return state;
#else
  return radio.setFrequencyDeviation(value);
#endif
}

float rtl_433_ESP::getFrequencyDeviation() {
  return _currentFreqDev;
}

bool rtl_433_ESP::setRxBandwidth(float khz) {
  _currentRxBw = khz;
#ifdef RF_CC1101
  if (!initialized || ookModulation) return true;
  if (!spiTake(500)) return false;
  radio.SPIsendCommand(RADIOLIB_CC1101_CMD_IDLE);
  int16_t state = radio.setRxBandwidth(khz);
  applySensitivityRegs(khz);
  radio.receiveDirectAsync();
  spiGive();
  return state == RADIOLIB_ERR_NONE;
#else
  return true;
#endif
}

float rtl_433_ESP::getRxBandwidth() { return _currentRxBw; }

bool rtl_433_ESP::setFrequency(float mhz) {
  currentFrequency = mhz;
#ifdef RF_CC1101
  if (!initialized) return true;
  if (!spiTake(500)) return false;
  radio.SPIsendCommand(RADIOLIB_CC1101_CMD_IDLE);
  int16_t state = radio.setFrequency(mhz);
  radio.receiveDirectAsync();
  spiGive();
  _totalRssi = 0; // أرضية الضجيج تتغير مع التردد: إعادة حساب المتوسط
  _rssiCount = 0;
  return state == RADIOLIB_ERR_NONE;
#else
  return true;
#endif
}

float rtl_433_ESP::getFrequency() { return currentFrequency; }

void rtl_433_ESP::setPins(int8_t sck, int8_t miso, int8_t mosi, int8_t cs, int8_t gdo0, int8_t gdo2) {
  _pins = {sck, miso, mosi, cs, gdo0, gdo2};
}

rtl_433_ESPPins rtl_433_ESP::getPins() { return _pins; }

void rtl_433_ESP::setDecoderMask(uint32_t mask) {
  _decoderMask = mask;
  tpmsApplyMask(mask);   // بلا أثر قبل begin، ويُعاد تطبيقه داخل begin
}

uint32_t rtl_433_ESP::getDecoderMask() { return _decoderMask; }
const char* rtl_433_ESP::decoderName(int idx) { return tpmsGetDeviceName(idx); }
const char* rtl_433_ESP::decoderShortName(int idx) { return tpmsGetDecoderShortName(idx); }
uint8_t rtl_433_ESP::decoderPriority(int idx) { return tpmsGetDecoderPriority(idx); }
int rtl_433_ESP::protocolIndex(const char* protocolName) { return tpmsProtocolIndex(protocolName); }
int rtl_433_ESP::activeDecoderCount() { return __builtin_popcount(tpmsGetEnabledMask()); }

#ifndef RADIOLIB_CC1101_REG_VERSION
#  define RADIOLIB_CC1101_REG_VERSION 0x31
#endif
#ifndef RADIOLIB_CC1101_REG_MARCSTATE
#  define RADIOLIB_CC1101_REG_MARCSTATE 0x35
#endif

// فحص صحة الراديو: الشريحة تجيب (VERSION=0x14) وهي في وضع الاستقبال.
// إن خرجت من RX (تشويش/خطأ) نحاول إعادتها قبل إعلان الفشل.
bool rtl_433_ESP::isRadioAlive() {
#ifdef RF_CC1101
  if (!initialized || !radioPtr) return false;
  if (!spiTake(100)) return true;
  bool alive = radio.SPIgetRegValue(RADIOLIB_CC1101_REG_VERSION) == 0x14;
  if (alive) {
    int ms = radio.SPIgetRegValue(RADIOLIB_CC1101_REG_MARCSTATE) & 0x1F;
    if (ms != 0x0D) {
      radio.SPIsendCommand(RADIOLIB_CC1101_CMD_IDLE);
      radio.SPIsendCommand(RADIOLIB_CC1101_CMD_RX);
      delayMicroseconds(300);
      ms = radio.SPIgetRegValue(RADIOLIB_CC1101_REG_MARCSTATE) & 0x1F;
      alive = (ms == 0x0D);
    }
  }
  spiGive();
  return alive;
#else
  return true;
#endif
}

int rtl_433_ESP::getTpmsDecoderCount() { return tpmsGetDecoderCount(); }
const char* rtl_433_ESP::getTpmsDecoderName(int idx) { return tpmsGetDecoderName(idx); }
const char* rtl_433_ESP::getTpmsDecoderLabel(int idx) { return tpmsGetDecoderLabel(idx); }
bool rtl_433_ESP::isTpmsDecoderToyota(int idx) { return tpmsIsDecoderToyota(idx); }
void rtl_433_ESP::applyToyotaOnlyDecoders() { tpmsApplyToyotaOnly(); }
void rtl_433_ESP::applyTpmsMask(uint32_t mask) { tpmsApplyMask(mask); }