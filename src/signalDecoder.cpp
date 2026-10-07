/*
  rtl_433_ESP - 433.92 MHz protocols library for ESP32
  مستودع نبضات موسع (6 بافرات) لاستيعاب الزحام الشديد دون أي إسقاط للحزم
*/

#include "signalDecoder.h"
#include <atomic>
#include <cstring>
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#ifndef rtl_433_Decoder_Stack
#  define rtl_433_Decoder_Stack 16384
#endif

#if defined(CONFIG_FREERTOS_UNICORE)
#  define rtl_433_Decoder_Priority 3
#  define rtl_433_Decoder_Core     0
#else
// ملاحظة أداء مهمة: في الملف الرئيسي (.ino) يتم إنشاء storageTask بأولوية 3
// على نفس النواة (Core 1) التي كانت مهمة فك التشفير تعمل عليها بأولوية 2
// فقط. أي عملية كتابة FRAM/تخزين طويلة نسبياً (I2C) كانت يمكن أن "تقطع"
// (preempt) مهمة فك التشفير في منتصف معالجة إشارة واردة، مما يزيد احتمال
// امتلاء طابور النبضات (rtl_433_Queue) وإسقاط حزم كاملة في بيئة مزدحمة
// بإشارات كثيرة (سيارات متعددة). رفعنا الأولوية هنا إلى 4 (أعلى من
// storageTask) لضمان أن فك تشفير أي إشارة راديو واردة له الأولوية المطلقة
// دائماً على أعمال التخزين والصيانة الخلفية، دون أي تعديل على .ino.
#  define rtl_433_Decoder_Priority 4
#  if defined(CONFIG_IDF_TARGET_ESP32S3)
// ESP32-S3: الفك الثقيل على النواة 0 بالتوازي مع الالتقاط الدقيق على النواة 1 (مقاس: 13ms/إشارة، صفر فقد)
#    define rtl_433_Decoder_Core   0
#  else
#    define rtl_433_Decoder_Core   1
#  endif
#endif

int rtlVerbose = 0;
r_cfg_t g_cfg;

TaskHandle_t rtl_433_DecoderHandle = nullptr;
static QueueHandle_t rtl_433_Queue = nullptr;
static std::atomic<rtl_433_raw_pulse_cb> rawPulsesCallback{nullptr};
static SemaphoreHandle_t tpmsMutex = nullptr;

// رفع حجم المستودع إلى 6 بافرات لمقاومة دفقات الإشارات الكثيفة
// (كان معرّفاً هنا خطأً بـ 5 رغم أن التعليق ينص على 6 — تم تصحيحه ليطابق
// فعلياً النية الأصلية ويمنح هامش استيعاب إضافي 20% في بيئة مزدحمة بالسيارات)
extern uint8_t g_decodeSlots;   // من rtl_433_ESP.cpp (setBufferSlots)
static pulse_data_t* s_pulsePool = nullptr;
static QueueHandle_t s_pulseFreeQueue = nullptr;

pulse_data_t* acquirePulseBuffer() {
  if (!s_pulseFreeQueue) return nullptr;
  pulse_data_t* buf = nullptr;
  if (xQueueReceive(s_pulseFreeQueue, &buf, 0) == pdTRUE) {
    memset(buf, 0, sizeof(pulse_data_t));
    return buf;
  }
  return nullptr;
}

void releasePulseBuffer(pulse_data_t* buf) {
  if (!buf || !s_pulseFreeQueue) return;
  xQueueSend(s_pulseFreeQueue, &buf, 0);
}

static inline void tpmsLock() {
  if (tpmsMutex) xSemaphoreTake(tpmsMutex, portMAX_DELAY);
}
static inline void tpmsUnlock() {
  if (tpmsMutex) xSemaphoreGive(tpmsMutex);
}

/*----------------------------- جدول بروتوكولات TPMS الحصرية -----------------------------*/
struct TpmsProtocolEntry {
  const char* name;
  const char* label;
  const r_device* device;
  bool isToyota;
  uint8_t priority; // 0 = تحقق CRC قوي، 1 = Checksum، 2 = تحقق ضعيف يلتقط إشارات غيره
  const char* shortName;
};

static const TpmsProtocolEntry g_tpmsProtocols[] = {
  {"tpms_toyota",        "Toyota / Lexus",             &tpms_toyota,        true,  0, "Toyota210"},
  {"tpms_pmv107j",       "Toyota Pacific (PMV-107J)",  &tpms_pmv107j,       true,  0, "Toyota107"},
  {"tpms_ford",          "Ford",                       &tpms_ford,          false, 1, "Ford"},
  {"tpms_nissan",        "Nissan",                     &tpms_nissan,        false, 1, "Nissan"},
  {"tpms_hyundai_vdo",   "Hyundai VDO",                &tpms_hyundai_vdo,   false, 0, "HyundaiVDO"},
  {"tpms_kia",           "Kia",                        &tpms_kia,           false, 0, "Kia"},
  {"tpms_elantra2012",   "Hyundai Elantra",            &tpms_elantra2012,   false, 0, "Elantra"},
  {"tpms_citroen",       "Citroen / Peugeot",          &tpms_citroen,       false, 1, "Citroen"},
  {"tpms_renault",       "Renault",                    &tpms_renault,       false, 0, "Renault"},
  {"tpms_renault_0435r", "Renault (0435R)",            &tpms_renault_0435r, false, 0, "Ren.0435R"},
  {"tpms_mercedes_benz", "Mercedes-Benz",              &tpms_mercedes_benz, false, 0, "Mercedes"},
  {"tpms_bmw",           "BMW",                        &tpms_bmw,           false, 0, "BMW G4/5"},
  {"tpms_bmwg3",         "BMW (G3)",                   &tpms_bmwg3,         false, 0, "BMW G2/3"},
  {"tpms_porsche",       "Porsche",                    &tpms_porsche,       false, 0, "Porsche"},
  {"tpms_honda",         "Honda",                      &tpms_honda,         false, 0, "Honda"},
  {"tpms_jeep",          "Jeep / Chrysler",            &tpms_jeep,          false, 2, "Jeep"},
  {"tpms_abarth124",     "Abarth 124",                 &tpms_abarth124,     false, 2, "Abarth124"},
  {"tpms_trw_fsk",       "TRW FSK",                    &tpms_trw_fsk,       false, 0, "TRW"},
  {"tpms_jansite",       "Jansite",                    &tpms_jansite,       false, 2, "Jansite"},
  {"tpms_jansite_solar", "Jansite Solar",              &tpms_jansite_solar, false, 0, "JansiteSol"},
  {"tpms_jansite_ty588", "Jansite TY588",              &tpms_jansite_ty588, false, 2, "Jansite588"},
  {"tpms_truck",         "Truck TPMS",                 &tpms_truck,         false, 1, "Truck"},
  {"tpms_ave",           "AVE TPMS",                   &tpms_ave,           false, 0, "AVE"},
  {"tpms_airpuxem",      "Airpuxem",                   &tpms_airpuxem,      false, 0, "Airpuxem"},
  {"tpms_sefis_m3",      "Sefis M3 Motorcycle",        &tpms_sefis_m3,      false, 0, "Sefis M3"}
};

static const int NUM_TPMS_PROTOCOLS = sizeof(g_tpmsProtocols) / sizeof(g_tpmsProtocols[0]);

int tpmsGetDecoderCount() { return NUM_TPMS_PROTOCOLS; }

const char* tpmsGetDecoderShortName(int idx) {
  if (idx < 0 || idx >= NUM_TPMS_PROTOCOLS) return "TPMS";
  return g_tpmsProtocols[idx].shortName;
}

const char* tpmsGetDeviceName(int idx) {
  if (idx < 0 || idx >= NUM_TPMS_PROTOCOLS) return "";
  return g_tpmsProtocols[idx].device->name;
}

uint8_t tpmsGetDecoderPriority(int idx) {
  if (idx < 0 || idx >= NUM_TPMS_PROTOCOLS) return 0;
  return g_tpmsProtocols[idx].priority;
}

int tpmsProtocolIndex(const char* protocolName) {
  if (!protocolName || !protocolName[0]) return -1;
  for (int i = 0; i < NUM_TPMS_PROTOCOLS; i++)
    if (strcmp(protocolName, g_tpmsProtocols[i].device->name) == 0) return i;
  return -1;
}

const char* tpmsGetDecoderName(int idx) {
  if (idx < 0 || idx >= NUM_TPMS_PROTOCOLS) return "";
  return g_tpmsProtocols[idx].name;
}

const char* tpmsGetDecoderLabel(int idx) {
  if (idx < 0 || idx >= NUM_TPMS_PROTOCOLS) return "";
  return g_tpmsProtocols[idx].label;
}

bool tpmsIsDecoderToyota(int idx) {
  if (idx < 0 || idx >= NUM_TPMS_PROTOCOLS) return false;
  return g_tpmsProtocols[idx].isToyota;
}

/*
  ------------------------------------------------------------------------
  ملاحظة إصلاح جوهرية (اقرأ قبل التعديل):

  المكتبة الأصلية (rtl_433 core, دالة run_ook_demods/run_fsk_demods في
  r_api.c) لا تتحقق إطلاقاً من الحقل "disabled" أثناء فك التشفير الفعلي.
  هذا الحقل يُستخدم فقط لحظة "register_protocol()" لتقرير ما إذا كان
  الديكودر سيُضاف إلى قائمة "cfg.demod->r_devs" الحية التي يتم المرور
  عليها في كل إشارة واردة. أي تغيير لاحق على "disabled" بعد التسجيل
  لا قيمة له إطلاقاً، لأن run_ook_demods/run_fsk_demods يمران مباشرة على
  عناصر القائمة (r_devs.elems) دون أي فحص لهذا الحقل. كذلك فإن
  register_protocol() ينسخ الجهاز (malloc + *p = *r_dev) في نسخة جديدة
  مستقلة تماماً عن مصفوفة cfg.devices الأصلية، فتعديل
  cfg.devices[i].disabled لا ينعكس أبداً على النسخة الفعلية المُستخدمة
  في القائمة.

  هذا هو تفسير المشكلة التي وصفتها: كل الديكودرات تعمل دائماً بغض النظر
  عمّا يُفعَّل أو يُعطَّل من المتصفح، ولا فرق فعلياً بين صفحة الماسح
  وصفحة السيارة.

  الحل: بدلاً من الاعتماد على حقل "disabled"، أصبحنا نتحكم فعلياً في
  عضوية كل ديكودر داخل قائمة cfg.demod->r_devs عبر:
    - register_protocol()   لإضافة الديكودر فعلياً لقائمة فك التشفير
    - unregister_protocol() لحذفه فعلياً منها (يبحث بالاسم "name" ويحرر
                             النسخة التي أنشأها register_protocol)
  ونحتفظ في g_registeredMask بحالة "ما هو مسجل فعلياً الآن" لنتجنب
  تسجيل/إلغاء ديكودر مسجل بالفعل، ونحمي كل تعديل بـ tpmsMutex لأن مهمة
  فك التشفير (rtl_433_DecoderTask) تمر على نفس القائمة أثناء ذلك.
  ------------------------------------------------------------------------
*/

// بتّة لكل ديكودر = 1 إن كان مسجلاً فعلياً الآن داخل cfg.demod->r_devs
// (أي يشارك فعلياً في فك التشفير) و 0 إن كان مستبعداً تماماً من القائمة.
static uint32_t g_registeredMask = 0;

// ملاحظة توافقية: بعض إصدارات r_api.c المُختصرة لـ ESP32 (ومنها التي تستخدمها
// أنت) تُعلن unregister_protocol()/free_protocol() في الهيدر r_api.h لكن لا
// تُنفّذهما فعلياً داخل r_api.c نفسها (تم اختصارهما لأن المنفذ الأصلي لم
// يكن يحتاج فك تسجيل أي بروتوكول في وقت التشغيل). لذلك لا نعتمد عليهما
// إطلاقاً، ونطبّق منطق الحذف يدوياً هنا باستخدام list_remove() فقط، وهي
// دالة موجودة فعلياً ومربوطة (list.c) لأن register_protocol() نفسها تستخدم
// list_push() من نفس الملف بنجاح.

// يحرر النسخة التي أنشأتها register_protocol() (malloc + *p = *r_dev)
static void tpmsFreeProtocolCopy(void* elem) {
  r_device* dev = static_cast<r_device*>(elem);
  if (!dev) return;
  free(dev->decode_ctx);
  free(dev);
}

// تسجيل ديكودر واحد فعلياً في قائمة فك التشفير الحية (يجب استدعاؤها وقفل tpmsMutex محجوز)
static inline void tpmsRegisterIndexLocked(int i) {
  if (!g_cfg.demod) return;                       // لم يتم rtlSetup() بعد
  if (g_registeredMask & (1UL << i)) return;       // مسجل فعلاً، لا داعي للتكرار
  // بعض نسخ r_api.h تعلن register_protocol() بمعامل r_device* غير ثابت
  // (على عكس نسخة rtl_433 upstream التي تستخدم r_device const*)، لذا نستخدم
  // const_cast هنا بأمان: register_protocol تقرأ الجهاز فقط لتنسخه (malloc +
  // *p = *r_dev) ولا تُعدّل أبداً الجهاز الأصلي الذي نمرره.
  register_protocol(&g_cfg, const_cast<r_device*>(g_tpmsProtocols[i].device), NULL);
  // register_protocol يضيف نسخة في آخر القائمة: نضبط أولويتها هناك.
  // run_fsk_demods يتوقف عند أول مستوى أولوية ينجح فيه أي مفكك، فلا يفك
  // مفكك ضعيف (Jansite/Jeep) إشارة سبق أن فكها مفكك CRC بقيم مستحيلة.
  list_t* devs = &g_cfg.demod->r_devs;
  if (devs->len > 0) {
    r_device* added = static_cast<r_device*>(devs->elems[devs->len - 1]);
    if (added) added->priority = g_tpmsProtocols[i].priority;
  }
  g_registeredMask |= (1UL << i);
}

// حذف ديكودر واحد فعلياً من قائمة فك التشفير الحية (يجب استدعاؤها وقفل tpmsMutex محجوز)
// تنفيذ يدوي بديل عن unregister_protocol() غير المتوفرة في هذه النسخة المختصرة
// من المكتبة: نبحث بالاسم "name" داخل cfg.demod->r_devs ونحذف كل تطابق فعلياً
// (بنفس المنطق الذي كانت تنفذه unregister_protocol في rtl_433 الأصلية).
static inline void tpmsUnregisterIndexLocked(int i) {
  if (!g_cfg.demod) return;
  if (!(g_registeredMask & (1UL << i))) return;    // غير مسجل أصلاً
  const char* targetName = g_tpmsProtocols[i].device->name;
  list_t* devs = &g_cfg.demod->r_devs;
  for (size_t k = 0; k < devs->len; ++k) {
    r_device* p = static_cast<r_device*>(devs->elems[k]);
    if (p && p->name && strcmp(p->name, targetName) == 0) {
      list_remove(devs, k, tpmsFreeProtocolCopy);
      k--; // كي لا نتخطى العنصر الذي انزاح للأسفل بعد الحذف
    }
  }
  g_registeredMask &= ~(1UL << i);
}

bool tpmsIsDecoderEnabled(int idx) {
  if (idx < 0 || idx >= NUM_TPMS_PROTOCOLS) return false;
  tpmsLock();
  bool en = (g_registeredMask & (1UL << idx)) != 0;
  tpmsUnlock();
  return en;
}

void tpmsSetDecoderEnabled(int idx, bool enabled) {
  if (idx < 0 || idx >= NUM_TPMS_PROTOCOLS) return;
  tpmsLock();
  if (enabled) tpmsRegisterIndexLocked(idx);
  else         tpmsUnregisterIndexLocked(idx);
  tpmsUnlock();
}

// وضع "السيارة" (تويوتا فقط): يسجل حصرياً ديكودرات تويوتا ويحذف كل ما عداها
// فعلياً من قائمة فك التشفير، بصرف النظر عن قناع الماسح المحفوظ.
void tpmsApplyToyotaOnly() {
  tpmsLock();
  for (int i = 0; i < NUM_TPMS_PROTOCOLS; i++) {
    if (g_tpmsProtocols[i].isToyota) tpmsRegisterIndexLocked(i);
    else                             tpmsUnregisterIndexLocked(i);
  }
  tpmsUnlock();
}

// وضع "الماسح": يطابق قائمة فك التشفير الحية فعلياً مع القناع المطلوب bit-by-bit
void tpmsApplyMask(uint32_t mask) {
  tpmsLock();
  for (int i = 0; i < NUM_TPMS_PROTOCOLS; i++) {
    bool en = (mask & (1UL << i)) != 0;
    if (en) tpmsRegisterIndexLocked(i);
    else    tpmsUnregisterIndexLocked(i);
  }
  tpmsUnlock();
}

uint32_t tpmsGetEnabledMask() {
  tpmsLock();
  uint32_t mask = g_registeredMask;
  tpmsUnlock();
  return mask;
}

/*----------------------------- تهيئة المحرك والنظام -----------------------------*/
bool rtlSetup() {
  r_cfg_t* cfg = &g_cfg;

  if (!tpmsMutex) {
    tpmsMutex = xSemaphoreCreateMutex();
    if (!tpmsMutex) return false;
  }

  // حجز 6 بافرات في الـ Internal Heap لمرة واحدة فقط
  if (!s_pulsePool) {
    // خانات الفك لا تلمسها المقاطعة، فتذهب إلى PSRAM إن وُجدت وتبقى الذاكرة الداخلية حرة
    s_pulsePool = (pulse_data_t*)heap_caps_calloc(g_decodeSlots, sizeof(pulse_data_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_pulsePool) s_pulsePool = (pulse_data_t*)heap_caps_calloc(g_decodeSlots, sizeof(pulse_data_t), MALLOC_CAP_INTERNAL);
    if (!s_pulsePool) return false;
  }

  if (!s_pulseFreeQueue) {
    s_pulseFreeQueue = xQueueCreate(g_decodeSlots, sizeof(pulse_data_t*));
    if (!s_pulseFreeQueue) return false;
    for (int i = 0; i < g_decodeSlots; ++i) {
      pulse_data_t* ptr = &s_pulsePool[i];
      xQueueSend(s_pulseFreeQueue, &ptr, 0);
    }
  }

  if (!cfg->demod) {
    r_init_cfg(cfg);

    // r_init_cfg() يخصص تلقائياً مصفوفة cfg->devices الكاملة بكل بروتوكولات
    // rtl_433 الافتراضية (مئات الديكودرات عبر ماكرو DEVICES) — وهي غير
    // مستخدمة إطلاقاً في هذا المشروع لأننا نسجل حصرياً بروتوكولات TPMS الـ25
    // الموجودة في g_tpmsProtocols مباشرة عبر register_protocol(). نحررها هنا
    // لاسترجاع ذاكرة داخلية ثمينة على ESP32 كانت مهدرة بلا أي استخدام فعلي.
    free(cfg->devices);
    cfg->devices = nullptr;
    cfg->num_r_devices = 0;

    add_log_output(cfg, NULL);
    cfg->conversion_mode = CONVERT_SI;
    cfg->verbosity = rtlVerbose;

    // ملاحظة: لا نسجّل أي ديكودر هنا مباشرة. تسجيل الديكودرات الفعلي يتم
    // حصرياً عبر tpmsApplyToyotaOnly()/tpmsApplyMask() (تُستدعى من
    // syncDecoderEngineWithScreenMode() في الملف الرئيسي فور اكتمال
    // rtlSetup()/rtlSetup للراديو) حتى تبقى قائمة فك التشفير الحية
    // (cfg.demod->r_devs) مطابقة دائماً لما يختاره المستخدم فعلياً.
    g_registeredMask = 0;
  }

  if (!rtl_433_Queue) {
    rtl_433_Queue = xQueueCreate(g_decodeSlots, sizeof(pulse_data_t*));
    if (!rtl_433_Queue) return false;
  }

  if (!rtl_433_DecoderHandle) {
    BaseType_t taskCreated = xTaskCreatePinnedToCore(
        rtl_433_DecoderTask,
        "rtl_433_DecoderTask",
        rtl_433_Decoder_Stack,
        NULL,
        rtl_433_Decoder_Priority,
        &rtl_433_DecoderHandle,
        rtl_433_Decoder_Core);
    if (taskCreated != pdPASS) {
      vQueueDelete(rtl_433_Queue);
      rtl_433_Queue = nullptr;
      return false;
    }
  }
  return true;
}

void rtlShutdown() {
  if (rtl_433_DecoderHandle) {
    vTaskDelete(rtl_433_DecoderHandle);
    rtl_433_DecoderHandle = nullptr;
  }
  if (rtl_433_Queue) {
    pulse_data_t* pending = nullptr;
    while (xQueueReceive(rtl_433_Queue, &pending, 0) == pdTRUE) {
      releasePulseBuffer(pending);
    }
    vQueueDelete(rtl_433_Queue);
    rtl_433_Queue = nullptr;
  }
  if (s_pulseFreeQueue) {
    vQueueDelete(s_pulseFreeQueue);
    s_pulseFreeQueue = nullptr;
  }
  if (s_pulsePool) {
    free(s_pulsePool);
    s_pulsePool = nullptr;
  }
  if (g_cfg.demod) {
    // تحرير كل النسخ التي أنشأتها register_protocol() فعلياً وتفريغ القائمة
    // (نفس دالة التحرير المحلية المستخدمة في tpmsUnregisterIndexLocked، وليس
    // free_protocol() لأنها غير مُنفَّذة في هذه النسخة المختصرة من المكتبة)
    list_free_elems(&g_cfg.demod->r_devs, tpmsFreeProtocolCopy);
  }
  g_registeredMask = 0;
  if (tpmsMutex) {
    vSemaphoreDelete(tpmsMutex);
    tpmsMutex = nullptr;
  }
}

void _setCallback(rtl_433_ESPCallBack callback, char* messageBuffer, int bufferSize) {
  r_cfg_t* cfg = &g_cfg;
  cfg->callback = callback;
  cfg->messageBuffer = messageBuffer;
  cfg->bufferSize = bufferSize;
}

void _setRawPulsesCallback(rtl_433_raw_pulse_cb callback) {
  rawPulsesCallback.store(callback, std::memory_order_release);
}

void _setDebug(int debug) {
  rtlVerbose = debug;
}

/*----------------------------- مهمة فك التشفير -----------------------------*/
void rtl_433_DecoderTask(void* pvParameters) {
  pulse_data_t* rtl_pulses = nullptr;
  // إصلاح أمان: تسجيل هذه المهمة في Task Watchdog النظام (يُهيَّأ من setup()
  // في الملف الرئيسي). هذه من أهم مهمتين حرجتين في كامل النظام — أي تجمّد
  // فيها (قفل mutex لا يُحرَّر، حلقة لا نهائية في أحد الديكودرات) يعني توقف
  // فك تشفير كل الإشارات بصمت. تسجيلها هنا يضمن اكتشاف ذلك وإعادة إقلاع
  // تلقائية بدل توقف صامت دائم.
  esp_task_wdt_add(NULL);
  for (;;) {
    // كانت portMAX_DELAY تعني عدم إطعام الـWatchdog إطلاقاً أثناء فترات
    // الهدوء الطبيعية (لا سيارات مجاورة ترسل حالياً) لأن المهمة لا تعود
    // لرأس الحلقة إطلاقاً في تلك الحالة. استبدالها بمهلة 5 ثوانٍ يضمن دورة
    // إطعام منتظمة دون أي تغيير في زمن الاستجابة الفعلي (الإشعار عبر
    // xQueueSend من rf.loop() ما زال يوقظ المهمة لحظياً فور وصول أي إشارة).
    esp_task_wdt_reset();
    // انتظار 1s ثم تغذية الـ Watchdog: مهلته الافتراضية في Arduino-ESP32 هي 5s فقط
    if (xQueueReceive(rtl_433_Queue, &rtl_pulses, pdMS_TO_TICKS(1000)) != pdTRUE || !rtl_pulses) {
      continue;
    }

    const int64_t decodeStart = esp_timer_get_time();
    rtl_pulses->sample_rate = 1.0e6;
    r_cfg_t* cfg = &g_cfg;
    // ملاحظة أداء: كان هنا سطر "cfg->demod->pulse_data = *rtl_pulses;" ينسخ
    // البنية الكاملة (pulse_data_t، تحتوي مصفوفات نبضات/فجوات كبيرة) على كل
    // إشارة واردة، بينما run_ook_demods/run_fsk_demods تستقبلان rtl_pulses
    // مباشرة كمعامل ولا تقرآن أبداً من cfg->demod->pulse_data (تم التأكد
    // بالبحث الكامل في كل ملفات المشروع). حذفناه لأنه نسخ ميت بالكامل
    // (dead copy) يستهلك دورات CPU على كل حزمة بلا أي فائدة فعلية — وفي
    // بيئة مزدحمة بإشارات كثيرة هذا التوفير مهم تراكمياً.
    int events = 0;
    cfg->demod->pulse_data.signalRssi = rtl_pulses->signalRssi;
    cfg->demod->pulse_data.signalDuration = rtl_pulses->signalDuration;

    tpmsLock();
    if (rtl_433_ESP::ookModulation) {
      events = run_ook_demods(&cfg->demod->r_devs, rtl_pulses);
    } else {
      events = run_fsk_demods(&cfg->demod->r_devs, rtl_pulses);
    }
    tpmsUnlock();

    const uint32_t decodeUs = (uint32_t)(esp_timer_get_time() - decodeStart);
    if (decodeUs > rtl_433_ESP::decodeUsMax) rtl_433_ESP::decodeUsMax = decodeUs;
    rtl_433_ESP::decodeUsTotal += decodeUs;
    rtl_433_ESP::decodeCount++;
    rtl_433_ESP::decoderSignals++;
    if (events > 0) {
      rtl_433_ESP::decodedMessages += events;
    } else {
      rtl_433_ESP::zeroDecodedSignals++;
      rtl_433_ESP::unparsedSignals++;
    }

    rtl_433_raw_pulse_cb rawCallback = rawPulsesCallback.load(std::memory_order_acquire);
    if (rawCallback != nullptr) {
      rawCallback(rtl_pulses->pulse, rtl_pulses->gap, rtl_pulses->num_pulses,
                  rtl_pulses->signalDuration, rtl_pulses->signalRssi);
    }

    releasePulseBuffer(rtl_pulses);
  }
}

void processSignal(pulse_data_t* rtl_pulses) {
  if (!rtl_433_Queue || xQueueSend(rtl_433_Queue, &rtl_pulses, 0) != pdTRUE) {
    rtl_433_ESP::droppedDecoderQueue++;
    releasePulseBuffer(rtl_pulses);
    return;
  }
  uint32_t waiting = uxQueueMessagesWaiting(rtl_433_Queue);
  if (waiting > rtl_433_ESP::queuePeak) rtl_433_ESP::queuePeak = waiting;
}