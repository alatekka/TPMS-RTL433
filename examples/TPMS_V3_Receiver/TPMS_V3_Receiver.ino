// =============================================================================
//  TPMS V3 — وحدة الاستقبال (ESP32 + CC1101)
//  راديو مخصص: يفك حزم TPMS ويرسلها لوحدة الشاشة (CYD) عبر UART بروابط محمية
//  برمز تحقق. لا واي فاي هنا عمداً: إبقاء الـ CC1101 بعيداً عن تشويش 2.4GHz.
//
//  البروتوكول (TPMS-LINK v2): كل سطر  $<json>*<XX>\n  حيث XX = XOR لبايتات json (hex)
//    مستقبل ← شاشة:
//      {"t":"P",...}  حزمة حساس: p رقم المفكك، id، k ضغط kPa، c حرارة، b بطارية، s حالة، r RSSI
//      {"t":"S",...}  حالة الراديو كل ثانية
//      {"t":"L","cur":..,"pk":..}  RSSI حي (عند الطلب، كل 150ms)
//      {"t":"D","i":..,"n":..,"s":..,"w":..}  قائمة المفككات (عند الطلب)
//      {"t":"A","ok":1,"c":"..."}  تأكيد أمر
//    شاشة ← مستقبل:
//      {"c":"cfg","bw":102,"foff":30,"d":9,"mask":33554431}   (أي مجموعة من الحقول)
//      {"c":"live","on":1}   {"c":"dec"}   {"c":"ping"}   {"c":"reboot"}
// =============================================================================
#include <Arduino.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <TPMS_RTL433.h>
#include "esp_task_wdt.h"

#define FW_VERSION "V3-RX 3.0"

// منافذ CC1101 (VSPI)
#define CC_SCK   18
#define CC_MISO  19
#define CC_MOSI  23
#define CC_CS    22
#define CC_GDO0  21
#define CC_GDO2  27

// الربط مع وحدة الشاشة
#define LINK_RX  16
#define LINK_TX  17
#define LINK_BAUD 115200
HardwareSerial Link(2);

#define BASE_FREQ_MHZ 433.92f

rtl_433_ESP rf;
char messageBuffer[512];
QueueHandle_t rfQueue = nullptr;
SemaphoreHandle_t linkMutex = nullptr;
Preferences prefs;

struct RawPacket { char text[512]; };

// إعدادات الراديو (NVS)
int   rssiDelta = 9;
float rxBw = 102.0f;
int   freqOffsetKhz = 0;
uint32_t decoderMask = TPMS_ALL_MASK;

volatile bool ccOk = false;
volatile int  ccErr = 0;
volatile uint32_t pktCount = 0, rejCount = 0, queueDrops = 0, ccResets = 0;
volatile bool liveRssi = false;
volatile bool usbLog = false;

float currentFreq() { return BASE_FREQ_MHZ + freqOffsetKhz / 1000.0f; }

// ---------------- الإعدادات الدائمة ----------------
void loadSettings() {
  prefs.begin("v3rx", true);
  rssiDelta     = constrain(prefs.getInt("delta", 9), 3, 30);
  rxBw          = prefs.getFloat("bw", 102.0f);
  freqOffsetKhz = constrain(prefs.getInt("foff", 0), -150, 150);
  decoderMask   = prefs.getUInt("mask", TPMS_ALL_MASK) & TPMS_ALL_MASK;
  prefs.end();
}

void saveSettings() {
  prefs.begin("v3rx", false);
  prefs.putInt("delta", rssiDelta);
  prefs.putFloat("bw", rxBw);
  prefs.putInt("foff", freqOffsetKhz);
  prefs.putUInt("mask", decoderMask);
  prefs.end();
}

// ---------------- إرسال سطر محمي ----------------
void linkSend(JsonDocument& doc) {
  char buf[600];
  size_t n = serializeJson(doc, buf, sizeof(buf) - 8);
  if (n == 0 || n >= sizeof(buf) - 8) return;
  uint8_t x = 0;
  for (size_t i = 0; i < n; i++) x ^= (uint8_t)buf[i];
  if (xSemaphoreTake(linkMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
    Link.write('$');
    Link.write((const uint8_t*)buf, n);
    Link.printf("*%02X\n", x);
    xSemaphoreGive(linkMutex);
  }
  if (usbLog) { Serial.write((const uint8_t*)buf, n); Serial.println(); }
}

void sendAck(const char* cmd) {
  JsonDocument d;
  d["t"] = "A"; d["ok"] = 1; d["c"] = cmd;
  linkSend(d);
}

void sendStatus() {
  rtl_433_ESPStatus st = rtl_433_ESP::statusSnapshot();
  JsonDocument d;
  d["t"] = "S";
  d["fw"] = FW_VERSION;
  d["up"] = millis() / 1000;
  d["cc"] = ccOk ? 1 : 0;
  d["err"] = ccErr;
  d["f"] = serialized(String(currentFreq(), 3));
  d["foff"] = freqOffsetKhz;
  d["bw"] = (int)rxBw;
  d["d"] = rssiDelta;
  d["mask"] = decoderMask;
  d["act"] = rtl_433_ESP::activeDecoderCount();
  d["n"] = st.averageRssi;
  d["th"] = st.rssiThreshold;
  d["cur"] = st.currentRssi;
  d["cap"] = st.capturedSignals;
  d["dec"] = st.decodedMessages;
  d["unk"] = st.zeroDecodedSignals;
  d["lost"] = st.droppedCaptureBuffers + st.droppedDecoderQueue + queueDrops;
  d["pk"] = pktCount;
  d["rej"] = rejCount;
  d["rst"] = ccResets;
  d["heap"] = ESP.getFreeHeap() / 1024;
  linkSend(d);
}

void sendDecoderList() {
  for (int i = 0; i < rtl_433_ESP::decoderCount(); i++) {
    JsonDocument d;
    d["t"] = "D";
    d["i"] = i;
    d["n"] = rtl_433_ESP::decoderName(i);
    d["s"] = rtl_433_ESP::decoderShortName(i);
    d["w"] = rtl_433_ESP::isWeakDecoder(i) ? 1 : 0;
    linkSend(d);
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ---------------- تطبيق إعدادات الراديو ----------------
float nearestBw(float want) {
  static const float bws[] = {58, 68, 81, 102, 116, 135, 162, 203, 232, 270, 325, 406, 464, 541, 650, 812};
  float best = bws[0];
  for (float b : bws) if (fabsf(b - want) < fabsf(best - want)) best = b;
  return best;
}

void applyConfig(JsonDocument& c) {
  bool changed = false;
  if (!c["bw"].isNull())   { rxBw = nearestBw(c["bw"].as<float>()); rtl_433_ESP::setRxBandwidth(rxBw); changed = true; }
  if (!c["foff"].isNull()) { freqOffsetKhz = constrain(c["foff"].as<int>(), -150, 150); rtl_433_ESP::setFrequency(currentFreq()); changed = true; }
  if (!c["d"].isNull())    { rssiDelta = constrain(c["d"].as<int>(), 3, 30); rf.setRSSIThreshold(rssiDelta); changed = true; }
  if (!c["mask"].isNull()) { decoderMask = c["mask"].as<uint32_t>() & TPMS_ALL_MASK; rtl_433_ESP::setDecoderMask(decoderMask); changed = true; }
  if (changed) saveSettings();
}

// ---------------- استقبال الأوامر من الشاشة ----------------
bool parseLinkLine(char* line, JsonDocument& doc) {
  // $<json>*XX
  if (line[0] != '$') return false;
  char* star = strrchr(line, '*');
  if (!star || strlen(star) < 3) return false;
  *star = 0;
  uint8_t x = 0;
  for (char* p = line + 1; *p; p++) x ^= (uint8_t)*p;
  if (x != (uint8_t)strtoul(star + 1, nullptr, 16)) return false;
  return deserializeJson(doc, line + 1) == DeserializationError::Ok;
}

void handleCommand(JsonDocument& c) {
  const char* cmd = c["c"] | "";
  if (!strcmp(cmd, "cfg"))        { applyConfig(c); sendAck(cmd); sendStatus(); }
  else if (!strcmp(cmd, "live"))  { liveRssi = (c["on"] | 0) != 0; sendAck(cmd); }
  else if (!strcmp(cmd, "dec"))   { sendDecoderList(); }
  else if (!strcmp(cmd, "ping"))  { sendStatus(); }
  else if (!strcmp(cmd, "reboot")) { sendAck(cmd); delay(100); ESP.restart(); }
}

// ---------------- معالجة الحزم ----------------
void onPacket(char* message) {
  if (!message || !rfQueue) return;
  RawPacket p;
  strlcpy(p.text, message, sizeof(p.text));
  if (xQueueSend(rfQueue, &p, 0) != pdTRUE) queueDrops++;
}

void processPacket(const char* text) {
  JsonDocument in;
  if (deserializeJson(in, text) != DeserializationError::Ok) return;
  int proto = rtl_433_ESP::protocolIndex(in["protocol"] | "");

  char id[24] = {0};
  if (in["id"].is<const char*>()) strlcpy(id, in["id"].as<const char*>(), sizeof(id));
  else if (in["id"].is<unsigned long>()) snprintf(id, sizeof(id), "%lX", in["id"].as<unsigned long>());
  // توحيد المعرف: HEX كبير، 8 خانات على الأقل
  char norm[16] = {0};
  int n = 0;
  for (const char* q = id; *q && n < 12; q++) if (isxdigit((unsigned char)*q)) norm[n++] = toupper((unsigned char)*q);
  if (n == 0) return;
  char idOut[16];
  if (n < 8) { memset(idOut, '0', 8 - n); strcpy(idOut + 8 - n, norm); } else strcpy(idOut, norm);

  float kpa = in["pressure_kPa"].is<float>() ? in["pressure_kPa"].as<float>()
            : in["pressure_PSI"].is<float>() ? in["pressure_PSI"].as<float>() * 6.894757f : NAN;
  float temp = in["temperature_C"].is<float>() ? in["temperature_C"].as<float>()
             : in["temperature_F"].is<float>() ? (in["temperature_F"].as<float>() - 32.0f) * 5.0f / 9.0f : NAN;

  // فلتر المنطقية للمفككات ضعيفة التحقق
  if (proto >= 0 && rtl_433_ESP::isWeakDecoder(proto)) {
    if ((!isnan(kpa) && (kpa < 30 || kpa > 700)) || (!isnan(temp) && (temp < -30 || temp > 110))) { rejCount++; return; }
  }

  JsonDocument out;
  out["t"] = "P";
  out["p"] = proto;
  out["id"] = idOut;
  if (!isnan(kpa)) out["k"] = serialized(String(kpa, 1));
  if (!isnan(temp)) out["c"] = serialized(String(temp, 0));
  out["b"] = in["battery_ok"].isNull() ? -1 : (in["battery_ok"].as<int>() ? 1 : 0);
  if (!in["status"].isNull()) out["s"] = in["status"].as<int>();
  if ((in["rapid_change"] | 0) || (in["fast_leak"] | 0) || (in["leaking"] | 0)) out["lk"] = 1;
  out["r"] = in["rssi"] | rtl_433_ESP::signalRssi;
  pktCount++;
  linkSend(out);
}

// ---------------- المهام ----------------
void rfTask(void*) {
  esp_task_wdt_add(NULL);
  rf.setCallback(onPacket, messageBuffer, sizeof(messageBuffer));
  bool first = true;
  uint32_t lastAttempt = 0, lastHealth = 0;
  for (;;) {
    esp_task_wdt_reset();
    if (!ccOk) {
      if (first || millis() - lastAttempt >= 3000) {
        lastAttempt = millis();
        if (!first) { rf.end(); ccResets++; }
        first = false;
        rtl_433_ESP::setPins(CC_SCK, CC_MISO, CC_MOSI, CC_CS, CC_GDO0, CC_GDO2);
        // وحدة راديو فقط (لا واي فاي ولا شاشة): مخزن كبير للزحمة مع إبقاء ~90KB حرة
        rtl_433_ESP::setBufferSlots(8, 24);
        rtl_433_ESP::setRxBandwidth(rxBw);
        rf.setRSSIThreshold(rssiDelta);
        rtl_433_ESP::setDecoderMask(decoderMask);
        if (rf.begin(CC_GDO0, currentFreq())) {
          rtl_433_ESP::setNotifyTask(xTaskGetCurrentTaskHandle());
          rf.enableReceiver();
          ccOk = true; ccErr = 0; lastHealth = millis();
          Serial.printf("[RF] CC1101 ready @ %.3f MHz BW %d delta %d\n", currentFreq(), (int)rxBw, rssiDelta);
        } else {
          ccErr = (int)rf.lastError();
          Serial.printf("[RF] init failed (%d)\n", ccErr);
        }
      }
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(50));
    rf.loop();
    if (millis() - lastHealth >= 5000) {
      lastHealth = millis();
      if (!rtl_433_ESP::isRadioAlive()) { ccOk = false; ccErr = -99; Serial.println("[RF] health check failed"); }
    }
  }
}

void packetTask(void*) {
  esp_task_wdt_add(NULL);
  RawPacket p;
  for (;;) {
    esp_task_wdt_reset();
    if (xQueueReceive(rfQueue, &p, pdMS_TO_TICKS(1000)) == pdTRUE) processPacket(p.text);
  }
}

void linkRxTask(void*) {
  esp_task_wdt_add(NULL);
  static char line[400];
  size_t len = 0;
  uint32_t lastStatus = 0, lastLive = 0;
  int peak = -128;
  for (;;) {
    esp_task_wdt_reset();
    while (Link.available()) {
      char ch = (char)Link.read();
      if (ch == '\n' || ch == '\r') {
        if (len) {
          line[len] = 0;
          JsonDocument c;
          if (parseLinkLine(line, c)) handleCommand(c);
          len = 0;
        }
      } else if (len < sizeof(line) - 1) line[len++] = ch;
      else len = 0;
    }
    int cur = rtl_433_ESP::currentRssi;
    if (cur > peak) peak = cur;
    uint32_t now = millis();
    if (liveRssi && now - lastLive >= 150) {
      lastLive = now;
      JsonDocument d;
      d["t"] = "L"; d["cur"] = cur; d["pk"] = peak;
      d["n"] = rtl_433_ESP::averageRssi; d["th"] = rtl_433_ESP::rssiThreshold;
      linkSend(d);
      peak = -128;
    }
    if (now - lastStatus >= 1000) { lastStatus = now; sendStatus(); }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ---------------- أوامر USB للتشخيص ----------------
void handleUsb() {
  static char line[96];
  static size_t len = 0;
  while (Serial.available()) {
    char ch = (char)Serial.read();
    if (ch != '\n' && ch != '\r') { if (len < sizeof(line) - 1) line[len++] = ch; continue; }
    if (!len) continue;
    line[len] = 0; len = 0;
    JsonDocument c;
    if (!strcmp(line, "stat")) {
      rtl_433_ESPStatus st = rtl_433_ESP::statusSnapshot();
      Serial.printf("[STAT] up %lus cc %d | %.3f MHz bw %d delta %d | noise %d thr %d | captured %u decoded %u pk %lu rej %lu lost %lu | heap %u KB\n",
                    millis() / 1000, ccOk, currentFreq(), (int)rxBw, rssiDelta, st.averageRssi, st.rssiThreshold,
                    st.capturedSignals, st.decodedMessages, (unsigned long)pktCount, (unsigned long)rejCount,
                    (unsigned long)(st.droppedCaptureBuffers + st.droppedDecoderQueue + queueDrops), ESP.getFreeHeap() / 1024);
      uint32_t dn = rtl_433_ESP::decodeCount;
      Serial.printf("[PERF] decode avg %.1f ms max %.1f ms | queue peak %lu/%u | slots %u+%u | lost capture %u queue %u\n",
                    dn ? rtl_433_ESP::decodeUsTotal / 1000.0 / dn : 0.0, rtl_433_ESP::decodeUsMax / 1000.0,
                    (unsigned long)rtl_433_ESP::queuePeak, rtl_433_ESP::decodeSlots(), rtl_433_ESP::captureSlots(),
                    rtl_433_ESP::decodeSlots(), rtl_433_ESP::droppedCaptureBuffers, rtl_433_ESP::droppedDecoderQueue);
    } else if (!strncmp(line, "bw ", 3))   { c["bw"] = atof(line + 3);  applyConfig(c); Serial.printf("[CMD] bw %d kHz\n", (int)rxBw); }
    else if (!strncmp(line, "foff ", 5))   { c["foff"] = atoi(line + 5); applyConfig(c); Serial.printf("[CMD] freq %.3f MHz\n", currentFreq()); }
    else if (!strncmp(line, "rssi ", 5))   { c["d"] = atoi(line + 5);   applyConfig(c); Serial.printf("[CMD] rssi delta %d\n", rssiDelta); }
    else if (!strcmp(line, "log on"))      { usbLog = true;  Serial.println("[CMD] log on"); }
    else if (!strcmp(line, "log off"))     { usbLog = false; Serial.println("[CMD] log off"); }
    else if (!strcmp(line, "reboot"))      ESP.restart();
    else Serial.printf("[CMD] unknown: %s (stat | bw N | foff N | rssi N | log on/off | reboot)\n", line);
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.printf("\n[SYSTEM] TPMS %s | TPMS_RTL433 %s\n", FW_VERSION, TPMS_RTL433_VERSION);

  esp_task_wdt_config_t wdt = {};
  wdt.timeout_ms = 15000;
  wdt.trigger_panic = true;
  if (esp_task_wdt_init(&wdt) == ESP_ERR_INVALID_STATE) esp_task_wdt_reconfigure(&wdt);

  loadSettings();
  Link.setRxBufferSize(2048);
  Link.begin(LINK_BAUD, SERIAL_8N1, LINK_RX, LINK_TX);
  linkMutex = xSemaphoreCreateMutex();
  rfQueue = xQueueCreate(24, sizeof(RawPacket));

  xTaskCreatePinnedToCore(rfTask, "rf", 8192, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(packetTask, "pkt", 8192, NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(linkRxTask, "link", 6144, NULL, 2, NULL, 1);
}

void loop() {
  handleUsb();
  vTaskDelay(pdMS_TO_TICKS(20));
}
