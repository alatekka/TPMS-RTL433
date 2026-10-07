/*
  rtl_433_ESP - 433.92 MHz protocols library for ESP32
  مكتبة فك تشفير إشارات TPMS المحسنة بنظام الذاكرة الساكنة المسبقة
*/

#ifndef rtl_433_DECODER_H
#define rtl_433_DECODER_H

#include "rtl_433_ESP.h"

extern "C" {
#include "bitbuffer.h"
#include "fatal.h"
#include "list.h"
#include "pulse_analyzer.h"
#include "pulse_detect.h"
#include "r_api.h"
#include "r_private.h"
#include "rtl_433.h"
#include "rtl_433_devices.h"
}

#include "log.h"
#include "tools/aprintf.h"

/*----------------------------- إدارة مستودع النبضات الساكن -----------------------------*/
pulse_data_t* acquirePulseBuffer();
void releasePulseBuffer(pulse_data_t* buf);

/*----------------------------- وظائف النظام والمهام -----------------------------*/
bool rtlSetup();
void rtlShutdown();
void _setCallback(rtl_433_ESPCallBack callback, char* messageBuffer, int bufferSize);
void _setRawPulsesCallback(rtl_433_raw_pulse_cb callback);
void _setDebug(int debug);
void processSignal(pulse_data_t* rtl_pulses);
void rtl_433_DecoderTask(void* pvParameters);
extern TaskHandle_t rtl_433_DecoderHandle;

/*----------------------------- إدارة ديكودرات TPMS الحصرية -----------------------------*/
int         tpmsGetDecoderCount();
const char* tpmsGetDecoderName(int idx);
const char* tpmsGetDecoderLabel(int idx);
const char* tpmsGetDecoderShortName(int idx);
const char* tpmsGetDeviceName(int idx);
uint8_t tpmsGetDecoderPriority(int idx);
int tpmsProtocolIndex(const char* protocolName);
bool        tpmsIsDecoderToyota(int idx);
bool        tpmsIsDecoderEnabled(int idx);
void        tpmsSetDecoderEnabled(int idx, bool enabled);
void        tpmsApplyToyotaOnly();
void        tpmsApplyMask(uint32_t mask);
uint32_t    tpmsGetEnabledMask();

#endif