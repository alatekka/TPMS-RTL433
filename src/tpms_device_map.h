#ifndef TPMS_DEVICE_MAP_H
#define TPMS_DEVICE_MAP_H

// خريطة مفككات TPMS الموحّدة (الترتيب يحدده جدول g_tpmsProtocols في signalDecoder.cpp)
// رقم المفكك = رقم البت في قناع البروتوكولات.
#include "rtl_433_ESP.h"

#define TPMS_NUM_DECODERS 25
#define TOYOTA_ONLY_MASK  ((1UL << 0) | (1UL << 1))   // Toyota PMV-C210 + PMV-107J
#define TPMS_ALL_MASK     ((1UL << TPMS_NUM_DECODERS) - 1UL)

static inline bool tpmsIsToyota(int idx) { return idx == 0 || idx == 1; }
static inline bool tpmsIsWeakDecoder(int idx) { return rtl_433_ESP::isWeakDecoder(idx); }
static inline const char* tpmsShortName(int idx) { return rtl_433_ESP::decoderShortName(idx); }

#endif
