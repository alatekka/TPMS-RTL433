# دليل البناء

## المتطلبات

| العنصر | الإصدار المختبر |
|---|---|
| حزمة لوحات ESP32 (Espressif) | 4.0.0-alpha1 |
| RadioLib | 7.8.1 |
| ArduinoJson | 7.4.3 |
| Adafruit GFX Library | 1.12.6 |
| Adafruit ST7735 and ST7789 Library | 1.11.0 |
| RTClib (لنسخة DEV فقط) | 2.1.4 |

تثبيت المكتبات بـ arduino-cli:

```bash
arduino-cli lib install RadioLib ArduinoJson "Adafruit GFX Library" "Adafruit ST7735 and ST7789 Library" RTClib
```

## الطريقة 1: سكربت المستودع (موصى بها)

```bash
tools/build.sh all
```

| الأمر | الوظيفة |
|---|---|
| `tools/build.sh all` | بناء النسختين |
| `tools/build.sh r3 COM4` | بناء نسخة R3 ورفعها |
| `tools/build.sh dev COM5` | بناء نسخة DEV ورفعها |

السكربت يمرر جذر المستودع كمكتبة (`--library`)، فلا حاجة لنسخ أي شيء إلى مجلد المكتبات.

## الطريقة 2: Arduino IDE
1. انسخ مجلد المستودع كاملاً إلى `Documents/Arduino/libraries/TPMS_RTL433`.
2. افتح من القائمة: File ← Examples ← TPMS_RTL433 ← `TPMS_R3_ESP32S3` أو `TPMS_DEV_ESP32`.
3. اختر إعدادات اللوحة:

| الإعداد | R3 (ESP32-S3) | DEV (ESP32) |
|---|---|---|
| Board | ESP32S3 Dev Module | ESP32 Dev Module |
| Flash Size | 16MB | 4MB |
| PSRAM | OPI PSRAM | — |
| Partition Scheme | 16M Flash (3MB APP/9.9MB FATFS) | Minimal SPIFFS (1.9MB APP with OTA)، ويُستخدم ملف `partitions.csv` المرفق تلقائياً |

## ملاحظات
- **مدة البناء الأول:** قد تصل إلى 10–15 دقيقة على Windows، والبناءات التالية أسرع بكثير.
- **المنافذ:** تُضبط في كل برنامج عبر `rtl_433_ESP::setPins()` قبل `begin()`. القيم الافتراضية حسب الشريحة موجودة في `src/rtl_433_ESP.h`.
- **التعريفات:**
  - لوحة DEV تستخدم شريحة CP2102، وتعريفها CP210x Universal Windows Driver من Silicon Labs.
  - لوحة R3 تستخدم شريحة CH343.
