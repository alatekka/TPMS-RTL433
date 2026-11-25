# TPMS_RTL433 — نظام مراقبة ضغط الإطارات الاحترافي

[![Build](https://github.com/alatekka/TPMS-RTL433/actions/workflows/build.yml/badge.svg)](https://github.com/alatekka/TPMS-RTL433/actions/workflows/build.yml)
[![License: GPL-3.0](https://img.shields.io/badge/license-GPL--3.0-blue.svg)](LICENSE)
[![alatekka.com](https://img.shields.io/badge/project-alatekka.com-0ea5e9)](https://alatekka.com/project.html?p=tpms)

**[صفحة المشروع / Project page](https://alatekka.com/project.html?p=tpms)** · [English](#english) · © على تكّة (Alatekka)

مكتبة موحّدة وبرامج كاملة لاستقبال حساسات ضغط الإطارات (TPMS) على تردد **433.92 MHz** بشريحة **CC1101**، تعمل على **ESP32** و**ESP32-S3**. كل ما يلزم للبناء موجود في هذا المستودع.

## الأجهزة

| | R3 | DEV | V3 (وحدتان) |
|---|---|---|---|
| المعالج | ESP32-S3 N16R8 (PSRAM 8MB) | ESP32-WROOM-32 DevKit 38 منفذ | ESP32 للراديو + CYD ESP32-2432S028 للشاشة |
| الشاشة | ST7789 320×240 أفقية | ST7789 240×320 عمودية | ST7789 2.8" لمس، LovyanGFX |
| الذاكرة الدائمة | NVS | FRAM 32KB وساعة DS3231 | NVS |
| واجهة الويب | صفحة تفاعلية واحدة (JSON) | صفحات متعددة مع CSRF | في وحدة الشاشة، بعيداً عن الراديو |
| البرنامج | [`examples/TPMS_R3_ESP32S3`](examples/TPMS_R3_ESP32S3) | [`examples/TPMS_DEV_ESP32`](examples/TPMS_DEV_ESP32) | [`TPMS_V3_Receiver`](examples/TPMS_V3_Receiver) + [`TPMS_V3_CYD_Display`](examples/TPMS_V3_CYD_Display) |
| الدليل | [docs/R3_ESP32S3_GUIDE.md](docs/R3_ESP32S3_GUIDE.md) | [docs/DEV_ESP32_GUIDE.md](docs/DEV_ESP32_GUIDE.md) | [docs/V3_GUIDE.md](docs/V3_GUIDE.md) |

## أبرز الميزات
- **25 بروتوكول TPMS بتعديل FSK:** تويوتا، وهيونداي/كيا، وفورد، ورينو، وسيتروين، وBMW، ومرسيدس، وغيرها.
- **أولويات فك:** مفككات CRC تعمل أولاً، فلا تُفك الإشارة الواحدة ببروتوكولين. ومعها فلتر يرفض القيم المستحيلة.
- **مدى بعيد:**
  - عرض نطاق 102 kHz، وسجلات تحسين الحساسية في CC1101، ومعايرة انحراف الكريستال.
  - **النتيجة في الاختبار الميداني:** فك إشارات حتى **-99 dBm**.
- **الاستقرار:**
  - قفل لناقل SPI، وسماح بفجوات قصيرة داخل الحزمة، وفحص صحة الراديو مع إعادة تشغيله تلقائياً.
  - حماية Watchdog، و**صفر حزم مفقودة** في اختبارات استمرت ساعات.
- **الإنذارات:**
  - ضغط منخفض أو مرتفع، وحرارة، وبطارية، وتسريب سريع، وانقطاع الإشارة.
  - صوت عبر طنان وضوء عبر لمبة اللوحة في نسخة DEV.
- **التعلّم الذاتي** في نسخة DEV:
  - اكتشاف حساسات السيارة، وكشف تبديل إطار أو حساس.
  - تعلّم الضغط الطبيعي لكل إطار.
  - معايرة تردد الراديو ذاتياً.
- **الحساسات الأصلية:** محفوظة في الذاكرة، مع زر استعادة.
- **وضع فني الإطارات:** يلتقط الحساس الأقرب ويحكم على حالته، مع تقرير قابل للطباعة وتصدير CSV.
- **الضغط البارد عند 20°C،** ومنحنيات الضغط مع تقدير معدل التسريب البطيء.
- **ميزات إضافية:** ملفات لعدة سيارات، واختيار الوحدات (bar أو PSI أو kPa، و°C أو °F)، والتحديث اللاسلكي OTA مع رجوع تلقائي.

## بنية المستودع

| المسار | المحتوى |
|---|---|
| `library.properties` | تعريف مكتبة TPMS_RTL433 |
| `src/` | المكتبة: محرك الاستقبال، ومفككات TPMS الـ 25، ومكتبة rtl_433 |
| `include/` | ترويسات rtl_433 |
| `examples/TPMS_R3_ESP32S3/` | برنامج جهاز R3 |
| `examples/TPMS_DEV_ESP32/` | برنامج جهاز DEV، مع ملف `partitions.csv` |
| `examples/TPMS_V3_Receiver/` | النسخة 3: وحدة الراديو (CC1101 فقط) |
| `examples/TPMS_V3_CYD_Display/` | النسخة 3: وحدة الشاشة اللمسية والواي فاي |
| `docs/` | الأدلة: البناء، وكل جهاز، وإضافة جهاز جديد |
| `tools/` | سكربتات البناء والرفع والمنفذ التسلسلي والتحليل |
| `.github/workflows/` | بناء تلقائي لكل النسخ مع كل تغيير |

## البدء السريع

```bash
tools/build.sh all
```

لبناء جهاز ورفعه مباشرة:

```bash
tools/build.sh dev COM5
```

التفاصيل في [docs/BUILD.md](docs/BUILD.md). ولإضافة جهاز جديد، راجع [docs/ADDING_A_DEVICE.md](docs/ADDING_A_DEVICE.md).

## المصادر والترخيص
- **مبني على:** [rtl_433_ESP](https://github.com/NorthernMan54/rtl_433_ESP) من NorthernMan54، و[rtl_433](https://github.com/merbanan/rtl_433) من merbanan ومساهميه.
- **الترخيص:** GPL-3.0، كترخيص المكتبة الأصلية (ملف [LICENSE](LICENSE)).

## الحقوق
© 2026 **على تكّة (Alatekka)** — [alatekka.com](https://alatekka.com). حقوق كل ما أُضيف أو عُدّل في هذا المستودع محفوظة لـ«على تكّة»: برامج الأجهزة R3 وDEV وV3، وتعديلات المكتبة، والأدلة، والأدوات.

الكود منشور برخصة GPL-3.0: يحق لأي شخص استخدامه وتعديله وإعادة نشره، بشرط أن تبقى إشارات الحقوق كما هي، وأن يُنشر أي تعديل بنفس الرخصة مع كوده المصدري. حقوق الملفات الأصلية من rtl_433 وrtl_433_ESP لأصحابها كما هو مكتوب في رأس كل ملف. التفاصيل في ملف [NOTICE](NOTICE).

---

## English

**TPMS_RTL433** is a unified Arduino library and complete firmware for receiving tire-pressure (TPMS) sensors at **433.92 MHz** with a **CC1101** radio on **ESP32 / ESP32-S3**.

- **25 FSK TPMS protocols** (Toyota, Hyundai/Kia, Ford, Renault, Citroën, BMW, Mercedes and more), CRC decoders first, with a filter that rejects impossible values.
- **Long range:** 102 kHz bandwidth, CC1101 sensitivity registers and crystal-offset calibration; decodes down to about **-99 dBm** in field tests.
- **Stable:** SPI locking, gap retention, radio health checks with automatic restart, watchdog, zero lost packets in multi-hour tests.
- **Alarms:** low / high pressure, temperature, battery, fast leak and lost signal.
- **Self-learning (DEV):** finds the car's sensors, detects a swapped tire or sensor, learns each tire's normal pressure.
- **Tire-technician mode:** reads the nearest sensor and judges it, with a printable report and CSV export.
- **Three builds:** **R3** (ESP32-S3 + ST7789), **DEV** (ESP32 + FRAM + DS3231) and **V3** (ESP32 radio unit + CYD ESP32-2432S028 touch display with a WiFi web page).

Build everything with `tools/build.sh all` — see [docs/BUILD.md](docs/BUILD.md) (the guides are in Arabic).

### Copyright and license
© 2026 **Alatekka (على تكّة)** — [alatekka.com](https://alatekka.com). Copyright in all additions and changes in this repository (the R3, DEV and V3 firmware, library changes, documentation and tools) belongs to Alatekka.

Released under the **GNU GPL v3.0** ([LICENSE](LICENSE)), the same license as the upstream code: anyone may use, modify and redistribute it, as long as the copyright notices stay and any modified version is released under the same license with its source. Based on [rtl_433_ESP](https://github.com/NorthernMan54/rtl_433_ESP) by NorthernMan54 and [rtl_433](https://github.com/merbanan/rtl_433) by merbanan and contributors; their files keep their original copyright headers. See [NOTICE](NOTICE).
