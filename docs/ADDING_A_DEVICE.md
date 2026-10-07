# إضافة جهاز أو إصدار جديد (مثل الإصدار 3)

المكتبة مستقلة عن اللوحة، فكل ما يخص جهازاً بعينه موجود داخل برنامجه في `examples/`.

## الخطوات
1. أنشئ مجلداً جديداً، مثل `examples/TPMS_V3_<اسم>/`، وفيه ملف `.ino` بنفس الاسم.
2. ابدأ البرنامج بالتضمين:
   ```cpp
   #include <TPMS_RTL433.h>
   rtl_433_ESP rf;
   ```
3. اضبط منافذ CC1101 قبل `begin()`:
   ```cpp
   rtl_433_ESP::setPins(SCK, MISO, MOSI, CS, GDO0, GDO2);
   rf.setCallback(onPacket, messageBuffer, sizeof(messageBuffer));
   rtl_433_ESP::setRxBandwidth(102);   // الأفضل في القياس الميداني
   rf.setRSSIThreshold(9);
   rf.begin(GDO0, 433.92 + offsetKhz / 1000.0);
   rtl_433_ESP::setNotifyTask(xTaskGetCurrentTaskHandle());
   rf.enableReceiver();
   rtl_433_ESP::setDecoderMask(TPMS_ALL_MASK);   // أو TOYOTA_ONLY_MASK
   ```
4. في مهمة الراديو، استدعِ `rf.loop()` بعد كل `ulTaskNotifyTake()`، وافحص الراديو كل 5 ثوانٍ بـ `rtl_433_ESP::isRadioAlive()` حتى يُعاد تشغيله إن توقف.
5. كل حزمة مفكوكة تصل إلى `onPacket()` بصيغة JSON، وفيها هذه الحقول: `protocol`، و`id`، و`pressure_kPa`، و`temperature_C`، و`rssi`، و`battery_ok` إن وُجد.
   - رقم المفكك: `rtl_433_ESP::protocolIndex(doc["protocol"])`.
   - اسمه المختصر: `tpmsShortName(idx)`.
   - هل هو مفكك ضعيف: `tpmsIsWeakDecoder(idx)`. المفككات الضعيفة تحتاج فلتر منطقية للقيم.
6. أضف هدف البناء إلى `tools/build.sh`، وإلى `.github/workflows/build.yml`.
7. اكتب دليل الجهاز في `docs/`: المنافذ، والميزات، والمعايرة.

## واجهة المكتبة (مختصر)

| الدالة | الوظيفة |
|---|---|
| `setPins()` / `getPins()` | منافذ CC1101 |
| `setFrequency(mhz)` / `getFrequency()` | تردد الاستقبال، ومعه معايرة انحراف الكريستال |
| `setRxBandwidth(khz)`، ومرادفها `setFskBandwidth` | عرض النطاق: 58، 68، 81، 102، 116، 135، 162، 203، 232، 270... |
| `setRSSIThreshold(dB)` | كم dB فوق أرضية الضجيج يجب أن تكون الإشارة لالتقاطها |
| `setDecoderMask(mask)`، أو `applyTpmsMask` / `applyToyotaOnlyDecoders` | البروتوكولات المفعّلة |
| `decoderCount()` / `decoderName(i)` / `decoderShortName(i)` / `decoderPriority(i)` / `protocolIndex(name)` / `activeDecoderCount()` | معلومات المفككات |
| `statusSnapshot()` | الإشارات الملتقطة والمفكوكة والمفقودة، وأرضية الضجيج، والعتبة |
| `isRadioAlive()` | فحص صحة الـ CC1101 وإعادته لوضع الاستقبال |
| `spiTake()` / `spiGive()` | قفل ناقل SPI، لأي وصول مباشر لسجلات الراديو |
