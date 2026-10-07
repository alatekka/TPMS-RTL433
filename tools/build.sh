#!/usr/bin/env bash
# بناء ورفع برامج TPMS_RTL433
# الاستخدام:
#   tools/build.sh r3            # بناء نسخة R3 (ESP32-S3)
#   tools/build.sh dev           # بناء نسخة DEV (ESP32)
#   tools/build.sh dev COM5      # بناء ورفع إلى المنفذ
#   tools/build.sh v3rx COM5     # النسخة 3: وحدة الاستقبال
#   tools/build.sh v3ui COM6     # النسخة 3: وحدة الشاشة CYD
#   tools/build.sh all           # بناء الكل
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
CLI="${ARDUINO_CLI:-}"
if [ -z "$CLI" ]; then
  if command -v arduino-cli >/dev/null 2>&1; then CLI="arduino-cli"
  elif [ -x "/c/Program Files/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe" ]; then
    CLI="/c/Program Files/Arduino IDE/resources/app/lib/backend/resources/arduino-cli.exe"
  else echo "arduino-cli غير موجود. ثبّته أو عرّف ARDUINO_CLI"; exit 1; fi
fi

FQBN_R3="esp32:esp32:esp32s3:FlashSize=16M,PSRAM=opi,PartitionScheme=app3M_fat9M_16MB,FlashMode=qio,UploadSpeed=921600"
FQBN_DEV="esp32:esp32:esp32:FlashSize=4M,PartitionScheme=min_spiffs,UploadSpeed=921600"
FQBN_V3RX="esp32:esp32:esp32:FlashSize=4M,PartitionScheme=default,UploadSpeed=115200"
FQBN_V3UI="esp32:esp32:esp32:FlashSize=4M,PartitionScheme=min_spiffs,UploadSpeed=921600"

build() {
  local name="$1" fqbn="$2" sketch="$3" port="${4:-}"
  local extra=()
  [ -n "$port" ] && extra=(-u -p "$port")
  echo "=== $name ==="
  "$CLI" compile -b "$fqbn" --library "$ROOT" --build-path "$ROOT/.build/$name" -j 4 "${extra[@]}" "$ROOT/examples/$sketch"
}

case "${1:-all}" in
  r3)  build r3  "$FQBN_R3"  TPMS_R3_ESP32S3 "${2:-}" ;;
  dev) build dev "$FQBN_DEV" TPMS_DEV_ESP32  "${2:-}" ;;
  v3rx) build v3rx "$FQBN_V3RX" TPMS_V3_Receiver "${2:-}" ;;
  v3ui) build v3ui "$FQBN_V3UI" TPMS_V3_CYD_Display "${2:-}" ;;
  all) build r3 "$FQBN_R3" TPMS_R3_ESP32S3; build dev "$FQBN_DEV" TPMS_DEV_ESP32
       build v3rx "$FQBN_V3RX" TPMS_V3_Receiver; build v3ui "$FQBN_V3UI" TPMS_V3_CYD_Display ;;
  *)   echo "الاستخدام: $0 [r3|dev|v3rx|v3ui|all] [PORT]"; exit 1 ;;
esac
