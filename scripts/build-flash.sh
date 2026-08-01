#!/usr/bin/env bash
#
# build-flash.sh — build the Falcon Core firmware and flash it to a connected
# ESP32-S3 over USB, without leaning on the VS Code extension.
#
# Handles the usual snags:
#   - sources the ESP-IDF environment
#   - runs `idf.py fullclean` automatically if the Python-env mismatch appears
#   - frees the serial port if something else is holding it open
#   - flashes with esptool + build/flash_args (no rebuild at flash time)
#
# Usage:  ./build-flash.sh [-p PORT] [-c] [-m]
#   -p PORT   serial port           (default: $FALCON_PORT or /dev/ttyACM0)
#   -c        force a fullclean before building
#   -m        open the serial monitor after flashing (Ctrl+] to exit)

set -euo pipefail

PROJ="/home/eli/Projects/Falcon-Flight/falcon-core/src"
IDF_EXPORT="/home/eli/.espressif/v6.0.1/esp-idf/export.sh"
PORT="${FALCON_PORT:-/dev/ttyACM0}"
FORCE_CLEAN=0
MONITOR=0

while [ $# -gt 0 ]; do
  case "$1" in
    -p) PORT="$2"; shift 2 ;;
    -c) FORCE_CLEAN=1; shift ;;
    -m) MONITOR=1; shift ;;
    -h|--help) sed -n '2,14p' "$0" | sed 's/^# \?//'; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

echo "==> Sourcing ESP-IDF environment"
# export.sh is not -u clean; relax nounset just for the source.
set +u
# shellcheck disable=SC1090
source "$IDF_EXPORT" >/dev/null
set -u

if [ ! -e "$PORT" ]; then
  echo "!! Serial port $PORT not found. Is the board plugged in?" >&2
  echo "   Available: $(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | tr '\n' ' ')" >&2
  exit 1
fi

[ "$FORCE_CLEAN" -eq 1 ] && { echo "==> fullclean (forced)"; idf.py -C "$PROJ" fullclean; }

echo "==> Building"
if ! idf.py -C "$PROJ" build 2>&1 | tee /tmp/falcon-build.log; then
  if grep -q "fullclean" /tmp/falcon-build.log; then
    echo "==> Python-env mismatch detected — running fullclean and retrying"
    idf.py -C "$PROJ" fullclean
    idf.py -C "$PROJ" build
  else
    echo "!! Build failed." >&2
    exit 1
  fi
fi

echo "==> Freeing serial port $PORT (if held)"
fuser -k "$PORT" 2>/dev/null || true
sleep 1

echo "==> Flashing to $PORT"
# flash_args lists the .bin paths relative to the build dir, so run from there.
( cd "$PROJ/build" && python -m esptool --chip esp32s3 -p "$PORT" -b 460800 \
    --before default-reset --after hard-reset \
    write-flash "@flash_args" )

echo "==> Done. Firmware flashed to $PORT."

if [ "$MONITOR" -eq 1 ]; then
  echo "==> Opening monitor (Ctrl+] to exit)"
  idf.py -C "$PROJ" -p "$PORT" monitor
fi
