#!/usr/bin/env bash
#
# battery-life.sh — time how long the blimp stays powered, logging battery
# voltage and % to CSV. Probes the dashboard WebSocket (which carries the live
# telemetry): waits for the board to respond, starts a timer, then runs until
# the board has been unreachable for longer than the grace period (default
# 60s). Battery life is reported as the time to the LAST successful response,
# so the dead-grace window isn't counted against it.
#
# Each poll appends a row to the CSV so multiple test runs can be compared.
#
# Usage:  ./battery-life.sh [HOST] [-i POLL] [-g GRACE] [-t TIMEOUT] [-o CSV] [-n NAME]
#   HOST        board IP or hostname (default: $FALCON_HOST or 192.168.4.1)
#   -i POLL     seconds between polls              (default 5)
#   -g GRACE    seconds unreachable = dead         (default 60)
#   -t TIMEOUT  per-request WS timeout seconds     (default 3)
#   -o CSV      output CSV file (appended)         (default scripts/battery-log.csv)
#   -n NAME     run label written in each row      (default: start timestamp)

set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
HOST="${FALCON_HOST:-192.168.12.45}"
POLL=5
GRACE=60
TIMEOUT=3
CSV="${HERE}/battery-log.csv"
RUN=""

# First non-flag arg is the host.
args=()
while [ $# -gt 0 ]; do
  case "$1" in
    -i) POLL="$2"; shift 2 ;;
    -g) GRACE="$2"; shift 2 ;;
    -t) TIMEOUT="$2"; shift 2 ;;
    -o) CSV="$2"; shift 2 ;;
    -n) RUN="$2"; shift 2 ;;
    -h|--help) sed -n '2,20p' "$0" | sed 's/^# \?//'; exit 0 ;;
    *) args+=("$1"); shift ;;
  esac
done
[ "${#args[@]}" -gt 0 ] && HOST="${args[0]}"
[ -z "$RUN" ] && RUN="$(date '+%Y%m%d-%H%M%S')"

FETCH="${HERE}/fetch_battery.py"

# Pick a python that actually has websocket-client. The plain `python3` may be
# the ESP-IDF virtualenv (e.g. when this runs in a terminal that sourced the
# IDF environment for build-flash.sh), which lacks the module — that would make
# every probe fail silently and hang forever on "Waiting for telemetry".
PY=""
for cand in python3 /usr/bin/python3 python; do
  if command -v "$cand" >/dev/null 2>&1 && "$cand" -c "import websocket" 2>/dev/null; then
    PY="$cand"; break
  fi
done
if [ -z "$PY" ]; then
  echo "!! No python with 'websocket-client' found." >&2
  echo "   Install it:  pip install --user websocket-client" >&2
  echo "   (If your shell has the ESP-IDF env active, open a fresh terminal.)" >&2
  exit 1
fi

# Probe the WebSocket telemetry. On success, sets BATT_V and BATT_PCT and
# returns 0; on failure returns non-zero (board unreachable / no telemetry).
BATT_V=""
BATT_PCT=""
alive() {
  local out
  out="$("$PY" "$FETCH" "$HOST" "$TIMEOUT" 2>/dev/null)" || return 1
  BATT_V="${out%%,*}"
  BATT_PCT="${out##*,}"
  [ -n "$BATT_V" ]
}

fmt() { # seconds -> Hh Mm Ss
  local s=$1
  printf '%dh %02dm %02ds' $((s/3600)) $(((s%3600)/60)) $((s%60))
}

# Create CSV with a header if it doesn't exist yet.
if [ ! -f "$CSV" ]; then
  echo "run,timestamp,elapsed_s,status,voltage,percent" > "$CSV"
fi

log_row() { # status
  printf '%s,%s,%s,%s,%s,%s\n' \
    "$RUN" "$(date '+%Y-%m-%d %H:%M:%S')" "${1:-}" "$2" "${BATT_V}" "${BATT_PCT}" >> "$CSV"
}

echo "Falcon battery-life timer"
echo "  target : ws://${HOST}/ws"
echo "  poll   : ${POLL}s   grace: ${GRACE}s   ws timeout: ${TIMEOUT}s"
echo "  csv    : $CSV   (run='${RUN}')"
echo

# --- Wait for the board to come up -----------------------------------------
printf 'Waiting for telemetry'
until alive; do printf '.'; sleep "$POLL"; done
echo " up.  battery ${BATT_V}V ${BATT_PCT}%"

START=$(date +%s)
LAST_OK=$START
log_row 0 up
echo "Timer started at $(date '+%H:%M:%S'). Running until unreachable for >${GRACE}s."
echo

# --- Run until unreachable past the grace window ---------------------------
while true; do
  sleep "$POLL"
  now=$(date +%s)
  elapsed=$((now-START))
  if alive; then
    LAST_OK=$now
    log_row "$elapsed" up
    printf '\r[%s]  up   %sV  %s%%            ' \
      "$(fmt "$elapsed")" "$BATT_V" "$BATT_PCT"
  else
    BATT_V=""; BATT_PCT=""
    down=$((now-LAST_OK))
    log_row "$elapsed" down
    printf '\r[%s]  DOWN %ss (grace %ss)      ' \
      "$(fmt "$elapsed")" "$down" "$GRACE"
    if [ "$down" -ge "$GRACE" ]; then
      echo
      echo
      echo "Unreachable for ${down}s — battery considered dead."
      echo "Last response at $(date -d "@$LAST_OK" '+%H:%M:%S')."
      echo "==> Battery life: $(fmt $((LAST_OK-START)))  ($((LAST_OK-START))s)"
      echo "    Data logged to $CSV (run='${RUN}')."
      exit 0
    fi
  fi
done
