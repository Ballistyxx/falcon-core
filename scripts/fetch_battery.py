#!/usr/bin/env python3
"""Connect to the Falcon dashboard WebSocket, read one telemetry frame, and
print the battery reading as "voltage,percent" on stdout.

Exit 0 on success, 1 on any failure (used by battery-life.sh as both the
liveness probe and the data source).

Usage: fetch_battery.py HOST [TIMEOUT]
"""
import json
import sys

import websocket  # websocket-client


def main() -> int:
    host = sys.argv[1] if len(sys.argv) > 1 else "192.168.4.1"
    timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0

    try:
        ws = websocket.create_connection(f"ws://{host}/ws", timeout=timeout)
    except Exception:
        return 1

    try:
        ws.settimeout(timeout)
        # The firmware only pushes telemetry to sockets it has registered, and
        # the handshake-only registration is unreliable — the dashboard sends a
        # one-shot message on open to force it, so do the same here.
        ws.send("hello")
        # Telemetry is broadcast periodically; read a few frames until one
        # carries a battery field, then stop.
        for _ in range(10):
            msg = ws.recv()
            if not msg:
                continue
            try:
                data = json.loads(msg)
            except (ValueError, TypeError):
                continue
            batt = data.get("battery")
            if isinstance(batt, dict) and "voltage" in batt:
                print(f"{batt['voltage']:.3f},{batt.get('percent', '')}")
                return 0
        return 1
    except Exception:
        return 1
    finally:
        try:
            ws.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
