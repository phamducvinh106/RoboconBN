#!/usr/bin/env bash
# Verify Pi UART: /dev/serial0 exists and loopback works when GPIO14 <-> GPIO15 are jumpered.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

echo "=== Pi UART loopback check ==="
if [[ ! -e /dev/serial0 ]]; then
  echo "FAIL: /dev/serial0 missing. Enable UART in /boot/firmware/config.txt:"
  echo "  enable_uart=1"
  echo "  dtoverlay=disable-bt"
  exit 1
fi
echo "OK: $(ls -l /dev/serial0)"

if ! grep -qE '^enable_uart=1' /boot/firmware/config.txt 2>/dev/null; then
  echo "WARN: enable_uart=1 not found in /boot/firmware/config.txt"
fi

echo ""
echo "Jumper GPIO14 (TX) to GPIO15 (RX), then press Enter to run loopback test..."
read -r _

python3 tools/pi5_bench.py --mode uart-listen --port /dev/serial0 --baud 9600 --seconds 3 &
LISTEN_PID=$!
sleep 0.5
python3 tools/pi5_bench.py --mode uart --count 5 --port /dev/serial0 --baud 9600
wait "$LISTEN_PID" || true
echo "If you saw RX heartbeat lines above, Pi UART / GPIO14 TX is working."
