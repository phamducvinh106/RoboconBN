#!/usr/bin/env bash
# Sync Pi 5 repo to origin/test and smoke-test CDC vision.
set -euo pipefail

REPO="${1:-$HOME/RoboconBN}"
cd "$REPO"

echo "==> branch"
git fetch origin
git switch test
git pull origin test
git log -1 --oneline

echo "==> python deps"
if [[ ! -d .venv ]]; then
  python3 -m venv .venv
fi
# shellcheck disable=SC1091
source .venv/bin/activate
python -m pip install --upgrade pip
pip install -r block_detected_for_pi/requirements.txt

echo "==> compile"
python3 -m compileall -q main.py block_detected_for_pi

echo "==> gadget"
if [[ ! -e /dev/ttyGS0 ]]; then
  echo "WARN: /dev/ttyGS0 missing — run: sudo modprobe dwc2 g_serial"
else
  ls -l /dev/ttyGS0
fi

echo "==> tests"
python3 -m unittest block_detected_for_pi.test_cdc -q
python3 -m unittest block_detected_for_pi.test_payload -q
python3 -m unittest block_detected_for_pi.test_monitor -q
python3 -m unittest block_detected_for_pi.test_model_benchmark -q

echo "==> vision smoke (headless, 10 frames)"
python3 main.py --no-cdc --no-ui --frames 10

echo "OK: Pi synced to test branch. Run CDC link test:"
echo "  python3 main.py --cdc-device /dev/ttyGS0"
