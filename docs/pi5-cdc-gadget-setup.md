# Pi 5 USB CDC Gadget Setup (`/dev/ttyGS0`)

Use this when the Control Hub must receive vision metadata over USB CDC ACM (not GPIO UART).

## Hardware

- Pi 5 **USB-C** port (device/gadget) → USB A-to-C cable → REV Control Hub **USB-A** host port
- Power Pi separately; do not rely on Hub power for Pi 5 + dual webcams
- Webcams stay on Pi USB-A ports

## Boot config (persistent)

Edit `/boot/firmware/config.txt` — add at end:

```text
dtoverlay=dwc2,dr_mode=peripheral
```

Edit `/boot/firmware/cmdline.txt` — append on the **same line** (space-separated):

```text
modules-load=dwc2,g_serial
```

Reboot:

```bash
sudo reboot
```

## One-shot enable (until reboot)

```bash
sudo modprobe dwc2
sudo modprobe g_serial
ls -l /dev/ttyGS0
```

Expected:

```text
crw-rw---- 1 root dialout ... /dev/ttyGS0
```

## Python access

```bash
sudo usermod -aG dialout "$USER"
# log out/in or reboot
groups   # should include dialout
```

## Run vision + CDC TX

```bash
cd ~/RoboconBN
source .venv/bin/activate
pip install -r block_detected_for_pi/requirements.txt
python3 main.py --cdc-device /dev/ttyGS0
```

Smoke test (10 frames, no infinite loop):

```bash
python3 main.py --cdc-device /dev/ttyGS0 --frames 10
```

## Hub side

1. Build/deploy **TeamCode** APK to Control Hub
2. Driver Station → TeleOp → **Pi USB CDC Communication Test**
3. Grant USB permission when prompted
4. Telemetry should show `USB CDC: CONNECTED` and `LEFT` / `RIGHT` lines updating

## Troubleshooting

| Symptom | Check |
|---------|--------|
| No `/dev/ttyGS0` | `ls /sys/class/udc`, `dmesg \| grep -Ei 'dwc2\|g_serial\|ttyGS'` |
| Permission denied | `groups` includes `dialout` |
| Hub `NO DEVICE/PERMISSION` | Replug USB, restart Robot Controller, allow USB dialog |
| `LEFT/RIGHT WAITING` | Pi `main.py` running? JSON printing on Pi terminal? |
