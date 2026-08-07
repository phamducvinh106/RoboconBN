# Pi 5 UART → Control Hub

## Bring-up checklist (do in order)

1. **Offline protocol** (laptop/Pi, no wiring):
   ```bash
   python3 run_tests.py --suite pi5-uart --suite python-uart
   python3 tools/pi5_protocol_e2e.py
   ```
2. **Pi UART enable** — see [Pi setup](#pi-setup) below; verify `ls -l /dev/serial0`.
   Optional loopback (jumper GPIO14↔GPIO15): `bash tools/pi5_uart_loopback_check.sh`
3. **Wire Pi → Hub** — GPIO14 TX → `pi5UartRx`, GND → GND.
4. **Hub Robot Config** — add digital input `pi5UartRx` only (no motors required for link test).
   Confirm the **physical digital port number** (0–7) matches the wire — not the UART 3-pin debug port.
5. **Deploy TeamCode** APK to Control Hub.
6. **Link test** — Pi: `python3 tools/pi5_bench.py --loop`; Hub: OpMode **Pi5 UART Communication Test**.
7. **Vision** (after link stable ≥30s) — `python3 main.py --uart-port /dev/serial0`.

Success on Driver Station: `LINK=OK`, `webcam1 valid=true`, `hb` incrementing, `fresh=true`.

## Wiring (default)

| Pi 5 | Control Hub |
|------|-------------|
| GPIO14 (TXD0) | Digital input `pi5UartRx` |
| GND | GND |

Baud: **9600** on both Pi and Hub (`pi5UartBaud` in JSON).

**Rules:** Pi TX → Hub RX (digital input). Common ground required. Both 3.3 V — no level shifter. Keep signal wire short, away from stepper motor power.

## Pi setup

Enable UART in `/boot/firmware/config.txt`:

```
enable_uart=1
dtoverlay=disable-bt
```

Disable serial console on GPIO14/15:

```
sudo raspi-config   # Interface Options → Serial → login shell off, hardware on
```

Reboot, then verify:

```bash
ls -l /dev/serial0
python3 tools/pi5_bench.py --mode uart --count 5 --port /dev/serial0
```

Continuous heartbeat for link test:

```bash
python3 tools/pi5_bench.py --loop --port /dev/serial0 --baud 9600
```

Run vision + UART publish (after link test passes):

```bash
python3 main.py --uart-port /dev/serial0
```

## Hub setup

1. Robot Config → add digital input **`pi5UartRx`** (Pi TX → this pin).
2. Ensure `phase2-lifting-config.json` has `"pi5UartBaud": 9600` and `"pi5UartDevice": "pi5UartRx"`.
3. Run **Pi5 UART Communication Test** OpMode — check `LINK`, `rx bytes`, `heartbeat`, `fresh`.
4. Optional full hardware test: **Lifting Hardware Communication Test** (requires all robot devices).

## Frame format

```
$V1,<heartbeat>,<status>,<payload_hex>,<crc8>\n
```

Example: `$V1,42,87,1A2B3,C4\n`

## Bench test

```bash
python3 tools/pi5_bench.py --mode uart --count 10
python3 tools/pi5_bench.py --loop --mock          # dev machine without serial
python3 tools/pi5_bench.py --mode uart-listen --port /dev/serial0 --baud 9600
```

## Troubleshooting

| Symptom | Likely cause | Fix |
|---------|--------------|-----|
| `LINK=WAITING`, bytes=0 | No signal / wrong pin | TX→RX, common GND, Pi bench running |
| bytes>0, decodeErr rising | Baud mismatch or noise | Match 9600 both sides; shorten wire |
| hb stuck, fresh=false | Heartbeat not changing | Use `--loop`; check Pi publisher |
| OpMode init crash | Missing `pi5UartRx` in config | Add digital input in Robot Config |

## Notes

- Soft-UART on Hub digital accepts **300–19200** baud only (`FtcPi5SoftUartLineReader`). **Do not use 115200** with digital wiring.
- If upgrading baud after bench test, try **19200** max — change Pi `--uart-baud` and JSON together.
