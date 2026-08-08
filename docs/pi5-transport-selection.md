# Pi 5 USB CDC Transport (Pi ↔ Control Hub)

The project uses **USB CDC only** between Raspberry Pi 5 and REV Control Hub.

**Default wiring:** Pi USB gadget (`/dev/ttyGS0`) → Control Hub USB port.

## Run Pi vision

```bash
python3 main.py --cdc-device /dev/ttyGS0
```

Link test before vision: run `main.py` on Pi + **Pi USB CDC Communication Test** on Hub.

Gadget setup: [`pi5-cdc-gadget-setup.md`](pi5-cdc-gadget-setup.md)

Use `--no-cdc` for JSON stdout only (dev machines without USB gadget).

## Hub side

- OpMode: **Pi USB CDC Communication Test** (`PiCdcCommTestOpMode`)
- Lifting OpModes use `Pi5CameraTransportFactory.create()` → `PiCdcCameraTransport`

## Compliance

Confirm with referees: [`pi5-transport-compliance.md`](pi5-transport-compliance.md).
