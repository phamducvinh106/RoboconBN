# Pi 5 UART Transport (Pi ↔ Control Hub)

The project uses **UART only** between Raspberry Pi 5 and REV Control Hub.

**Default wiring:** Pi GPIO14 (TX) → Hub digital input `pi5UartRx` (soft-UART @ **9600** baud).

## Config

[`phase2-lifting-config.json`](../TeamCode/src/main/assets/phase2-lifting-config.json):

```json
{
  "pi5UartBaud": 9600,
  "pi5UartDevice": "pi5UartRx"
}
```

Robot Config: digital input named **`pi5UartRx`**.

## Run Pi vision

```bash
python3 main.py --uart-port /dev/serial0 --uart-baud 9600
```

(`9600` is the default — `--uart-baud` can be omitted.)

Link test before vision: `python3 tools/pi5_bench.py --loop` on Pi + **Pi5 UART Communication Test** on Hub.

Use `--mock-uart` on dev machines without serial hardware, or `--no-uart` for JSON stdout only.

## Baud limits

| Wiring | Baud |
|--------|------|
| Pi TX → Hub **digital IN** (soft-UART, default) | **9600** (max **19200** in Hub code) |
| Pi TX → Hub UART 3-pin (HW, not implemented in Hub reader) | would need HW reader + higher baud |

Pi and Hub JSON **must use the same** `pi5UartBaud`.

## Compliance

Confirm with referees: [`pi5-transport-compliance.md`](pi5-transport-compliance.md).
