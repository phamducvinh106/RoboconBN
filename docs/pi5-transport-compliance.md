# Pi 5 Vision Transport — Compliance Checklist (Robocon BNMR)

**Action required:** Confirm the items below with referees before competition.

## Rule context (from team robot context)

- *"Camera AI phải nối trực tiếp bộ điều khiển"*
- *"xử lý ảnh bằng thư viện cục bộ; không Internet, không máy tính/điện thoại ngoài"*

## Open questions for referees

| # | Question | Impact if disallowed |
|---|----------|----------------------|
| 1 | Is a Raspberry Pi 5 (on-robot, offline) acting as a **camera co-processor** allowed if detection runs locally and only metadata is sent to the Control Hub? | Must move YOLO inference onto the Hub or an approved camera module. |
| 2 | Is **UART** from Pi GPIO14 to Control Hub digital input or UART debug port allowed? FTC marks UART as debug-only; Robocon rules may differ. | Need USB serial or approved alternative. |
| 3 | Is **USB serial** (Pi USB gadget → Control Hub USB) allowed for metadata only? | Must use GPIO UART or integrate vision on Hub. |

## Current implementation

- **Transport:** UART `$V1,...` frames @ **9600** baud (default, digital soft-UART)
- **Hub reader:** soft-UART on digital `pi5UartRx` (max 19200 baud in code)

## Wiring safety

- Common ground between Pi and Control Hub
- Logic levels: both 3.3 V — no level shifter needed
- Route signal wires away from stepper motor power

## Sign-off (fill before match)

| Role | Name | Date | Transport approved |
|------|------|------|--------------------|
| Team lead | | | [ ] UART GPIO [ ] UART 3-pin [ ] USB |
| Referee | | | |
