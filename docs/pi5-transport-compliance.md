# Pi 5 Vision Transport — Compliance Checklist (Robocon BNMR)

**Action required:** Confirm the items below with referees before competition.

## Rule context (from team robot context)

- *"Camera AI phải nối trực tiếp bộ điều khiển"*
- *"xử lý ảnh bằng thư viện cục bộ; không Internet, không máy tính/điện thoại ngoài"*

## Open questions for referees

| # | Question | Impact if disallowed |
|---|----------|----------------------|
| 1 | Is a Raspberry Pi 5 (on-robot, offline) acting as a **camera co-processor** allowed if detection runs locally and only metadata is sent to the Control Hub? | Must move YOLO inference onto the Hub or an approved camera module. |
| 2 | Is **USB serial** (Pi USB gadget → Control Hub USB) allowed for metadata only? | Must integrate vision on Hub or use an approved alternative. |

## Current implementation

- **Transport:** USB CDC JSON lines @ **115200** baud (Pi `/dev/ttyGS0` → Hub USB ACM)
- **Hub reader:** `PiBlockReceiver` + `PiCdcCameraTransport`

## Wiring safety

- Use a data-capable USB cable between Pi and Control Hub
- Route USB cable away from stepper motor power

## Sign-off (fill before match)

| Role | Name | Date | Transport approved |
|------|------|------|--------------------|
| Team lead | | | [ ] USB CDC |
| Referee | | | |
