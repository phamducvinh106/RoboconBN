---
phase: 02-two-mode-opencv-camera
status: replan_required
planning_revision: 2026-08-07
hardware_first: true
plans: [02-01, 02-02, 02-03, 02-04, 02-05]
requirements: [LIFT-01, LIFT-02, LIFT-03, LIFT-04, LIFT-05, LIFT-06, MECH-01, MECH-02, MECH-03, MECH-04, MECH-05, AUTO-01, AUTO-05]
---

# Phase 2 Planning Verification Update

## User decision applied

One Pi5 I2C device (`pi5Camera`, 7-bit address 0x42) must be detected by Control Hub and pass address/read/write/heartbeat/channel checks before configuration and orchestration. Every physical subsystem gets its own manager class, with narrow interfaces, injectable seams, telemetry, and localized JSON calibration values. Two explicit logical channels (`webcam1`, `webcam2`) remain preserved on one device. The logical camera payload is packed 20 bits, not 20 bytes: unsigned X bits 0-7, unsigned Y bits 8-15, left type bits 16-17, right type bits 18-19, with explicit masks/shifts and configurable code 0..3 to block type 01..04 mapping. Invalid/reserved codes, stale, partial, or incomplete reads cannot authorize movement. Phase 2 defines placeholder transport and logical decoding only; physical UART/I2C framing, endian/register layout, checksum, sentinel policy, screen dimensions/axis origin, and I2C read atomicity remain deferred; pinned FTC SDK API verification remains a prerequisite. Encoder-confirmed arrival gates and the no-timeout decision remain preserved. Error recovery discussion remains deferred.

## Plan order and dependencies

1. `02-01`: manager contracts, seven physical-subsystem managers, and Pi5 I2C placeholder camera/frame contracts.
2. `02-02`: Control Hub HardwareMap detection plus I2C address scan/read/write/heartbeat and both explicit logical channel checks, then isolated hardware communication and hardware-first validation.
3. `02-03`: strict centralized JSON after I2C/camera communication evidence.
4. `02-04`: manager-backed pickup/state-machine integration and production wiring.
5. `02-05`: serial placement, release/back-out wiring, acceptance, and traceability.

## Requirement traceability

- `LIFT-01`, `MECH-01`, `MECH-02`: `02-01`, `02-02`, `02-03`, `02-04`, `02-05`.
- `LIFT-02`, `MECH-03`, `MECH-04`: `02-01`, `02-02`, `02-03`, `02-04`, `02-05`.
- `LIFT-03`, `AUTO-01`: `02-04`, `02-05`.
- `LIFT-04`, `LIFT-05`, `LIFT-06`, `AUTO-05`: `02-01`, `02-03`, `02-04`, `02-05`.
- `MECH-05`: `02-01`, `02-02`, `02-04`, `02-05` (placeholder two-channel camera contract only in Phase 2; OpenCV/parser deferred).

## Remaining acceptance conditions

Physical hardware, polarity, timing, clearance, calibration, release sensing, and six-cycle evidence remain human-gated. No plan claims those results before execution.
