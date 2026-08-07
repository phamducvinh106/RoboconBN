# Phase 2: Lifting Sequence State Machine - Research

**Researched:** 2026-08-07
**Domain:** FTC Android Java, elevator step/dir control, mecanum drive, odometry, sensor-gated autonomous state machine
**Confidence:** HIGH for repository patterns and locked constraints; MEDIUM for unmeasured hardware timing/positions

<user_constraints>
## User Constraints (from 02-CONTEXT.md)

### Locked Decisions

- Hai block dùng cùng placement sequence.
- Block trái đặt trước block phải.
- `READY1` nằm sau `PLACE` và trước `MOVE_TO_PLACEMENT_POSITION`.
- `HOME` chỉ chạy khi robot đã ở đúng pose đặt.
- Camera chỉ cung cấp detection; state machine/drive mới phát lệnh chuyển động.
- Không thêm dependency; giữ FTC SDK/OpenCV hiện có.
- D-01: every runtime-tunable lifting parameter comes from strict versioned external JSON; Java keeps only structural safety constants and state/enum identifiers, with invalid/missing config entering `SAFE_STOP` and no silent defaults.

### Claude's Discretion

1. Retry camera tại chỗ hay lùi về pose scan sau `CAMERA_STALE`?
2. `IR_PARTIAL` được phép tiến tối đa bao nhiêu cm trước khi abort?
3. Khi `RELEASE_UNCONFIRMED`, giữ block hay lùi 20 cm rồi chờ operator?
4. Sau `SAFE_STOP`, chỉ cho reset toàn chu kỳ hay cho resume tại `shelf/level` sau kiểm tra thủ công?
5. Pose factory cố định có cần giới hạn sai số X/Y/heading riêng không?
6. `BACK_OUT_AFTER_RELEASE_20CM` dùng tolerance bao nhiêu cm và có cần xác nhận fork đã rời block bằng cảm biến không?
7. Khi một shelf/level lỗi, bỏ lượt đó hay dừng toàn bộ task?

### Deferred Ideas (OUT OF SCOPE)

- AprilTag localization
- Deep-learning detector
- Separate `SingleTargetCamera`/`MultiTargetCamera` classes
- Automatic task 2 before task 1 completion
</user_constraints>

<phase_requirements>
## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| LIFT-01 | Home elevator, select `READY1`/`READY2`, return `HOME` using bounded pulses and `endstop1` | `RobotHardware` already defines elevator positions and pulse primitive; planner must make operations cooperative and bounded. |
| LIFT-02 | Control `PLACE`/`HOLD`, scan/center, approach only with both IR valid, then lift | Context sequence and sensor polarity establish explicit gates and ownership. |
| LIFT-03 | Route by left block class; place left then right; use `READY1` to push deeper cargo | Factory mapping must be data/config, not scattered conditionals; placement is two serial sub-sequences. |
| LIFT-04 | Stop request, stale camera, failed IR, safe shutdown in every state; recovery policy deferred | Existing OpMode `try/finally`, drive stop, and SAFE_STOP contract provide implementation pattern. |
| MECH-01 | Endstop homing stops and resets logical position | `RobotHardware.elevatorHomed()` uses active-low `endstop1`; current home loop requires cooperative redesign. |
| MECH-02 | Step/dir bounded motion | Existing `ElevatorState` step targets and pulse timing are reusable, but hardware timing remains unmeasured. |
| MECH-03 | Fork PLACE/HOLD poses | Servo names exist; actual positions are not present and require constants/hardware calibration. |
| MECH-04 | Debounced left/right IR presence only | Existing `cargoReady()` active-low contract; add continuous debounce in state logic, not classification. |
| MECH-05 | Multi-target classification, left centering, mechanical right alignment, dual pickup/lift | Camera remains result provider; Phase 2 should consume a narrow result contract and stale age. |
| AUTO-01 | Bounded scan → center → IR → pickup → transport → place states | Context provides canonical state order and cycle gate. |
| AUTO-05 | Timeout, stop, missing detection, IR failure, jam, safe stop | Error table in context is the acceptance matrix. |
</phase_requirements>

## Summary

Phase 2 should add an explicit cooperative state machine around existing `RobotHardware`, `MecanumDrive`, and `Localizer`; do not place blocking autonomous sequencing inside hardware helpers. `RobotHardware` already owns exact device names, active-low IR/endstop interpretation, elevator logical targets, pulse timing, and actuator shutdown. [VERIFIED: `TeamCode/.../RobotHardware.java`]

Use one loop per OpMode/state machine tick: check `opModeIsActive()` and `isStopRequested()`, update odometry, run the current state's bounded action, publish telemetry, then yield. Each state owns entry timestamp, timeout, required preconditions, transition result, and failure code. Drive commands must be issued by state code; camera only publishes detection. [VERIFIED: `02-CONTEXT.md`, `MecanumDrive.java`, `AutomaticPidTuningOpMode.java`]

Primary recommendation: extract or extend non-blocking elevator/IR/fork primitives, then implement the canonical pickup and two serial placement sequences with a single `SAFE_STOP` exit path. Preserve current measured calibration and device names; treat servo poses, IR debounce, step timing, factory poses, tolerances, and retry policy as named constants requiring hardware validation. [VERIFIED: repository; [ASSUMED] final numeric values]

## Architectural Responsibility Map

| Capability | Primary Tier | Secondary Tier | Rationale |
|---|---|---|---|
| Elevator pulse/homing | Hardware wrapper | State machine | Wrapper owns pins and polarity; state machine owns deadlines and transitions. |
| Fork servo poses | Hardware wrapper | State machine | Centralize PLACE/HOLD writes; sequence decides when pose changes. |
| IR debounce/presence | State machine | Hardware wrapper | Presence gates are temporal and state-specific; hardware exposes raw/active-level reads. |
| Camera freshness/classification | Camera result API | State machine | Camera detects only; state machine decides retry, movement, and abort. |
| Pose/drive control | `Localizer` + `MecanumDrive` | State machine | Existing drive owns mixing/PID; state machine supplies bounded targets or slow manual commands. |
| Cycle orchestration/recovery | State machine OpMode | All hardware wrappers | Only orchestrator can enforce ordering and task completion invariants. |

## Standard Stack

| Component | Version | Purpose | Guidance |
|---|---|---|---|
| FTC SDK / Android Java | Repository-pinned | `LinearOpMode`, `HardwareMap`, motors, servos, digital channels | Reuse existing Gradle/FTC stack. [VERIFIED: repository] |
| `RobotHardware` | Repository source | Device contract, elevator targets, shutdown | Extend minimally; preserve names and active-low semantics. [VERIFIED: source] |
| `MecanumDrive` | Repository source | Robot/field drive and PID pose targets | Reuse `driveRobotCentric`, `goToPosition`, `atTarget`, `stop`. [VERIFIED: source] |
| `Localizer` | Repository source | `x`, `y`, heading and pose reset/update | Call `localizer.update()` before drive update/read. [VERIFIED: source] |
| Java stdlib | Platform | `System.nanoTime`, enums, immutable records/classes | No dependency installation. [VERIFIED: source] |

**Installation:** None.

## Architecture Patterns

### Cooperative state machine

Represent states as an enum matching context names. On entry capture `stateStartedNs`, reset per-state debounce/retry counters, and stop prior motion. On each tick, reject stop requests first, enforce monotonic timeout, then execute one bounded action. Transition only after explicit success predicate; failure enters `SAFE_STOP` with reason and location. [VERIFIED: context error contract; `AutomaticPidTuningOpMode` loop]

### Elevator command contract

`step` must be low before changing `dir`; pulse high for configured duration, then low for configured low interval. Home toward active-low `endstop1`; on activation immediately force step low, set logical position zero, and mark position known. Never lift unless homing succeeded and target height reached. Existing `RobotHardware` target values are reusable starting points, not validated field constants. [VERIFIED: `RobotHardware.java`; [ASSUMED] mechanical calibration]

### Drive ownership and pose gates

Use low-power robot-centric strafe for `CENTER_LEFT_SLOW`, slow forward for `APPROACH_IR_SLOW`, and odometry target control for fixed factory poses. Stop/hold drive before reading and saving shelf pose. Reject non-finite pose and invalid heading. `MecanumDrive` already validates finite targets and supports explicit stop/hold behavior. [VERIFIED: source/context]

### Serial placement transaction

After pickup: `LIFT` → `BACK_OUT` → `HOLD` → factory approach → `PLACE` → `READY1` → placement pose → `HOME` → release confirmation → 20 cm back-out. Complete left transaction before starting right. Re-home/re-lift as required for right transaction; do not assume elevator remains at lift height after left `HOME`. [VERIFIED: context]

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---|---|---|---|
| Mecanum mixing/PID | New motor mixing or PID loop | `MecanumDrive` | Existing signs, limits, slew, and pose tolerance are calibrated patterns. |
| Pose math | Localizer duplicate | `Localizer` | Encoder/IMU signs and offsets are phase-1 contract. |
| Hardware lookup | Repeated `hardwareMap.get` in state code | `RobotHardware` | Keeps exact names, modes, polarity, and shutdown centralized. |
| Camera lifecycle | Camera-driven motion or second camera controller | Existing camera result API + state machine | Prevents stale data from commanding motion and avoids lifecycle duplication. |
| Timing | `System.currentTimeMillis` deadlines | `System.nanoTime` | Monotonic timeout behavior already used by repository. |

## Common Pitfalls

- **Blocking elevator helpers:** Current `homeElevator()` and `moveElevatorTo()` loop internally and cannot inspect stop requests, OpMode activity, or state timeout. Convert to tickable commands or pass a cooperative cancellation predicate. [VERIFIED: source]
- **Incorrect active level:** `cargoReady()` returns true only when both IR channels are low; `elevatorHomed()` returns true when `endstop1` is low. Preserve active-low behavior and verify on hardware. [VERIFIED: source]
- **Direction change during pulse:** Set `dir` only while `step` is low; always force step low in success, timeout, and `finally` paths. [VERIFIED: context/source]
- **Stale camera movement:** Valid-looking old detection must not trigger strafe or forward movement. Gate on timestamp/age and bounded retry. [VERIFIED: context/requirements]
- **Single IR lift:** One active IR is partial confirmation only; never lift or save shelf pose until both are debounced active. [VERIFIED: context]
- **Pose saved while moving:** Stop drive and perform final `Localizer.update()` before storing shelf pose. [VERIFIED: context]
- **Task-order violation:** Do not start right placement, next level, or next shelf until left release/back-out and `CYCLE_COMPLETE` gates pass. [VERIFIED: context]
- **Unsafe generic finally:** `drive.stop()` alone does not guarantee servo safe pose or elevator step low. SAFE_STOP must stop drive, step pulses, and record failure context. [VERIFIED: context/source]

## Code Examples

### Loop and shutdown shape

```java
try {
    while (opModeIsActive() && !isStopRequested() && state != State.SAFE_STOP) {
        localizer.update();
        tickState();
        telemetry.update();
        idle();
    }
} finally {
    drive.stop();
    robot.stopActuators();
}
```

Pattern follows existing OpMode `try/finally` and drive-stop behavior; exact state API is implementation choice. [VERIFIED: `AutomaticPidTuningOpMode.java`; [ASSUMED] final class structure]

### Required transition gate shape

```java
if (!robot.cargoReadyDebounced(nowMs)) {
    if (elapsedMs >= IR_TIMEOUT_MS) fail("IR_TIMEOUT");
    return;
}
robot.stopActuators();
shelfPose = finitePose(localizer);
transition(State.LIFT1);
```

Use as planning shape only; names require implementation. [ASSUMED]

## Runtime State Inventory

| Category | Items Found | Action Required |
|---|---|---|
| Stored data | None found in repository; no DB/store referenced. | No migration. |
| Live service config | None found; FTC hardware configuration is external to git. | Verify device names and directions on Robot Controller before run. |
| OS-registered state | None found. | None. |
| Secrets/env vars | None found. | None. |
| Build artifacts / installed packages | No Phase-2-specific runtime artifact found. | Rebuild/deploy TeamCode after implementation. |

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|---|---|---|---|---|
| FTC SDK/Android Gradle project | Runtime OpMode | ✓ repository source | pinned | None for hardware run |
| FTC Robot Controller hardware | Elevator/IR/camera validation | ? not probeable from repo | — | Offline pure checks only |
| Physical elevator timing/servo/IR calibration | Safety acceptance | ? not measured | — | Planner must add hardware checkpoint |

Missing dependencies with no fallback: physical hardware is required for final polarity, travel, servo pose, and timeout validation.

## Validation Architecture

| Property | Value |
|---|---|
| Framework | Plain Java executable tests; repository has `LocalizerMathTest`, `MecanumDriveTest`, `MultiTargetCameraTest` |
| Config | No dedicated state-machine test framework detected |
| Quick run | Existing project-specific Java/Gradle test invocation; inspect root/TeamCode Gradle tasks during planning |
| Full suite | TeamCode Gradle verification task plus hardware OpMode |

| Requirement | Behavior | Test Type | Automated Command | File Exists? |
|---|---|---|---|---|
| LIFT-01/MECH-01 | Bounded homing, endstop reset, height transitions | unit simulation | project Java test command | No; Wave 0 |
| LIFT-02/MECH-04 | Debounced dual IR gate; partial IR cannot lift | unit | project Java test command | No; Wave 0 |
| LIFT-03 | Left-before-right placement and factory mapping | unit | project Java test command | No; Wave 0 |
| LIFT-04/AUTO-05 | Timeout, stop, stale camera, SAFE_STOP | unit | project Java test command | No; Wave 0 |
| MECH-02/03 | Pulse ordering and PLACE/HOLD commands | hardware + fake-channel unit | project Java test command plus OpMode | No; Wave 0 |

Sampling rate: run focused state transition test per task; full Gradle suite per wave; hardware OpMode before phase gate. Exact command remains open because Gradle task was not verified.

## Security Domain

No network, auth, or persistent-data surface. Runtime inputs still require validation: finite poses, bounded target steps, bounded retries/candidates, stale camera age, and finite timeouts. [VERIFIED: repository scope]

| ASVS Category | Applies | Standard Control |
|---|---|---|
| V2 Authentication | no | N/A |
| V3 Session Management | no | N/A |
| V4 Access Control | no | N/A |
| V5 Input Validation | yes | Reject non-finite pose, invalid enum/config, negative timeout, and unbounded step target. |
| V6 Cryptography | no | N/A |

## Phase 2 Camera/I2C Boundary

- Raspberry Pi 5 is camera data source behind one I2C device at 7-bit address 0x42, configured as FTC HardwareMap device `pi5Camera`; exact Android/FTC I2C manager API must be verified against pinned SDK before implementation.
- Keep `webcam1` and `webcam2` as explicit logical channels on that one device, selected through an explicit channel register/command; no silent fallback or merged identity.
- Add only placeholder device-manager/frame contracts with `valid` and `fresh`/timestamp semantics. Model payload as packed 20-bit logical data: X mask `0x000FF` shift 0, Y mask `0x0FF00` shift 8, left type mask `0x30000` shift 16, right type mask `0xC0000` shift 18; decode unsigned values and configurable code mapping 0..3 to block types 01..04. Register map, physical framing, endian order, checksum, sentinel policy, and atomic read semantics remain deferred.
- Do not implement OpenCV, template matching, candidate ranking, NMS, or camera classification in Phase 2; Phase 3 owns those behaviors.
- Invalid, incomplete, stale, partial, or reserved-code placeholder data cannot authorize centering, approach, lift, or any movement transition. User wording is 20-bit, not 20-byte; screen dimensions/axis origin and physical I2C read atomicity remain unresolved.

## Open Questions

1. Servo numeric positions for `PLACE` and `HOLD` are absent; hardware calibration must lock them.
2. IR debounce interval, partial-IR travel limit, and active polarity require hardware measurement despite current active-low code.
3. Factory coordinates and placement X/Y/heading tolerances are not in source; planner should use a route configuration object/constants and add field validation.
4. Decide whether any failure aborts whole six-cycle task or permits operator-approved reset; context leaves this open.
5. Exact Gradle command for plain Java tests was not established.

## Sources

### Primary (HIGH confidence)
- `.planning/ROADMAP.md`, `.planning/REQUIREMENTS.md`, `.planning/STATE.md` — phase scope and requirement mapping.
- `.planning/phases/02-two-mode-opencv-camera/02-CONTEXT.md` — locked sequence, invariants, hardware roles, error policy.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/RobotHardware.java` — device contract, active levels, elevator implementation.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/MecanumDrive.java` — drive API, finite validation, pose control, stop behavior.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/Localizer.java` — odometry contract and update/reset patterns.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/AutomaticPidTuningOpMode.java`, `OdometryCalibrationOpMode.java` — cooperative loop, telemetry, deadline, and finally patterns.

### Tertiary (LOW confidence)
- Final servo positions, debounce windows, factory tolerances, retry counts, and hardware timing are assumed until measured. [ASSUMED]

## Assumptions Log

| # | Claim | Section | Risk if Wrong |
|---|---|---|---|
| A1 | Existing elevator step targets are usable starting points. | Standard Stack | Wrong travel can collide with shelf/floor. |
| A2 | Plain Java executable tests remain accepted for state simulation. | Validation | Build may require JUnit or Android test setup. |
| A3 | Camera result API can expose freshness without changing camera ownership. | Architecture | Phase 3 API may require adapter work. |
| A4 | Fixed factory poses can be represented as named configuration data. | Architecture | Field coordinate source may require calibration OpMode. |

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH — direct source inspection.
- Architecture: HIGH — context and existing OpMode patterns are explicit.
- Hardware constants: MEDIUM/LOW — several values require physical measurement.

**Research date:** 2026-08-07
**Valid until:** 2026-09-06
