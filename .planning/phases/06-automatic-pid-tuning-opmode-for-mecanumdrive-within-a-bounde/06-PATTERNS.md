# Phase 6: Automatic PID Tuning OpMode - Pattern Map

**Mapped:** 2026-08-06  
**Files analyzed:** 1 planned OpMode; 6 supporting analogs  
**Analogs found:** 1 exact, 4 strong role matches, 1 offline-test match

## File Classification

| New/Modified File | Role | Data Flow | Closest Analog | Match Quality |
|---|---|---|---|---|
| `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/AutomaticPidTuningOpMode.java` (planned name) | Linear FTC OpMode / calibration controller | event-driven trial sequence + request-response telemetry | `opmode/PidTuningOpMode.java` | exact |

No core source changes belong to this phase. `MecanumDrive`, `Localizer`, and `RobotHardware` are reused as-is.

## Pattern Assignments

### `AutomaticPidTuningOpMode.java` (LinearOpMode, trial sequence + telemetry)

**Primary analog:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/PidTuningOpMode.java`

**Imports and initialization** (lines 3-28):

```3:28:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/PidTuningOpMode.java
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Localizer;
import org.firstinspires.ftc.teamcode.core.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.RobotHardware;

@TeleOp(name = "PID Tuning", group = "Calibration")
public final class PidTuningOpMode extends LinearOpMode {
    ...
    RobotHardware robot = new RobotHardware(hardwareMap);
    Localizer localizer = robot.localizer;
    MecanumDrive drive = new MecanumDrive(
            hardwareMap, "leftfront", "rightfront", "leftback", "rightback", localizer
    );
    drive.setTolerance(2.0, 2.0);
    drive.setPowerLimits(0.3, 0.3);
    applyGains(drive);
```

Copy hardware construction, named motor mapping, low-power limits, and `Calibration` group. Automatic trials should use constants from `MecanumDrive` rather than duplicate default gains.

**Runtime gain application** (lines 113-116):

```113:116:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/PidTuningOpMode.java
private void applyGains(MecanumDrive drive) {
    drive.setPositionGains(positionKp, positionKi, positionKd);
    drive.setHeadingGains(headingKp, headingKi, headingKd);
}
```

Use `setPositionGains()` and `setHeadingGains()` before each trial or gain candidate. No file persistence and no mutation of `MecanumDrive` constants.

**Trial lifecycle and safe shutdown** (lines 46-110):

```46:110:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/PidTuningOpMode.java
try {
    while (opModeIsActive()) {
        localizer.update();
        ...
        drive.update();
        telemetry.addData("pose", "X %.2f / Y %.2f / H %.2f",
                localizer.getX(), localizer.getY(), localizer.getHeadingDeg());
        telemetry.addData("at target", drive.atTarget());
        telemetry.update();
        sleep(20);
    }
} finally {
    drive.stop();
}
```

Preserve update order: `localizer.update()` before `drive.update()`. Automatic controller should stop on `finally`, and should stop/abort a trial before reset or starting the next one.

**Closest bounded-target analog:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/MecanumDriveGamepadTargetTestOpMode.java` lines 13-21 and 36-77. It uses `POWER_LIMIT = 0.3`, `goToPosition()`, edge-triggered controls, pose/state/power telemetry, and `finally { drive.stop(); }`. Its targets are not bounded, so reuse lifecycle only; replace target generation with a clamp/validation function that keeps X/Y inside the 50 cm square and applies any safe margin.

**Pose reset and multi-axis commands:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/GotoPositionDirectionTestOpMode.java` lines 25-67. It resets pose before translation trials, resets pose and heading before rotation trials, calls `goToPosition(x, y, heading)`, and exposes field error, robot command, motor power, state, and `atTarget` telemetry. Use `resetPoseAndHeading()` at every trial if heading baseline must be deterministic.

## Reusable APIs and Constraints

### `MecanumDrive`

- `setPositionGains(kp, ki, kd)` and `setHeadingGains(kp, ki, kd)` provide runtime tuning.
- `setPowerLimits(positionLimit, headingLimit)` clamps PID output. Existing calibration OpModes use `0.30`; this is the strongest established safety baseline.
- `setTolerance(toleranceCm, toleranceDeg)` validates positive finite values.
- `goToPosition(xCm, yCm, headingDeg)` rejects non-finite targets, resets PID state, and enters `MOVING`.
- `update()` must run after Localizer update.
- `atTarget()` only means `HOLDING`; `atTarget(tolCm, tolDeg)` checks current pose and heading.
- `getRemainingError()`, `getLastFieldErrorX/Y()`, `getLastHeadingErrorDeg()`, `getLastRobotForward/Strafe/Rotate()`, and motor power getters support scoring and telemetry.
- `setSlewPerLoop()` defaults to `0.08`; changing it affects trial dynamics and should be treated as a tuning variable or left at default.
- Heading error uses shortest wrapped error in `[-180, 180]`; positive heading convention is counter-clockwise per phase decision.
- `HOLDING` can transition back to `MOVING` if pose leaves tolerance. Settle scoring must require stability over time, not one sample.

### `Localizer`

- `resetPose()` resets X/Y but preserves current IMU heading.
- `resetPoseAndHeading()` resets encoders, yaw, pose, and accumulated calibration state.
- `getX()` is global +right cm; `getY()` is global +forward cm; `getHeadingDeg()` returns calibrated heading.
- Current calibration signs and offsets are experimental and must not be changed during PID tuning.
- Encoder/IMU updates may be deadbanded; short trials can show zero deltas. Avoid scoring from a single loop.

### `RobotHardware`

- Constructor initializes all configured hardware and immediately calls `stopActuators()`.
- `robot.localizer` is the canonical Localizer instance.
- Construct MecanumDrive with names `leftfront`, `rightfront`, `leftback`, `rightback` and `robot.localizer`.
- Hardware map failure occurs during initialization; no fallback behavior exists.

## Shared Patterns

### Bounded target generation

**Apply to:** every automatic trial.

- Define square bounds explicitly, preferably symmetric around reset origin: `[-25, +25]` cm per axis, or another documented 50 cm span.
- Clamp generated X/Y before `goToPosition`; never rely on `MecanumDrive` to enforce field boundaries because it only validates finiteness.
- Include a safe margin if the physical boundary is measured from robot center or wheel envelope. Record unclamped and clamped target in telemetry.
- Rotation changes position through imperfect mecanum/odometry coupling, so boundary checks must inspect live pose during trial and abort/hold if position approaches limit.

### Trial state machine

Use one active trial at a time. Recommended states: `IDLE`, `RUNNING`, `SETTLING`, `COMPLETE`, `ABORTED`. Store trial index, start time, target, peak error, final error, and consecutive in-tolerance time. Start each trial from a reset pose, apply candidate gains, call `goToPosition`, then loop `localizer.update()` → `drive.update()` → telemetry. Mark success only after a sustained settle window; timeout and boundary breach must stop the drive and advance or abort safely.

### Telemetry-only results

FTC telemetry is the only output contract. Existing OpModes use `telemetry.addLine`, `addData`, then `update()` every 20–50 ms. Display trial index/name, candidate gains, target, live pose, field/heading errors, state, elapsed time, power, clamp/boundary status, settle status, and final score. Do not write JSON, preferences, files, or Java constants.

### Input and shutdown

Automatic mode may run after `waitForStart()` without gamepad control, but preserve an operator abort button if useful. Existing edge-triggered A/B/X/Y handling is the project pattern. Always call `drive.stop()` in `finally`; if no active loop remains after all trials, stop and leave final telemetry visible.

## Risks for Planning

1. **Physical safety:** A 50 cm target is not itself a safe robot footprint. Use a margin, low power (`0.30` established), and live boundary abort. Rotation can induce translation.
2. **Coordinate ambiguity:** X means right and Y means forward. `MecanumDrive` accepts field X/Y; do not swap axes. Positive heading is counter-clockwise.
3. **Heading wrap:** Score wrapped heading error, not raw target-current subtraction, especially near ±180°.
4. **No persistence:** Results disappear when OpMode ends by explicit decision. Telemetry must summarize each trial while running.
5. **No gain getter:** `MecanumDrive` exposes setters but no getters. Keep candidate gains in OpMode state and display those values.
6. **Timing:** `sleep()` and loop scheduling affect derivative and settle metrics. Use elapsed wall-clock time and a consistent loop delay; do not compare trial scores without recording duration.
7. **Integral carryover:** `goToPosition()` resets all PID controllers. Start every candidate with a fresh command; avoid changing gains mid-trial unless intentionally testing that behavior.
8. **Heading reset side effects:** `resetPoseAndHeading()` resets encoder modes and IMU yaw. It is suitable between trials but may incur sensor transient; allow one update/settle cycle before measuring.
9. **Target boundary vs generated motion:** Clamping target only satisfies command safety. Live pose can still drift beyond limits; boundary monitoring remains required.
10. **No automatic constants:** Tuning must not modify `MecanumDrive` defaults or `Localizer` calibration.

## Offline Test Analog

`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/MecanumDriveTest.java` provides a fake odometry provider and deterministic loop simulation. Relevant patterns are `FakeOdo`, `drive.update()`, simulated motor step, convergence checks, and timeout checks (lines 11-33, 129-153). If Phase 6 adds a test, keep it offline and assert clamp/state/scoring math without FTC hardware. A new OpMode-only feature does not require modifying this test unless extracted pure trial logic needs coverage.

## No Analog Found

| Concern | Reason |
|---|---|
| Automatic multi-trial gain sweep | Existing `PidTuningOpMode` is manual; no autonomous trial runner exists. |
| Boundary enforcement | Existing 50 cm tests command fixed targets but do not clamp or monitor live bounds. |
| Persistent tuning report | Explicitly out of scope; telemetry only. |

## Metadata

**Analog search scope:** `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/`, `opmode/`, `test/`; `.planning/phases/06-.../06-CONTEXT.md`  
**Files scanned:** `MecanumDrive.java`, `Localizer.java`, `RobotHardware.java`, `PidTuningOpMode.java`, `MecanumDriveGamepadTargetTestOpMode.java`, `GotoPositionDirectionTestOpMode.java`, `MecanumDriveTest.java`, plus OpMode pattern search.  
**Pattern extraction date:** 2026-08-06

## Planner Notes

Implement one new OpMode first. Keep trial generation, clamping, settle scoring, and telemetry private to that file unless a pure utility is proven necessary. Reuse all existing core APIs; do not change drive/localizer behavior.