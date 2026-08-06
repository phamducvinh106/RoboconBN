# Phase 1: Localizer Validation - Research

**Researched:** 2026-08-06
**Domain:** FTC Java two-wheel dead-wheel odometry with REV Hub IMU
**Confidence:** MEDIUM

## User Constraints

No `01-CONTEXT.md` was present. Phase scope is constrained by `.planning/PROJECT.md`, `.planning/REQUIREMENTS.md`, and `.planning/ROADMAP.md`: validate localizer only; do not add AprilTags, new dependencies, or unrelated vision/mechanism behavior. Existing hardware names and `LogoFacingDirection.BACKWARD` / `UsbFacingDirection.UP` are fixed project decisions. [VERIFIED: codebase]

## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| LOC-01 | `rightfront` is strafe encoder; `leftfront` is forward encoder | `RobotHardware` constructs `Localizer(rightfront, leftfront, imu, ...)`; calibration opmode uses same object. [VERIFIED: codebase] |
| LOC-02 | Expose calibrated diameter, ticks/rev, signs, offsets, scale, deadbands | Current constants are private in `Localizer`; plan should expose or report them through a stable calibration contract. [VERIFIED: codebase] |
| LOC-03 | Correct IMU orientation and yaw sign | `RobotHardware` already passes BACKWARD/UP and `Localizer` initializes/resetYaw. Physical yaw direction still requires robot test. [VERIFIED: codebase] |
| LOC-04 | Correct primitive and combined pose signs/magnitudes | Current transform must be checked against explicit robot/global frame fixtures and measured straight/strafe/turn runs. [VERIFIED: codebase] |
| LOC-05 | Verify update, wrapping, reset, and offset suggestions | Current implementation has deadbands, angle wrapping, reset methods, and 360-degree offset OpMode; repeatable offline checks and bounded hardware runs are missing. [VERIFIED: codebase] |

## Summary

`Localizer` already owns two-wheel odometry, IMU setup, pose integration, reset behavior, and offset suggestions. `RobotHardware` wires `rightfront` as the perpendicular/strafe pod and `leftfront` as the parallel/forward pod, with REV Hub orientation `LogoFacingDirection.BACKWARD` and `UsbFacingDirection.UP`. [VERIFIED: codebase]

Current tuning values are `WHEEL_DIAMETER_CM=4.8`, `TICKS_PER_REV=2000`, both encoder signs `-1`, heading sign `+1`, parallel Y offset `5.0 cm`, perpendicular X offset `-25.0 cm`, heading epsilon `1e-5 rad`, tick deadband `2`, and offset-calibration minimum accumulated heading `0.2 rad`. These are code defaults, not measured hardware truth; treat them as hypotheses until telemetry records physical movement and repeatability. [VERIFIED: codebase]

Primary recommendation: keep the two-pod architecture, first run a stationary sign/yaw test, then independent straight and strafe distance tests, then low-speed rotation and combined-motion checks. Record measured constants and only change signs/scale/offsets after each isolated test passes. Do not trust the existing 360-degree OpMode as a complete calibration: it has no timeout, stop-safe `finally`, or explicit mechanical rotation-distance validation. [VERIFIED: codebase]

## Architectural Responsibility Map

| Capability | Primary Tier | Secondary Tier | Rationale |
|------------|-------------|----------------|-----------|
| Encoder sampling and tick-to-distance conversion | API / Backend (TeamCode core) | Hardware | `Localizer` owns motor reads and scale constants. [VERIFIED: codebase] |
| IMU orientation, yaw reset, heading sign | API / Backend (TeamCode core) | Hardware | `Localizer` configures IMU; physical mounting and direction must be tested on robot. [VERIFIED: codebase] |
| Pose integration and coordinate transform | API / Backend (TeamCode core) | — | `update()` computes local deltas then integrates global X/Y. [VERIFIED: codebase] |
| Calibration telemetry and controlled motion | Browser / Client equivalent: FTC OpMode entry point | API / Backend | `OdometryCalibrationOpMode` drives test procedure and displays measurements while `Localizer` supplies data. [VERIFIED: codebase] |
| Offline mathematical verification | API / Backend test tier | — | Existing project convention places offline checks under `TeamCode/src/main/java/.../test`; no Localizer test exists. [VERIFIED: codebase] |

## Standard Stack

### Core

| Library | Version | Purpose | Why Standard |
|---------|---------|---------|--------------|
| FTC SDK `DcMotorEx` | 11.2.1 project release | Encoder position reads | Existing hardware API; no new dependency. [VERIFIED: README.md / official FTC release notes] |
| FTC SDK `IMU` + `RevHubOrientationOnRobot` | 11.2.1 project release | Oriented yaw measurement | Existing REV Hub API and project wiring. [VERIFIED: codebase; README.md] |
| Java `Math` | Java 8 source compatibility | radians, trig, wrapping | Existing implementation and SDK-compatible standard library. [VERIFIED: codebase; README.md] |

### Supporting

None. Do not install packages for this phase. [VERIFIED: project constraints]

## Architecture Patterns

### Data flow

```text
REV motor encoders (rightfront, leftfront) + oriented IMU yaw
        -> Localizer.update()
        -> signed ticks * (pi * wheelDiameter / ticksPerRev)
        -> rotation compensation using pod offsets
        -> robot frame (+forward, +left)
        -> global frame (+X right, +Y forward)
        -> pose getters and telemetry
```

### Current mapping

`RobotHardware` passes `rightfront` first and `leftfront` second. Therefore `rightfront` is the perpendicular/strafe pod and `leftfront` is the parallel/forward pod. [VERIFIED: codebase]

Current local integration is:
- `forwardLocal = parallelDelta + parallelYOffset * deltaHeading`
- `leftLocal = perpendicularDelta - perpendicularXOffset * deltaHeading`

Current global integration is:
- `xGlobal = -forwardLocal * sin(theta) - leftLocal * cos(theta)`
- `yGlobal = forwardLocal * cos(theta) - leftLocal * sin(theta)`

At heading zero, this produces `forward -> -X?` specifically `forwardLocal > 0` gives `x=0, y>0`; `leftLocal > 0` gives `x<0, y=0`. Thus the stated global convention is `+X=right`, `+Y=forward`, while positive local-left maps to global negative X. Validate this explicitly; do not infer correctness from comments. [VERIFIED: codebase]

### Calibration sequence

1. **Static baseline:** initialize with pods unloaded/robot stationary; verify zero deltas and heading stable.
2. **Encoder sign:** lift one pod or roll the pod in its positive physical direction; log raw tick delta. Then drive robot slowly forward and strafe while comparing each pod to a tape-measured displacement. Select sign so intended positive local motion is positive in telemetry.
3. **Scale:** command a measured straight distance; calculate `cmPerTick = measuredCm / signedTicks`; derive `wheelDiameter = cmPerTick * ticksPerRev / pi`. Do not change diameter and ticks/rev simultaneously without an independent motor/encoder specification.
4. **IMU:** with robot square on floor, reset yaw; rotate slowly in known direction; verify heading sign and wrap through ±180° without a jump larger than the physical incremental turn.
5. **Pod offsets:** execute multiple low-slip rotations in both directions. Average `parallelDelta / deltaHeading` and `perpendicularDelta / deltaHeading`; compare with current suggested formulas. Reject runs with wheel slip or insufficient accumulated heading.
6. **Transform:** test forward, strafe, rotation, and combined motion at headings near 0°, +90°, and -90°; compare pose to tape/field marks.

The existing calibration OpMode rotates until accumulated heading reaches 360°, then reports suggested offsets. It uses `rotatePower=0.3`, sleeps one second, updates once, and loops without timeout. [VERIFIED: codebase]

### Anti-patterns to avoid

- Tuning encoder signs, scale, offsets, and transform signs in one run; isolates no cause. [ASSUMED]
- Treating `TICKS_PER_REV=2000` as confirmed merely because it compiles; encoder CPR depends on hardware and counting mode. [ASSUMED]
- Measuring pod offsets during high-speed or slipping rotation. [ASSUMED]
- Running calibration without an operator stop path, timeout, clear area, or motor stop in all exits. [VERIFIED: codebase; project safety constraints]

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---------|-------------|-------------|-----|
| IMU orientation transform | Custom axis/sign conversion | FTC `RevHubOrientationOnRobot` + `IMU.Parameters` | SDK owns sensor-frame orientation. [CITED: https://ftc-docs.firstinspires.org/] |
| Angle normalization | Ad hoc repeated `if` chains in callers | One tested `angleWrap` helper in `Localizer` | Prevents discontinuities at ±pi. [VERIFIED: codebase] |
| Hardware device lookup | Duplicate name/config logic | Existing `RobotHardware` wiring | Preserves case-sensitive hardware contract. [VERIFIED: codebase] |
| Physical calibration truth | Guessing constants from comments | Telemetry plus tape/turntable measurements | Software cannot infer wheel wear, mounting, or slip. [ASSUMED] |

## Runtime State Inventory

| Category | Items Found | Action Required |
|----------|-------------|------------------|
| Stored data | None found in reviewed project docs/source; no DB or persistent pose store referenced. [VERIFIED: reviewed codebase] | None |
| Live service config | Robot Controller hardware configuration is external to source and must contain `rightfront`, `leftfront`, and `imu`; exact configured motor directions are not visible in Java. [VERIFIED: codebase limitation] | Hardware inspection before test |
| OS-registered state | None relevant to localizer. [ASSUMED] | None |
| Secrets/env vars | None relevant to localizer. [VERIFIED: reviewed project files] | None |
| Build artifacts / installed packages | Existing FTC Gradle project only; no new package required. [VERIFIED: README.md / build files] | None |

## Common Pitfalls

### Wrong pod-to-axis mapping
**What goes wrong:** forward/strafe values appear swapped or signs seem inconsistent.
**Why:** constructor order is positional; `rightfront` must be perpendicular and `leftfront` parallel. [VERIFIED: codebase]
**How to avoid:** assert mapping in telemetry and test one axis at a time.
**Warning signs:** forward motion changes `lastLeftLocalCm`, or strafe changes `lastForwardLocalCm`.

### Wrong encoder counting convention
**What goes wrong:** distance scale is exactly off by a hardware-specific factor.
**Why:** `TICKS_PER_REV=2000` is a project constant, not verified encoder metadata. [VERIFIED: codebase; physical value unresolved]
**How to avoid:** use measured cm/tick and confirm motor/encoder configuration.
**Warning signs:** repeatable straight run has proportional error; signs are correct but magnitude is not.

### Coordinate-frame sign mismatch
**What goes wrong:** pose moves in a mirrored or rotated field direction.
**Why:** current transform uses non-obvious signs and local +left/global +right conventions. [VERIFIED: codebase]
**How to avoid:** record expected `(dX,dY)` for each primitive at known heading before tuning.
**Warning signs:** forward is correct at heading 0° but wrong at 90°; left strafe reports negative X by design.

### Offset suggestion contaminated by slip
**What goes wrong:** suggested offsets vary substantially across runs or have implausible signs.
**Why:** rotation slip and pod scrub contaminate accumulated encoder distance. [ASSUMED]
**How to avoid:** run both directions, low speed, multiple turns, average only repeatable runs.
**Warning signs:** clockwise and counterclockwise estimates disagree beyond measurement tolerance.

### Calibration motion never safely exits
**What goes wrong:** OpMode continues driving if heading never reaches target or stop handling is delayed.
**Why:** current loop has no elapsed-time limit and no `try/finally`; it only calls `drive.stop()` after normal loop exit. [VERIFIED: codebase]
**How to avoid:** planner must include timeout, `isStopRequested()` handling, and unconditional stop before publishing results.
**Warning signs:** frozen heading telemetry while motors keep power.

## Code Examples

### Existing hardware wiring pattern

```44:50:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/RobotHardware.java
localizer = new Localizer(
        rightfront,
        leftfront,
        imu,
        RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
        RevHubOrientationOnRobot.UsbFacingDirection.UP
);
```

### Existing scale and sign contract

```17:25:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/Localizer.java
private static final double WHEEL_DIAMETER_CM    = 4.8;
private static final double TICKS_PER_REV        = 2000.0;
private static final double PARALLEL_ENCODER_SIGN     = -1.0;
private static final double PERPENDICULAR_ENCODER_SIGN = -1.0;
private static final double HEADING_SIGN               =  1.0;
private static final double PARALLEL_Y_OFFSET_CM      =  5.0;
private static final double PERPENDICULAR_X_OFFSET_CM = -25.0;
```

### Existing rotation calibration entry point

```45:78:TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/OdometryCalibrationOpMode.java
double targetRotationDeg = 360.0;
double rotatePower = 0.3;
while (opModeIsActive()) {
    odo.update();
    double currentAccumDeg = Math.toDegrees(odo.getAccumHeadingRad());
    if (Math.abs(currentAccumDeg) >= targetRotationDeg) {
        break;
    }
    drive.setRawPowers(rotatePower, -rotatePower, rotatePower, -rotatePower);
}
drive.stop();
```

## State of the Art

| Old Approach | Current Approach | When Changed | Impact |
|--------------|------------------|--------------|--------|
| Custom IMU axis assumptions | SDK `RevHubOrientationOnRobot` orientation parameters | FTC SDK API practice; exact project release uses 11.2.1 | Keep orientation declaration centralized. [CITED: https://ftc-docs.firstinspires.org/] |
| Guessing pod offsets | Measured rotation-based offset suggestion plus repeatability checks | This phase | Requires physical calibration runs, not source-only edits. [VERIFIED: codebase for existing suggestion API; measurement recommendation ASSUMED] |

**Deprecated/outdated:** No phase-specific deprecated API found in reviewed source. [VERIFIED: reviewed codebase]

## Assumptions Log

| # | Claim | Section | Risk if Wrong |
|---|-------|---------|---------------|
| A1 | `TICKS_PER_REV=2000` may not match actual encoder counting mode. | Summary/Common Pitfalls | All distance magnitudes wrong. |
| A2 | Wheel diameter 4.8 cm is nominal and may differ under load/wear. | Summary/Common Pitfalls | Scale bias. |
| A3 | Pod offsets measured from robot center use current sign convention. | Calibration sequence | Offset compensation mirrored. |
| A4 | No OS/runtime registrations contain localizer state. | Runtime State Inventory | Hidden external state may be missed. |
| A5 | Rotation slip is significant enough to bias offset estimates. | Common Pitfalls | Could overstate need for repeated runs; verify empirically. |

## Open Questions

1. What are actual odometry wheel model, encoder CPR/ticks-per-revolution, and SDK counting mode?
   - Recommendation: record hardware datasheet/config and derive scale from measured straight travel.
2. What are motor direction settings in the Robot Controller configuration for `rightfront` and `leftfront`?
   - Recommendation: inspect configuration and log raw tick signs during manual motion before editing constants.
3. What physical direction should positive heading represent for this robot/global frame?
   - Recommendation: mark robot front, right, and field axes; perform clockwise and counterclockwise yaw test.
4. Are `PARALLEL_Y_OFFSET_CM=5.0` and `PERPENDICULAR_X_OFFSET_CM=-25.0` measured from robot center to pod contact lines?
   - Recommendation: measure perpendicular distances mechanically, then validate with low-slip rotations.
5. What tolerance defines pass for distance, heading, and combined transform?
   - Recommendation: choose explicit acceptance thresholds after tape measure, IMU repeatability, and field use-case accuracy are known.

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|------------|-------------|-----------|---------|----------|
| FTC Gradle project | Build/offline checks | Yes | README reports FTC SDK 11.2.1 | None needed |
| Android Studio / FTC Robot Controller | Deploy hardware OpMode | Not verified in session | — | Run Gradle/offline math checks; hardware validation remains blocked |
| Configured robot hardware | Encoder/IMU measurements | Not available to source review | — | None for physical calibration |

**Missing dependencies with no fallback:** Physical robot plus configured `rightfront`, `leftfront`, and `imu` blocks hardware sign/scale/orientation proof.

## Validation Architecture

### Test Framework

| Property | Value |
|----------|-------|
| Framework | No JUnit/Vitest-style test framework detected; existing checks are Java executable test classes. [VERIFIED: codebase] |
| Config file | None detected for Localizer tests |
| Quick run command | `gradlew.bat :TeamCode:assembleDebug` (build check; hardware behavior still manual) |
| Full suite command | `gradlew.bat assembleDebug` |

### Phase Requirements → Test Map

| Req ID | Behavior | Test Type | Automated Command | File Exists? |
|---------|----------|-----------|-------------------|--------------|
| LOC-01 | Constructor wiring maps rightfront=strafe/perpendicular and leftfront=forward/parallel | static/code review + hardware smoke | `gradlew.bat :TeamCode:assembleDebug` | No dedicated test; Wave 0 |
| LOC-02 | Scale/sign/offset/deadband constants match calibration record | unit/model | project test command after harness exists | No; Wave 0 |
| LOC-03 | BACKWARD/UP and yaw sign | hardware smoke | manual OpMode | No; hardware-only |
| LOC-04 | primitive and combined transforms | deterministic unit/model | project test command after harness exists | No; Wave 0 |
| LOC-05 | update, wrap, reset, suggestions | deterministic unit/model + hardware rotation | project test command after harness exists | No; Wave 0 |

### Sampling Rate

- Per task commit: `gradlew.bat :TeamCode:assembleDebug`
- Per wave merge: `gradlew.bat assembleDebug`
- Phase gate: offline checks plus documented hardware calibration runs green before `/gsd-verify-work`

### Wave 0 Gaps

- Add a minimal Localizer math model/test seam that does not require FTC hardware, or provide fake `DcMotorEx`/`IMU` adapters. [VERIFIED: no Localizer test exists]
- Add deterministic fixtures for heading wrap at `pi/-pi`, pose reset, primitive motion, and offset suggestion denominator guard.
- Add a hardware calibration telemetry checklist for raw ticks, signed deltas, heading, local deltas, global pose, and timeout/stop result.

## Security Domain

No network/authentication/data-storage surface. Safety still applies because calibration commands motors.

| ASVS Category | Applies | Standard Control |
|---------------|---------|-----------------|
| V2 Authentication | no | — |
| V3 Session Management | no | — |
| V4 Access Control | no | FTC OpMode start/stop controls | 
| V5 Input Validation | yes | Validate finite constants, nonzero ticks/rev and diameter before use |
| V6 Cryptography | no | — |

### Known Threat Patterns for FTC hardware

| Pattern | STRIDE | Standard Mitigation |
|---------|--------|---------------------|
| Calibration loop hangs while motors powered | Denial of service / physical safety | Deadline, stop-request check, unconditional motor stop |
| Wrong sign/scale drives robot unexpectedly | Tampering / physical hazard | Low power, wheels/pods clear, operator near stop, primitive tests first |
| Invalid NaN/zero calibration values | Tampering / unsafe control | Reject non-finite or non-positive scale inputs |

## Safety Checklist

- Raise or secure drive wheels/pods for sign tests; keep robot on blocks where appropriate.
- Use low power and a clear perimeter for rotation; no hands near pods/wheels.
- Require Driver Station stop to stop all four motors; use `finally` cleanup in calibration OpMode.
- Add a hard timeout shorter than the match control window; stop if IMU heading freezes or encoder values stop changing.
- Log raw and signed encoder deltas before accepting any sign change.
- Verify `resetPoseAndHeading()` only when robot is physically at known heading; `resetPose()` alone preserves current IMU heading baseline. [VERIFIED: codebase]
- Do not tune offsets from one clockwise run; repeat both directions and record variance.

## Sources

### Primary (HIGH confidence)
- `.planning/PROJECT.md`, `.planning/REQUIREMENTS.md`, `.planning/ROADMAP.md`, `.planning/STATE.md` — project scope and LOC requirements. [VERIFIED: codebase]
- `TeamCode/.../Localizer.java` — constants, update math, IMU initialization, reset, wrapping, offset suggestions. [VERIFIED: codebase]
- `TeamCode/.../RobotHardware.java` — hardware mapping and IMU orientation. [VERIFIED: codebase]
- `TeamCode/.../OdometryCalibrationOpMode.java` — current physical calibration procedure. [VERIFIED: codebase]
- `README.md` — FTC SDK/project release information, including 11.2.1. [VERIFIED: codebase]

### Secondary (MEDIUM confidence)
- [FTC Documentation](https://ftc-docs.firstinspires.org/) — official SDK/IMU orientation documentation reference. [CITED: official docs; page not fetched in this session]

### Tertiary (LOW confidence)
- Physical wheel/encoder and slip observations are explicitly assumptions until measured on robot. [ASSUMED]

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH — existing project and README identify FTC SDK 11.2.1; no package additions.
- Architecture: HIGH — wiring and integration code directly reviewed.
- Pitfalls: MEDIUM — software failure modes verified; physical calibration risks require robot measurements.

**Research date:** 2026-08-06
**Valid until:** 2026-09-05 for stable code findings; physical constants valid only until hardware changes.
