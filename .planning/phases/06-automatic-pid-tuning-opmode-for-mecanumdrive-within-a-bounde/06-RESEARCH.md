# Phase 6: Automatic PID Tuning OpMode for MecanumDrive Within a Bounded Square - Research

**Researched:** 2026-08-06
**Domain:** FTC Android Java LinearOpMode, mecanum pose PID trial execution
**Confidence:** HIGH for existing-code integration; MEDIUM for physical tuning policy

<user_constraints>
## User Constraints (from CONTEXT.md)

### Locked Decisions
- **D-01:** Run many different test cases, not one tuning maneuver.
- **D-02:** Cover multiple translation directions, multiple headings, and combined X/Y/heading targets.
- **D-03:** Use targets clamped to the permitted 50 x 50 cm square; selected targets must not command outside it.
- **D-04:** When a target or generated motion approaches/exceeds the boundary, clamp it and continue the tuning sequence.
- **D-05:** Keep heading convention: positive heading means counter-clockwise, negative means clockwise.
- **D-06:** Display measured results through FTC telemetry only. Do not write gains or results to files.

### Claude's Discretion
- Trial ordering, scoring formula, settle detection, gain sweep strategy, telemetry fields, and low-power limits.
- Exact safe margin inside the 50 cm square, provided no generated target exceeds the boundary.

### Deferred Ideas (OUT OF SCOPE)
None — discussion stayed within phase scope.
</user_constraints>

<phase_requirements>
## Phase Requirements

| ID | Description | Research Support |
|----|-------------|------------------|
| TBD | Phase 6 requirements are not yet assigned in REQUIREMENTS.md. | Research defines bounded trial matrix, clamping, telemetry metrics, and stop safety for planner task decomposition. |
</phase_requirements>

## Summary

Phase 6 should add one calibration `LinearOpMode` under the existing `opmode` package. Reuse `RobotHardware`, `Localizer`, and `MecanumDrive`; do not change PID implementation, odometry signs, or drive behavior. The OpMode should execute a deterministic list of trials, reset pose before each trial, apply runtime position and heading gains, call `localizer.update()` before `drive.update()`, stop on completion/timeout/stop request, and report each result through FTC telemetry only. [VERIFIED: codebase]

The current drive already exposes all required tuning seams: runtime position and heading gains, power limits, tolerance, `goToPosition`, `update`, state, remaining error, field errors, heading error, and motor powers. Targets must be generated inside a smaller safe margin and clamped again immediately before `goToPosition`; this protects against matrix mistakes and guarantees no command exceeds the 50 x 50 cm boundary. [VERIFIED: codebase]

**Primary recommendation:** Use a fixed matrix of low-power trials covering cardinal/diagonal translations, heading-only turns, and combined pose targets; score time-to-settle, final translational error, final heading error, peak error, and timeout status, then show compact per-trial and best-so-far telemetry without persistence.

## Architectural Responsibility Map

| Capability | Primary Tier | Secondary Tier | Rationale |
|---|---|---|---|
| Trial orchestration and sequencing | FTC OpMode | — | LinearOpMode owns lifecycle, loop, timeout, and telemetry. |
| Pose measurement/reset | Localizer | IMU/odometry hardware | Existing Localizer owns pose and reset contract. |
| PID command generation | MecanumDrive | PidController | Existing drive owns three PID loops, field transform, slew, limits, and state. |
| Safety boundary enforcement | FTC OpMode | MecanumDrive limits | OpMode owns target clamping and trial timeout; drive limits motor output. |
| Result presentation | FTC telemetry | — | User explicitly forbids persistence and automatic constant changes. |

## Standard Stack

### Core

| Library | Version | Purpose | Why Standard |
|---|---|---|---|
| FTC SDK Java APIs | Existing project version | `LinearOpMode`, `@TeleOp`, `HardwareMap`, telemetry, lifecycle | Already used by project; no dependency change. [VERIFIED: codebase] |
| Existing `MecanumDrive` | Project source | Runtime PID target execution and diagnostics | Provides required public API. [VERIFIED: codebase] |
| Existing `Localizer` | Project source | Reset and field pose measurement | Provides `resetPoseAndHeading`, X/Y/heading getters, and calibration contract. [VERIFIED: codebase] |
| Java standard library | Android-supported Java | Arrays/lists, math, monotonic timing | Avoid new dependencies; deterministic matrix can use arrays. [ASSUMED] |

### Supporting

No new packages. No installation required. [VERIFIED: codebase]

## Architecture Patterns

### System Architecture Diagram

```text
INIT hardware + drive
        |
WAIT START ---> STOP REQUEST? ---> safe stop
        |
reset pose, clamp trial target, set gains/power limits
        |
 goToPosition(target)
        |
loop: localizer.update() -> drive.update() -> measure errors -> telemetry
        |                         |
 settled --------------------- timeout / stop request
        |
stop drive, record metrics, pause briefly, next trial
        |
all trials -> show summary/best trial -> safe stop
```

### Recommended Project Structure

```text
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/
└── MecanumDriveAutomaticPidTuningOpMode.java
```

Keep trial data as private nested value holders in the OpMode unless another feature needs them. No new abstraction layer needed for one OpMode. [VERIFIED: codebase conventions; ASSUMED recommendation]

### Pattern 1: Bounded deterministic trial runner

**What:** Store immutable trial definitions containing target X/Y/heading and gain set; run one definition to completion or timeout before advancing.

**When to use:** Hardware calibration where repeatability and operator-readable comparisons matter.

**Implementation contract:** Reset pose and heading at trial start; clamp X/Y to `[-safeHalfSquare, safeHalfSquare]`; normalize or retain heading according to existing `MecanumDrive.wrapHeadingError`; call `goToPosition`; update Localizer before drive each loop; use `System.nanoTime()` for elapsed duration. [VERIFIED: codebase for APIs; ASSUMED timing recommendation]

### Pattern 2: Completion plus settle qualification

Use `drive.atTarget(toleranceCm, toleranceDeg)` as the baseline completion signal, then require it continuously for a short settle window or a small consecutive-loop count. Record first-entry settle time, final errors, peak absolute errors, and timeout. A consecutive-loop count is less timing-sensitive than a fixed sleep and avoids declaring success from one noisy sample. [VERIFIED: codebase for `atTarget`; ASSUMED test policy]

### Pattern 3: Telemetry-only result ledger

Maintain current trial index and an in-memory best score. During a run show target, gains, elapsed time, state, X/Y/heading errors, pose, power, clamp status, and completion/timeout. After each trial show one compact result line; after all trials show best score and instruct operator to manually record values. Never call file I/O or mutate `MecanumDrive.DEFAULT_*` constants. [VERIFIED: user constraint and codebase]

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---|---|---|---|
| PID calculation | Duplicate PID loop in OpMode | `MecanumDrive.setPositionGains`, `setHeadingGains`, `goToPosition`, `update` | Drive already owns synchronized X/Y/heading control. [VERIFIED: codebase] |
| Pose math | Recompute encoder/IMU pose | `Localizer.update` and getters | Avoid sign/offset divergence. [VERIFIED: codebase] |
| Motor mixing | Direct four-motor trial commands | `MecanumDrive` | Preserves field transform, normalization, slew, and limits. [VERIFIED: codebase] |
| Angle shortest-path logic | Custom heading error | `MecanumDrive.wrapHeadingError` / `atTarget` | Existing convention handles wrap. [VERIFIED: codebase] |
| Persistent tuning database | CSV/JSON/SharedPreferences output | FTC telemetry only | Explicit phase constraint; persistence creates unrequested state. [VERIFIED: CONTEXT.md] |

## Common Pitfalls

### Pitfall 1: Boundary interpreted as target-only safety
**What goes wrong:** A target is clamped, but a trial starts near an edge and overshoot carries robot outside the square.
**Why it happens:** PID dynamics and odometry error can exceed the target coordinate.
**How to avoid:** Use an interior safe margin, low translation/heading power, conservative gains, settle/timeout stop, and operator preflight confirmation that the physical square is clear. Target clamping is mandatory but cannot guarantee physical containment under overshoot.
**Warning signs:** Pose approaches ±25 cm, error remains large after target crossing, or target command changes unexpectedly.

### Pitfall 2: Resetting pose after starting drive
**What goes wrong:** Trial metrics include previous trial motion or target is interpreted from stale origin.
**Why it happens:** `goToPosition` captures target values while Localizer still reports old pose.
**How to avoid:** Stop drive, call `resetPoseAndHeading`, update/settle sensor baseline, then issue target.

### Pitfall 3: Wrong update order
**What goes wrong:** Controller uses one-loop-old pose and metrics misrepresent response.
**Why it happens:** Drive contract explicitly requires Localizer update before drive update.
**How to avoid:** Keep loop order `localizer.update(); drive.update();` exactly. [VERIFIED: codebase]

### Pitfall 4: Heading sign or wrap mistake
**What goes wrong:** Positive heading turns opposite expected direction or ±180° causes a false large error.
**Why it happens:** Project convention is positive counter-clockwise, and drive uses wrapped heading error.
**How to avoid:** Use positive/negative targets per D-05 and never calculate a second heading error in OpMode. [VERIFIED: CONTEXT.md and codebase]

### Pitfall 5: Gain sweep contaminates comparison
**What goes wrong:** Integral/derivative state or starting pose differs across gain trials.
**Why it happens:** Same drive instance retains controller state until `goToPosition` resets PID; trial starts are not equivalent if motion remains.
**How to avoid:** Stop, reset pose/heading, apply gains, then call `goToPosition` for every trial. Keep trial order deterministic and include gain IDs in telemetry.

### Pitfall 6: Blocking sleep hides stop request
**What goes wrong:** OpMode continues motion or delays safe stop during long sleep.
**Why it happens:** FTC lifecycle can request stop while code is blocked.
**How to avoid:** Use short sleeps only between trials and check `isStopRequested()`/`opModeIsActive()` in every wait and control loop; always stop in `finally`. [VERIFIED: existing OpMode pattern]

### Pitfall 7: Telemetry overload
**What goes wrong:** Operator cannot read current state and loop timing degrades.
**Why it happens:** Dumping every trial's full metrics every loop.
**How to avoid:** Show current trial plus a short rolling result summary; cap history to a small fixed number of lines.

## Code Examples

Verified project pattern:

```java
RobotHardware robot = new RobotHardware(hardwareMap);
Localizer localizer = robot.localizer;
MecanumDrive drive = new MecanumDrive(
        hardwareMap, "leftfront", "rightfront", "leftback", "rightback", localizer);
try {
    while (opModeIsActive()) {
        localizer.update();
        drive.update();
        telemetry.addData("pose", "X %.1f Y %.1f H %.1f",
                localizer.getX(), localizer.getY(), localizer.getHeadingDeg());
        telemetry.update();
        sleep(50);
    }
} finally {
    drive.stop();
}
```

Source: `MecanumDriveGamepadTargetTestOpMode.java` and `MecanumDrive.java`. [VERIFIED: codebase]

Target clamp should be a pure bounded operation and applied immediately before command issuance:

```java
private static double clamp(double value, double min, double max) {
    return Math.max(min, Math.min(max, value));
}
```

This is standard Java math usage; no external source required. [ASSUMED]

## State of the Art

| Old Approach | Current Approach | When Changed | Impact |
|---|---|---|---|
| One manual target selected by gamepad | Deterministic matrix of automatic trials | Phase 6 scope, 2026-08-06 | Repeatable coverage of directions, combined poses, and headings. [VERIFIED: CONTEXT.md] |
| Directly edit constants after each run | Runtime gain setters with telemetry-only comparison | Existing API and phase constraint | No source mutation or persistence during tuning. [VERIFIED: codebase and CONTEXT.md] |

**Deprecated/outdated:** None identified for this phase.

## Assumptions Log

| # | Claim | Section | Risk if Wrong |
|---|---|---|---|
| A1 | `System.nanoTime()` is available and monotonic enough for Android trial timing. | Pattern 1 | Timing metrics become noisy; use `ElapsedTime` if FTC SDK policy requires it. |
| A2 | A fixed consecutive-loop settle count is preferable to a fixed-duration settle window. | Pattern 2 | Trial comparison may depend on loop rate; document actual loop period. |
| A3 | An interior margin plus low power is sufficient operational protection for a 50 cm square. | Pitfall 1 | Robot can still physically overshoot; field operator must validate clearance and emergency stop. |
| A4 | No existing test framework is available under `TeamCode/src/test`. | Validation Architecture | Offline checks may need Wave 0 setup or remain hardware/manual. |

## Open Questions

1. **What exact physical origin and square orientation define ±25 cm?**
   - What we know: Localizer pose is resettable and uses field X/Y conventions.
   - What's unclear: Whether reset point is centered in the physical square and whether X/Y axes align with its edges.
   - Recommendation: Require operator placement/preflight and state origin assumption in OpMode telemetry before start.

2. **Which gains should the matrix sweep?**
   - What we know: Existing defaults are position `KP 0.05, KI 0.012, KD 0.04`; heading `KP 0.04, KI 0.006, KD 0.04`.
   - What's unclear: Safe robot-specific ranges and whether to sweep position and heading independently.
   - Recommendation: Start with a small hand-selected set around defaults, low power, then expand only after stable baseline trials.

3. **Does phase success require source-level tests or only hardware OpMode validation?**
   - What we know: Nyquist validation is enabled, but no `TeamCode/src/test` directory was found.
   - What's unclear: Build setup and available FTC test harness.
   - Recommendation: Add pure-Java tests only if project Gradle supports them; otherwise leave bounded clamp/scoring helpers pure and validate manually through telemetry.

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|---|---|---|---|---|
| FTC SDK / Android Gradle project | Compile and deploy OpMode | ✓ by repository structure | Not checked | None; existing project is required. |
| REV/FTC robot hardware | Runtime trials | Unknown | — | No meaningful software fallback; use dry compile/offline review only. |
| `TeamCode/src/test` infrastructure | Automated pure-Java validation | Not found | — | Manual telemetry validation; planner should inspect Gradle before adding tests. |
| New external packages | None | N/A | N/A | No installation. |

## Validation Architecture

### Test Framework

| Property | Value |
|---|---|
| Framework | No test framework detected in requested paths |
| Config file | Not found during research |
| Quick run command | FTC deploy and run OpMode; offline pure-Java check only if Gradle supports it |
| Full suite command | Existing project Gradle test task, to be confirmed during planning |

### Phase Requirements → Test Map

| Req ID | Behavior | Test Type | Automated Command | File Exists? |
|---|---|---|---|---|
| TBD-01 | Every generated X/Y target remains inside square | unit/pure Java | Project test task, command TBD | No |
| TBD-02 | Matrix covers cardinal, diagonal, combined, positive and negative headings | unit/static review | Project test task, command TBD | No |
| TBD-03 | Stop request and timeout always stop drive | hardware/manual | FTC OpMode run | No |
| TBD-04 | Telemetry reports clamp and trial metrics without persistence | hardware/manual | FTC OpMode run | No |
| TBD-05 | Localizer update precedes drive update | static review/manual | Code review | No |

### Sampling Rate

- **Per task commit:** Compile TeamCode; run any available pure-Java checks.
- **Per wave merge:** Deploy to robot and execute a short low-power subset.
- **Phase gate:** Full matrix completes without boundary command, timeout leaves motors stopped, and telemetry contains enough data for manual gain selection.

### Wave 0 Gaps

- [ ] Confirm Gradle test support and add one small pure-Java check for clamp and trial matrix if supported.
- [ ] Define physical preflight procedure and emergency stop/operator spotter requirement.
- [ ] Confirm hardware configuration contains `leftfront`, `leftback`, `rightfront`, `rightback`, `imu`, and odometry devices before run.

## Security Domain

This phase has no network, authentication, persistence, or untrusted input surface. [VERIFIED: scope]

| ASVS Category | Applies | Standard Control |
|---|---|---|
| V2 Authentication | no | N/A |
| V3 Session Management | no | N/A |
| V4 Access Control | no | N/A |
| V5 Input Validation | limited | Validate finite gains/targets; existing drive validates finite targets. |
| V6 Cryptography | no | N/A |

## Sources

### Primary (HIGH confidence)

- `MecanumDrive.java` — runtime gain setters, target execution, limits, state, error getters, and update order. [VERIFIED: codebase]
- `Localizer.java` — pose reset, X/Y/heading access, calibration and sign conventions. [VERIFIED: codebase]
- `RobotHardware.java` — hardware names and Localizer construction. [VERIFIED: codebase]
- `MecanumDriveGamepadTargetTestOpMode.java` — existing LinearOpMode lifecycle and telemetry pattern. [VERIFIED: codebase]
- `06-CONTEXT.md` — locked phase decisions and scope. [VERIFIED: codebase]
- `ROADMAP.md`, `REQUIREMENTS.md`, `STATE.md`, `PROJECT.md` — project constraints and phase status. [VERIFIED: codebase]

### Secondary (MEDIUM confidence)

- None required; phase uses existing project APIs and no external package.

### Tertiary (LOW confidence)

- None; assumptions are isolated in Assumptions Log.

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH — existing source APIs inspected; no package additions.
- Architecture: HIGH — lifecycle and integration points match existing OpMode.
- Pitfalls: MEDIUM — software pitfalls verified; physical overshoot and tuning ranges require robot testing.

**Research date:** 2026-08-06
**Valid until:** 2026-09-05, unless FTC SDK or drive API changes.
