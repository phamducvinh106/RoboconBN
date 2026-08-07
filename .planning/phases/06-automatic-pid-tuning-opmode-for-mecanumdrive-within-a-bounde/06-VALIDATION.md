# Phase 6 Validation

## Scope

Validate `AutomaticPidTuningOpMode` against Phase 6 decisions D-01 through D-06 and plan requirements PH6-01 through PH6-06. Implementation source remains unchanged by this artifact.

## Validation Architecture

- **Offline executable gate:** compile `TeamCode`, then run static/executable checks for target clamping, matrix composition, update order, shutdown paths, telemetry-only output, and forbidden persistence/network APIs.
- **Hardware gate:** deploy OpMode, run low-power subset before full matrix, and capture telemetry plus operator observations for live pose containment and abort behavior.
- **Evidence rule:** every gate records command, result, commit/build identity, and telemetry screenshot/log where applicable. A source check passing does not replace hardware evidence for physical motion or motor shutdown.

## Automated Checks

Run from repository root with the Windows Gradle wrapper:

```text
gradlew.bat :TeamCode:compileDebugJavaWithJavac
```

Expected: compile succeeds with no source changes outside planned Phase 6 OpMode.

Run source-level checks against the implementation file. Replace `<opmode>` with the actual path if implementation uses a different planned filename:

```text
rg -n "SAFE|BOUND|25|clamp|goToPosition" TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/<opmode>
rg -n "localizer\.update\(\)|drive\.update\(\)" TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/<opmode>
rg -n "drive\.stop\(\)|finally|isStopRequested|opModeIsActive|timeout" TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/<opmode>
rg -n "telemetry\.add(Data|Line)|telemetry\.update\(\)" TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/<opmode>
! rg -n "File|FileOutputStream|FileWriter|SharedPreferences|Preferences|OkHttp|HttpURLConnection|Socket|JSONObject" TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/<opmode>
```

Expected:

- X/Y command path clamps immediately before `goToPosition`.
- Interior bound is strictly inside ±25 cm; no generated target can exceed configured safe margin.
- `localizer.update()` appears before `drive.update()` in each control loop.
- `drive.stop()` covers normal completion, timeout/abort paths, and unconditional `finally` cleanup.
- Telemetry exposes target, unclamped/clamped status, pose, errors, power, state, settle/timeout, and best result.
- No file, preference, network, or automatic persistence output exists.

## Requirement Checks

| ID | Behavior | Executable check | Pass evidence | Failure threshold |
|---|---|---|---|---|
| PH6-01 | Deterministic matrix covers many trials | Compile plus inspect matrix definitions/entries | Matrix count and listed trial categories recorded | Any missing cardinal, diagonal, combined X/Y/heading, or heading sign category = fail |
| PH6-02 | Target clamping and bounded commands | Execute clamp helper/check if exposed; otherwise source check every `goToPosition` path | All commanded X/Y values in `[-safeHalfSquare, +safeHalfSquare]`, with `safeHalfSquare < 25.0` | Any non-finite, unclamped, or > ±25 cm target = fail |
| PH6-03 | Correct trial lifecycle and update order | Source check plus compile | Per trial: stop, reset pose/heading, gains, command; loop order Localizer then drive; measurable settle/timeout metrics | Reversed update order, missing reset, or no settle/timeout outcome = fail |
| PH6-04 | Telemetry-only measurable results | Compile/source check; inspect runtime telemetry | Current trial, gains, target, clamp, live pose, X/Y/heading errors, power, state, elapsed time, settle/timeout, best result visible | Missing required metric or any persistence/network API = fail |
| PH6-05 | Safe shutdown | Source check; hardware stop-request and timeout trials | Motors stop within one control-loop observation after abort/timeout and remain stopped | Continued nonzero motor power after abort/timeout, or missing `finally` stop = fail |
| PH6-06 | Hardware-approved bounded execution | Manual live-pose protocol below | Short subset and full matrix pass boundary, heading, shutdown, and telemetry checks | Any uncontained pose, wrong heading sign, unsafe abort, or missing evidence = block approval |

## Matrix Coverage Evidence

Record actual matrix entries from implementation, not intended categories. Minimum required coverage:

- Cardinal translation: `(+X,0,0)`, `(-X,0,0)`, `(0,+Y,0)`, `(0,-Y,0)`.
- Diagonal translation: at least one `(+X,+Y)` and one mixed-sign diagonal.
- Heading-only: positive and negative heading targets.
- Combined pose: nonzero X, Y, and heading in at least two sign combinations.
- Gain candidates: more than one candidate, with deterministic ordering.

Acceptance: all categories execute in matrix order; no duplicate-only matrix that claims broader coverage.

## Update Order Check

Static evidence must show each active trial loop performs:

1. `localizer.update()`
2. live boundary/pose measurement as applicable
3. `drive.update()`
4. error/settle telemetry publication

Acceptance: no `drive.update()` occurs before the corresponding Localizer update. Record source line numbers in validation notes.

## Safe Shutdown Check

Run or simulate these exits on hardware:

1. **Operator stop:** press controller stop/request stop during active motion.
2. **Timeout:** use a controlled trial that cannot settle within configured timeout, if test configuration permits.
3. **Boundary abort:** trigger configured live boundary threshold using a safe, supervised approach; do not intentionally drive outside the physical square.
4. **Normal completion:** allow one trial and full matrix completion.

For each exit record telemetry state, elapsed time, final errors, boundary/timeout flag, and motor-power telemetry immediately after exit. Acceptance requires `drive.stop()` execution and zero commanded motor power after cleanup. If hardware cannot safely induce timeout or boundary abort, use source evidence plus a documented dry-run limitation; do not mark PH6-06 approved.

## Manual Live-Pose Boundary Protocol

### Preflight

- Place robot center at documented reset origin, centered within physical 50 × 50 cm square.
- Mark or measure the physical boundary. Keep robot footprint, cables, and obstacles clear.
- Use low-power limit from implementation; reject run if telemetry does not show configured low-power limit.
- Assign spotter with physical emergency stop/controller ready.
- Verify configured `leftfront`, `rightfront`, `leftback`, `rightback`, IMU, and odometry devices.
- Start short subset only. Full matrix requires clean subset result.

### Measured thresholds

- **Command threshold:** every target X/Y must satisfy `abs(target) <= safeHalfSquare`, and `safeHalfSquare < 25.0 cm`.
- **Live warning threshold:** abort before pose reaches physical boundary. Record configured warning/abort threshold and ensure it leaves clearance for robot footprint and sensor error.
- **Abort evidence threshold:** on boundary approach, stop command must be visible within one control-loop update; motor power must read zero after `drive.stop()`.
- **Heading evidence:** positive target produces counter-clockwise rotation; negative target produces clockwise rotation. Record observed direction for each.
- **Settle evidence:** a trial counts settled only after configured consecutive in-tolerance loops/window, not one sample.
- **Timeout evidence:** timeout flag, elapsed duration, final translational/heading errors, and stopped power must all be present.

Do not invent tolerance values during review. Record implementation constants and measured telemetry values. If no live boundary threshold exists, classify PH6-06 as BLOCKED and escalate implementation gap.

### Run record

For each short-subset and full-matrix run, record:

- date/time, build identity, OpMode name;
- matrix size and trial IDs/categories;
- safe margin, outer bound, power limit, timeout, settle threshold/window;
- maximum observed `abs(X)` and `abs(Y)` live pose;
- closest boundary distance and whether warning/abort fired;
- positive/negative heading direction observations;
- stop-request, timeout, boundary-abort, and normal-completion results;
- telemetry-only confirmation: no files, preferences, network calls, or automatic gain changes;
- operator and spotter approval or issue description.

Acceptance requires maximum live pose remain inside physical boundary with documented clearance, all safety exits stop motors, all required matrix categories execute, and telemetry supports manual comparison.

## Validation Status Template

| Gate | Status | Evidence |
|---|---|---|
| TeamCode compile | pending | command output/build ID |
| Target clamping | pending | source/executable output; min/max command values |
| Matrix coverage | pending | trial IDs and category count |
| Update order | pending | source line references |
| Safe shutdown | pending | stop/timeout/abort telemetry |
| Telemetry-only output | pending | source scan and runtime observation |
| Manual live-pose containment | pending | max pose, clearance, threshold, abort evidence |

Phase 6 is **approved only when all gates pass**. Any failed or unexecuted hardware gate remains blocked, not inferred from compilation.
