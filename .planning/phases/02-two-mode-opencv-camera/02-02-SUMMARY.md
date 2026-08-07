---
phase: 02-two-mode-opencv-camera
plan: 02
subsystem: hardware
tags: [ftc, hardware, telemetry, uart, camera, testing]
requires:
  - phase: 02-two-mode-opencv-camera
    provides: manager seams and Pi5 UART placeholder contracts from 02-01
provides:
  - supervised low-power manager communication OpMode
  - manager-specific offline assertions and ordered validation contract
affects: [02-03, 02-04, 02-05]
tech-stack:
  added: []
  patterns: [explicit operator actions, manager telemetry, finally cleanup]
key-files:
  created: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/LiftingHardwareTestOpMode.java]
  modified: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/LiftingHardwareManagerTest.java, .planning/phases/02-two-mode-opencv-camera/02-VALIDATION.md]
key-decisions:
  - Hardware communication evidence precedes JSON, state-machine, and placement work.
  - Pi5 UART parser/OpenCV remain deferred; invalid placeholder frames cannot authorize movement.
  - No timeout or recovery behavior added; operator controls each isolated check.
requirements-completed: [MECH-01, MECH-02, MECH-03, MECH-04, MECH-05, TEST-02]
coverage:
  - id: D1
    description: Offline manager checks cover stepper, fork, IR, pose, release, and both UART camera channels.
    requirement: TEST-02
    verification:
      - kind: unit
        ref: "java -cp build/phase2-test org.firstinspires.ftc.teamcode.test.LiftingHardwareManagerTest"
        status: pass
    human_judgment: false
  - id: D2
    description: Physical wiring, polarity, pulse timing, camera transport, and telemetry require supervised robot verification.
    requirement: MECH-01
    verification: []
    human_judgment: true
    rationale: Hardware readings and low-power motion cannot be verified offline.
duration: 15 min
completed: 2026-08-07
status: complete
---

# Phase 2 Plan 2: Hardware Communication Summary

**Supervised OpMode now exercises every hardware manager independently, reports wiring and safety observations, and blocks invalid Pi5 placeholder frames on both explicit camera channels.**

## Accomplishments
- Added `LiftingHardwareTestOpMode` with explicit operator actions for homing, bounded elevator movement, PLACE/HOLD, IR readings, pose, release/back-out, and both cameras.
- Added telemetry for exact device identities, active-low polarity, pulse state, finite pose, freshness, tuning values, and safe-stop behavior.
- Expanded offline assertions to cover fork, release/pose, and webcam1/webcam2 invalid placeholder safety.
- Reordered validation contract so supervised hardware communication is first; state machine and placement remain excluded.

## Task Commits
1. Task 1: Build supervised hardware communication OpMode — `505f5b1`
2. Task 2: Record hardware-first validation contract — `f481b66`

## Verification
- `javac ... && java ... LiftingHardwareManagerTest` — passed: `RESULT: 14 passed, 0 failed`.
- `gradlew.bat :TeamCode:compileDebugJavaWithJavac` — passed.
- IDE lint reports classpath warnings only; Gradle compile is clean.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 3 - Blocking issue] Corrected camera manager API call.**
- **Found during:** Task 1 compile
- **Issue:** OpMode called nonexistent `CameraAdapterManager.read`.
- **Fix:** Used existing `reading` seam.
- **Files modified:** `LiftingHardwareTestOpMode.java`
- **Verification:** Gradle compile passed.
- **Committed in:** `505f5b1`

**Total deviations:** 1 auto-fixed (Rule 3).
**Impact on plan:** No scope change; reused existing manager API.

## Known Stubs
- `Pi5UartCameraTransport` intentionally returns invalid placeholder frames; parser and vision remain deferred by D-03/D-05.
- JSON config version/fingerprint telemetry is labeled pending until Plan 02-03 adds strict external loading.

## Issues Encountered
- Physical hardware execution remains pending; operator must deploy at low power with spotter and record measured values.

## Next Phase Readiness
- Plan 02-03 can add strict JSON loading after hardware communication evidence.
- No state machine, placement, timeout, or recovery logic added.

## Self-Check: PASSED
- SUMMARY file exists.
- Task commits `505f5b1` and `f481b66` exist.
- Required source and test files exist.
