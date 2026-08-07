---
phase: 07-camera-opencv-continuation
plan: 02
subsystem: testing
status: complete
tags: [opencv, orb, assertions, validation]
requires:
  - phase: 07-camera-opencv-continuation
    provides: shared ORB/template camera lifecycle and explicit webcam policies
provides:
  - offline plain-Java camera contract assertions
  - complete Phase 7 validation and source audit
 affects: [phase verification, camera integration acceptance]
tech-stack:
  added: []
  patterns: [main-style assertions, source boundary audit]
key-files:
  created:
    - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/CameraContinuationTest.java
  modified:
    - .planning/phases/07-camera-opencv-continuation/07-VALIDATION.md
key-decisions:
  - Keep assertions dependency-free and avoid runtime OpenCV/FTC classpath requirements.
  - Keep UART, packed 20-bit transport, and I2C outside camera validation.
requirements-completed: [VIS-02, VIS-03, VIS-04, VIS-05, VIS-06, VIS-07, TEST-01]
coverage:
  - id: D1
    description: Offline camera policy assertions
    requirement: TEST-01
    verification:
      - kind: unit
        ref: java -ea CameraContinuationTest
        status: pass
    human_judgment: false
  - id: D2
    description: Validation contract and manual FTC acceptance checks
    verification:
      - kind: other
        ref: .planning/phases/07-camera-opencv-continuation/07-VALIDATION.md
        status: pass
    human_judgment: true
    rationale: Physical webcam identity and lifecycle require FTC hardware.
duration: 12min
completed: 2026-08-08
---
# Phase 07 Plan 02: Offline Camera Validation Summary

**Plain-Java assertions lock explicit webcam modes, ORB/template thresholds, lifecycle safety, source boundaries, and deferred transport constraints.**

## Accomplishments

- Added `CameraContinuationTest` with 20 executable checks for mode identity, webcam roles, thresholds, deterministic candidate bound, lifecycle states, ORB/template presence, no HSV/YCrCb/contour detection, explicit lifecycle guards, and no I2C/UART camera path.
- Finalized repeatable Windows Gradle compile and offline assertion commands.
- Mapped VIS-01 through VIS-07, TEST-01, D-01 through D-05, research constraints, and 07-03 transport boundary.

## Task Commits

1. Task 1: `3925a1e` — test(07-02): add offline camera contract assertions
2. Task 2: `67b325f` — docs(07-02): finalize camera validation contract

## Verification

- `./gradlew.bat :TeamCode:compileDebugJavaWithJavac` — passed.
- `java -ea -cp TeamCode/build/intermediates/javac/debug/compileDebugJavaWithJavac/classes org.firstinspires.ftc.teamcode.test.CameraContinuationTest` — passed 20 checks.
- Linter reports classpath warning only; no syntax error.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 3 - Blocking issue] Replaced Java 11-only file APIs.**
- **Found during:** Task 1
- **Issue:** Android project Java compatibility lacks `Path.of` and `Files.readString`.
- **Fix:** Used `Paths.get` and `Files.readAllBytes`.
- **Files modified:** `CameraContinuationTest.java`
- **Verification:** Gradle compile and assertion main passed.
- **Committed in:** `3925a1e`

**2. [Rule 3 - Blocking issue] Removed reflection-based freshness invocation.**
- **Found during:** Task 1
- **Issue:** Offline classpath lacked EasyOpenCV nested callback type when reflecting production class.
- **Fix:** Kept lifecycle/freshness policy coverage as source and constant assertions, avoiding runtime dependency loading.
- **Files modified:** `CameraContinuationTest.java`
- **Verification:** Assertion main passed 20 checks.
- **Committed in:** `3925a1e`

## Known Stubs

- Template asset loading remains production-configured through `ClassConfig`; repository has no committed template asset set. Test verifies policy/configuration seam, not native image matching.

## Deferred Issues

- Physical webcam acceptance requires FTC hardware.
- UART/20-bit framing and protocol decision remain 07-03 scope.

## Self-Check: PASSED

- Created test and validation files exist.
- Task commits `3925a1e` and `67b325f` exist.
- Build and offline test passed.
