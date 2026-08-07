---
phase: 02-two-mode-opencv-camera
plan: 01
subsystem: hardware
status: complete
tags: [hardware, managers, uart, camera, telemetry]
dependency_graph:
  requires: [RobotHardware, Localizer]
  provides: [narrow hardware manager seams, Pi5 I2C camera placeholder]
  affects: [02-02, 02-03, 02-04, 02-05]
tech_stack:
  added: []
  patterns: [dependency injection, cooperative stepper, explicit camera channels]
key_files:
  created: [HardwareContracts.java, StepperElevatorManager.java, ForkServoManager.java, IrSensorManager.java, EndstopManager.java, EncoderLocalizerManager.java, CameraAdapterManager.java, Pi5I2cCameraDeviceManager.java, CameraFrameContract.java, CameraChannel.java, ReleaseBackoutSensorManager.java, LiftingHardwareManagerTest.java]
  modified: []
key_decisions:
  - Preserve no-timeout and deferred error-handling decisions; managers expose readings/actions only.
  - Keep Pi5 I2C parsing and OpenCV deferred; invalid placeholder frames cannot authorize movement.
metrics:
  duration: 12 min
  completed_date: 2026-08-07
---

# Phase 2 Plan 1: Hardware Manager Foundations Summary

Separate manager classes now isolate elevator, fork, IR, endstop, encoder/pose, camera, and release/back-out responsibilities. Injected channels and clocks support offline checks; camera exposes explicit `webcam1` and `webcam2` identities with validity/freshness gates.

## Verification

- `javac ... LiftingHardwareManagerTest.java` — passed.
- `java -cp build/phase2-test org.firstinspires.ftc.teamcode.test.LiftingHardwareManagerTest` — `RESULT: 11 passed, 0 failed`.
- `gradlew.bat :TeamCode:compileDebugJavaWithJavac` — passed.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 1 - Test bug] Corrected pulse-order assertion.**
- **Found during:** Task 2
- **Issue:** Test assumed exact log prefix despite initial homing writes.
- **Fix:** Assert required low-before-direction subsequence instead.
- **Files modified:** `LiftingHardwareManagerTest.java`

## Known Stubs

- `Pi5I2cCameraDeviceManager.java` intentionally returns invalid placeholder frames. I2C format/parser and vision remain deferred by D-03/D-05; later camera plans own integration.

## Lint

IDE reported classpath warnings only; standalone javac and Gradle compile passed.

## Self-Check: PASSED

All 12 planned source/test files exist; task commit recorded in git history.
