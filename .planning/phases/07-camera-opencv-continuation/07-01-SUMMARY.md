---
phase: 07-camera-opencv-continuation
plan: 01
subsystem: vision
status: complete
tags: [opencv, orb, lifecycle, webcam]
dependency_graph:
  requires: [EasyOpenCV, OpenCV, FTC HardwareMap]
  provides: [shared explicit camera contract, webcam1 centering policy, webcam1/webcam2 classification policy]
  affects: [camera test OpModes, RobotHardware]
tech_stack:
  added: []
  patterns: [ORB descriptors, immutable AtomicReference result, generation-guarded lifecycle]
key_files:
  created: []
  modified:
    - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/ColorContourCamera.java
    - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/RobotHardware.java
    - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/LeftCameraCenteringTestOpMode.java
    - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/LeftColorCenteringTestOpMode.java
    - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/RightCameraClassificationTestOpMode.java
    - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/CameraOdometryMain.java
decisions:
  - Keep one ColorContourCamera lifecycle owner and explicit CameraId/Mode policy.
  - Require webcam2 HardwareMap lookup; never alias webcam1.
  - Keep UART, packed 20-bit transport, I2C, HSV/YCrCb, and contour processing out of this plan.
metrics:
  duration: 8 min
  completed_date: 2026-08-08
---
# Phase 07 Plan 01: Shared ORB Camera Contract Summary

ORB/template-only camera lifecycle now exposes explicit `SINGLE_TARGET` and `MULTI_TARGET` policies. `SINGLE_TARGET` is rejected for webcam2; both named webcams can classify configured templates, while only webcam1 exposes centering movement authority. Async generation guards, idempotent start/stop, invalidation on error/stop, freshness constants, deterministic ranking, and duplicate suppression protect the result handoff.

## Tasks Completed

1. Hardened shared ORB/template camera contract — `bb22dc5`
2. Wired explicit camera modes and required webcam2 — `2fc5d92`

## Verification

- `./gradlew.bat :TeamCode:compileDebugJavaWithJavac` — passed.
- Linter diagnostics — no remaining syntax errors in edited camera source; classpath warnings only.
- No production DigitalUartRx, packed 20-bit decoder, I2C wiring, or UART wiring added.

## Deviations from Plan

### Auto-fixed Issues

1. Added compatibility `contourCount` result field because existing telemetry consumers referenced it; value now reports retained candidate count, not contours.
2. Corrected static pipeline lifecycle-state references discovered by compile.

## Known Stubs

- Default constructor receives an empty template configuration because repository has no committed template asset set. Production classification requires callers to pass configured `ClassConfig[]` templates; no fake labels or fallback detections were added.

## Self-Check: PASSED

- Modified source files exist.
- Task commits `bb22dc5` and `2fc5d92` exist.
- Build passed.
