---
phase: 02-two-mode-opencv-camera
plan: 02
subsystem: vision
tags: [opencv, multi-target, nms, telemetry]
requires: [02-01]
provides: [candidate-suppression, multi-result-contract, scan-telemetry]
affects: [TemplateMatchCamera, TemplateMatchCameraTest, TemplateMatchTest]
tech-stack:
  added: []
  patterns: [immutable-candidate, deterministic-greedy-suppression]
key-files:
  created: []
  modified:
    - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/TemplateMatchCamera.java
    - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/TemplateMatchCameraTest.java
    - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/TemplateMatchTest.java
decisions:
  - Preserve immutable candidate results and raw confidence through suppression.
metrics:
  duration: partial
  completed: 2026-08-06
status: partial
---

# Phase 02 Plan 02: Multi-target suppression Summary

Added immutable candidate geometry, deterministic confidence/order sorting, IoU and minimum-center-distance suppression, multi-result list compatibility, executable suppression assertions, and retained-candidate telemetry.

## Verification

- `gradlew.bat :TeamCode:compileDebugJavaWithJavac` — passed with JDK 17.
- Offline executable command — blocked because Gradle does not emit `TemplateMatchCameraTest.class` into `TeamCode/build/classes/java/main`; returned `ClassNotFoundException`.
- Git commit — unavailable; repository has no `.git` directory.

## Deviations from Plan

### Deferred Blocking Work

Runtime multi-template candidate extraction and publication remains incomplete because current implementation still owns one loaded template and one detection path. No commit made due to repository not being Git initialized.

## Known Stubs

- `CameraResult.detections` currently wraps single `detection`; full multi-template extraction remains for continuation.

## Self-Check: PASSED

Modified source compiles successfully. Git commit unavailable by repository state.
