---
phase: 02-two-mode-opencv-camera
plan: 01
subsystem: vision
tags: [opencv, camera, telemetry]
requires: []
provides: [camera-mode-contract, measurable-camera-config, stale-result-telemetry]
affects: [TemplateMatchCamera, camera-test-opmodes]
tech-stack:
  added: []
  patterns: [shared-lifecycle, immutable-result]
key-files:
  created: []
  modified:
    - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/TemplateMatchCamera.java
    - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/TemplateMatchSingleTest.java
    - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/TemplateMatchTest.java
decisions:
  - Preserve one TemplateMatchCamera lifecycle and expose mode/config through its API.
metrics:
  duration: 15m
  completed: 2026-08-06
status: complete
---

# Phase 02 Plan 01: Shared camera contracts Summary

Shared `SINGLE_TARGET`/`MULTI_TARGET` mode selection, measurable configuration, immutable result fields, and mode-aware telemetry added without new dependencies or camera wrappers.

## Tasks Completed

- Added `CameraMode`, immutable `CameraConfig`, mode-aware constructor, and policy accessors.
- Added `centerX`, `centerY`, raw `confidence`, and `staleAgeMs` to `CameraResult`.
- Updated both OpModes to select explicit mode and consume result freshness directly.
- Added policy and lifecycle telemetry.

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 3 - Blocking issue] Fixed default configuration constructor argument count**
- **Found during:** Task 1 verification
- **Fix:** Corrected default `CameraConfig` delegation.

## Verification

- `./gradlew.bat :TeamCode:compileDebugJavaWithJavac` — passed with JDK 17.
- Offline executable test could not run because Gradle does not emit `TemplateMatchCameraTest.class` into `TeamCode/build/classes/java/main`; command returned `ClassNotFoundException`.
- Repository commit unavailable: no `.git` repository detected.

## Known Stubs

None introduced.

## Self-Check: PASSED

Source files exist and compile successfully. No commit hash available because repository is not initialized as Git.
