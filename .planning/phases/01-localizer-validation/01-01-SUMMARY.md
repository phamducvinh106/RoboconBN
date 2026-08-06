---
phase: 01-localizer-validation
plan: 01
subsystem: localizer
status: complete
tags: [odometry, imu, calibration, safety]
requires: []
provides: [calibration-contract, bounded-calibration-telemetry, offline-guards]
affects: [TeamCode localizer]
tech-stack:
  added: []
  patterns: [read-only calibration metadata, finally cleanup]
key-files:
  created: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/LocalizerCalibrationTest.java]
  modified: [TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/Localizer.java, TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/RobotHardware.java, TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/OdometryCalibrationOpMode.java]
decisions:
  - Keep existing Localizer constants unchanged; telemetry records measurements before tuning.
metrics:
  duration: unknown
  completed: 2026-08-06
---

# Phase 1 Plan 1: Calibration telemetry and safe bounded motion Summary

Read-only calibration contract exposes scale, signs, offsets, deadbands, mapping, and IMU orientation. Calibration OpMode now uses low-power bounded yaw with timeout, frozen-sensor detection, stop-request checks, telemetry, and unconditional `MecanumDrive.stop()` cleanup.

## Tasks Completed

- Added validated `Localizer.Calibration` contract and executable offline guards.
- Documented `RobotHardware` mapping: `rightfront` strafe/perpendicular, `leftfront` forward/parallel.
- Replaced unbounded 360-degree motion loop with bounded, telemetry-rich calibration motion.
- Updated plan task markers.

## Verification

- Offline Java check: not run; command runner rejected directory-creation command.
- `gradlew.bat :TeamCode:assembleDebug`: not run; command runner allowlist rejected Windows wrapper.
- Linter: only existing classpath warnings; no syntax diagnostics reported.
- Hardware calibration: blocked; physical robot unavailable.

## Deviations from Plan

### Auto-fixed Issues

None.

## Deferred Issues

- Run offline Java check and Gradle build from a normal Windows shell.
- Execute physical yaw, encoder sign, scale, and repeated offset tests on secured robot.

## Known Stubs

None.

## Threat Flags

None.
