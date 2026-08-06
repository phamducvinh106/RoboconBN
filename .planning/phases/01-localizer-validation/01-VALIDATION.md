# Phase 1 Validation

## Validation Architecture

- Offline checks: executable `LocalizerCalibrationTest` and `LocalizerMathTest`; compile/run commands are declared in the plans.
- Build gate: `gradlew.bat :TeamCode:assembleDebug` after each task; `gradlew.bat assembleDebug` at wave/phase gate.
- Hardware gate: bounded calibration OpMode stages with low power, stop request handling, timeout, frozen-sensor rejection, and unconditional drive stop.
- Evidence: record exact `rightfront`/`leftfront` mapping, calibration constants, IMU orientation/yaw direction, primitive and combined pose errors, reset/wrap behavior, and offset repeatability.

## Acceptance Tolerances

- Offline fixture assertions use explicit tolerances documented in test source.
- Hardware stages declare tolerances in telemetry before motion; failed stages require rerun after a targeted correction.
- No measured constant is accepted from a single rotation or silently written into source.
