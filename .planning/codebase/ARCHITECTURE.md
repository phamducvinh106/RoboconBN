# Architecture

## Runtime layers

1. FTC Android runtime and SDK modules provide OpMode lifecycle, hardware access, telemetry, and camera APIs.
2. `TeamCode/core` contains reusable robot subsystems: drive, odometry, PID, and vision.
3. `TeamCode/opmode` contains FTC entry points and manual camera test flows.
4. `TeamCode/test` contains offline behavioral models/checks.

## Control flow

OpMode loop calls subsystem update/read methods. Odometry samples encoder/IMU deltas and integrates global pose. Mecanum drive runs three PID controllers, converts robot-frame commands to wheel powers, and manages movement/script states. Camera frames enter OpenCV pipeline asynchronously; pipeline publishes immutable-ish detection snapshots through `AtomicReference`.

## TemplateMatchCamera current design

- One `TemplateData` is active at any time.
- `matchTemplate(..., TM_CCOEFF_NORMED)` runs on grayscale, downscaled frame/template; confidence threshold is `0.55`.
- Detection emits label, center, pixel offsets, distance, confidence, and corners.
- EMA/deadband/outlier confirmation/miss hold smooth output and reject spikes.
- `setTarget()` clears tracking state, releases old Mats, loads one new asset, and changes label.
- `stop()` stops stream, closes device, releases pipeline Mats.

## Planned modes

`SINGLE_TARGET` fits current design: retain one loaded template, lowest processing and simplest state semantics. `MULTI_TARGET` needs a mode contract, collection of templates, per-template match results, cross-label selection policy, and result schema for multiple detections. Existing single `CameraResult.detection` and label-based filter are insufficient without deciding whether output means best target, all targets, or selected target. Multi-mode also raises CPU/memory, template-switch/reset, duplicate suppression, confidence comparability, and temporal tracking questions.
