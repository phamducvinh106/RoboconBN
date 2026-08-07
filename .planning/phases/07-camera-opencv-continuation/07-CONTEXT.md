# Phase 7 Context: Single-Target ORB Accuracy Replan

## Decisions

- **D-01**: Improve only `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/SingleTargetCamera.java` for production vision behavior. Keep ORB/template matching and its existing camera lifecycle; do not merge into or modify another camera implementation.
- **D-02**: `OrbTarget1TestOpMode.java` and `CameraContinuationTest.java` are support files only: hardware telemetry/tuning and dependency-free deterministic checks. They do not become alternate production camera architectures.
- **D-03**: Preserve camera opening during FTC `INIT`, idempotent start/stop, generation-safe late callbacks, open/start failure handling, native resource release, and stale/error/closed fail-closed results.
- **D-04**: Offline tests use plain Java assertions/main-style tests under `TeamCode/src/main/java/.../test`; add no dependency and test production scalar policy rather than copied logic.
- **D-05**: Qualification order is cheap-to-expensive: bounded ORB work, absolute-plus-ratio match filtering, deterministic sorting and unique scene correspondences, spatial coverage, RANSAC, reprojection checks, projected geometry, then temporal publication.
- **D-06**: Temporal output requires coherent stable-frame acquisition, adaptive smoothing, adaptive continuity/outlier confirmation, bounded miss hold without timestamp refresh, stale rejection, and processing-budget rejection before movement authorization.
- **D-07**: Research thresholds are initial named defaults. Keep tuning constants near `SingleTargetCamera` using existing project convention; add runtime/config externalization only where an existing project pattern already supports it. Hardware evidence determines final values.

## Assumptions

- Existing FTC SDK, EasyOpenCV, OpenCV, Java standard library, `webcam1`, and `target1.png` remain the implementation boundary.
- ORB has already run on hardware; current problem is noisy single-target center output, not basic camera bring-up.
- If `SingleTargetCamera.java` is absent in the current checkout because prior work deleted it, recover its last repository baseline before applying this replan; do not reconstruct it from `OrbTemplateCamera`.
- Result timestamps use wall-clock milliseconds. Held output preserves the last observation timestamp and never becomes fresh by republishing.
- `OrbTarget1TestOpMode` remains an operator-only test path and must keep camera startup in `init()`.

## Deferred Ideas

- `TemplateMatchCamera`, `OrbTemplateCamera`, `FourTargetCameraOrchestrator`, multi-target classification, UART, I2C, packed transport, lifting, autonomous integration, AprilTags, neural detection, new camera architecture, and unrelated components are outside this replan.
