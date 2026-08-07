# Phase 7 Context: Camera/OpenCV Continuation

## Decisions

- **D-01**: Continue from `ColorContourCamera.java`; do not create or restore `TemplateMatchCamera.java` unless an implementation task proves an existing caller requires that compatibility name.
- **D-02**: Use one shared camera lifecycle and result contract for both modes.
- **D-03**: `SINGLE_TARGET` runs on `webcam1` for left-pallet centering; `MULTI_TARGET` runs on `webcam2` for classification. Camera selection must be explicit, not inferred from mode.
- **D-04**: Offline tests use plain Java assertions/main-style tests already used under `TeamCode/src/main/java/.../test`; no new dependency.
- **D-05**: Lifecycle safety includes idempotent start/stop, open/start failures, released pipeline resources, and stale-result rejection after stop or camera error.

## Assumptions

- Existing EasyOpenCV/OpenCV/FTC dependencies remain the implementation boundary.
- Existing `LeftCameraCenteringTestOpMode` remains the webcam1 integration reference; right classification coverage may be updated or replaced only as needed to use the shared contract.
- `RobotHardware` will expose explicit `webcam2` handling while preserving `webcam1` behavior; missing webcam2 should fail at construction with the normal FTC hardware-map error rather than silently aliasing cameras.
- Classification labels and threshold constants remain configurable through existing `ColorContourCamera.ClassConfig`; no asset or neural detector is introduced.
- Camera result timestamps are wall-clock milliseconds and consumers must treat invalid, closed, error, or over-age results as unusable.

## Deferred Ideas

- AprilTags, neural detection, adaptive template scale/rotation compensation, generic camera multiplexing, and field-tuning calibration are outside Phase 7.
