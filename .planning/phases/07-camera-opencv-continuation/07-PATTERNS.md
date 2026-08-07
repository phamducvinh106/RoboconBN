# Phase 7 Patterns

- Core classes use final public APIs, nested immutable result/config types, and `HardwareMap` name lookup.
- Camera lifecycle uses `OpenCvCameraFactory`, async open callbacks, `OpenCvWebcam`, `startStreaming(640, 480, UPRIGHT)`, and guarded stop/close.
- Results use `AtomicReference`/volatile telemetry for pipeline-to-opmode handoff.
- OpModes use `try/finally` to stop drive and camera, inspect timestamp age, and show camera/error telemetry.
- Offline checks are executable Java classes under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test`, with no JUnit dependency.
- Hardware names are case-sensitive and must be centralized in explicit fields or constants where possible.
- Preserve existing source compatibility where practical; avoid introducing a second camera abstraction.
