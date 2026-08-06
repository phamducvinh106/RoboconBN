# Integrations

- FTC `HardwareMap` binds named `DcMotorEx`, `IMU`, and `WebcamName` devices at runtime.
- `MecanumDrive` controls four motors and consumes `TwoWheelOdometry` through `OdometryProvider`.
- `TwoWheelOdometry` combines two encoder pods with REV IMU yaw; hardware orientation is constructor-configured.
- `TemplateMatchCamera` owns an `OpenCvWebcam`, asynchronous open/stream lifecycle, and `OpenCvPipeline` processing.
- Camera loads PNG/JPG-like assets from Android `assets`; current repository includes `TeamCode/src/main/assets/target.png`.
- Camera output is pull-based: OpMode reads atomic latest `CameraResult`; telemetry exposes camera, detection, filtering, and performance state.
- `TemplateMatchTest` supplies four target names, but current camera matches only one loaded template at a time through `setTarget()`.
- `teamwebcamcalibrations.xml` provides webcam calibration integration; camera name currently expected as `Webcam 1` in test OpModes.
