# Phase 2 Validation Contract

## Execution order

Validation is blocking and ordered: (1) supervised hardware communication test for stepper, servos, IR, endstop, encoder/localizer, release/back-out, and Raspberry Pi 5 UART/camera-manager placeholder communication for both explicit camera channels, (2) strict centralized JSON load, (3) manager-backed state-machine tests and compile, (4) placement wiring, one-cycle, and six-cycle acceptance. No downstream gate substitutes for missing UART/camera communication evidence. UART frame parsing and OpenCV/template matching are not Phase 2 gates.

## Manager coverage

Each physical subsystem has separate manager evidence, including `CameraAdapterManager` with a Raspberry Pi 5 UART transport placeholder and two explicit camera channels/identities (`webcam1`, `webcam2`). Tests use injected fake channels/clocks/providers. Telemetry records camera identity/channel, transport status, frame validity, freshness, polarity, direction, raw/derived readings, pulse state, pose validity, release state, and config fingerprint. Invalid/incomplete placeholder data must never authorize movement.

## Offline commands

```text
javac -d build/phase2-test TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/LiftingHardwareManagerTest.java
java -cp build/phase2-test org.firstinspires.ftc.teamcode.test.LiftingHardwareManagerTest
javac -d build/phase2-config-test TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/LiftingSequenceConfig.java TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/LiftingSequenceConfigLoader.java TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/LiftingSequenceConfigTest.java
java -cp build/phase2-config-test org.firstinspires.ftc.teamcode.test.LiftingSequenceConfigTest
gradlew.bat :TeamCode:compileDebugJavaWithJavac
```

## Locked gates

- D-01: one versioned JSON asset contains all runtime tuning; missing, malformed, wrong-version, incomplete, non-finite, or out-of-range config enters `SAFE_STOP`; no defaults.
- D-02: every movement transition requires fresh finite Localizer/encoder state, target tolerance, position/heading error, settle cycles, encoder validity, and no-progress detection. Applies to `MOVE_TO_SHELF`, `CENTER`, `APPROACH`, `BACK_OUT`, `MOVE_NEAR_FACTORY`, `MOVE_TO_PLACEMENT_POSITION`, and `BACK_OUT_AFTER_RELEASE`.
- D-03/D-04/D-06/D-07: both explicit camera channels use Pi5 UART placeholder contracts; validity/freshness gates movement, invalid/incomplete data cannot authorize movement, and communication evidence precedes JSON/state integration.
- D-05: no OpenCV/template matching or UART frame parser implementation in Phase 2.
- Encoder-confirmed arrival gates remain required. Error recovery policy remains deferred; do not add recovery behavior beyond explicit invalid-safety stopping.

## Hardware acceptance

Run `LiftingHardwareTestOpMode` first at low power. Verify exact names (`step`, `dir`, `endstop1`, `servoLeft`, `servoRight`, `leftIR`, `rightIR`, `webcam1`, `webcam2`, odometry encoders), active-low behavior, step-low-before-dir, travel bounds, PLACE/HOLD, camera freshness, finite pose, release/back-out readings, and telemetry. Record config version/fingerprint and localized JSON edits.

Only after manager evidence passes: run one supervised cycle, then `3 shelves × 2 levels = 6 cycles`, `12 blocks`. Confirm left placement fully releases and backs out 20 cm before right starts, right re-establishes required height, and no cycle advances before `CYCLE_COMPLETE`.

## Evidence status

| Gate | Status | Evidence |
|---|---|---|
| Manager offline checks | pending | test output |
| Hardware communication OpMode | pending | telemetry and operator record |
| JSON fixture checks | pending | fixture output and fingerprint |
| State-machine checks | pending | transition/failure output |
| One-cycle acceptance | blocked until above pass | config fingerprint and telemetry |
| Six-cycle acceptance | blocked until one-cycle pass | operator/spotter record |
