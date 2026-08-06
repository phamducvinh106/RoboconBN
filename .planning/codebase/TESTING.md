# Testing

- `MecanumDriveTest` is an offline executable simulation covering wheel kinematics, PID convergence/hold, script execution, and timeout.
- `TemplateMatchCameraTest` is an offline executable filter/state test covering jitter, smooth motion, spike rejection, miss hold, label stability, and `setTarget()` reset.
- OpenCV `matchTemplate` itself is not tested offline because tests avoid camera/OpenCV runtime.
- OpModes provide hardware-in-loop manual checks with live telemetry: camera state, template load errors, FPS, processing time, detection confidence, offsets, and filter counters.
- No JUnit/Gradle test source set or CI test command is documented in TeamCode.
- Run offline tests according to their class documentation in an environment with required compiled classes; validate camera behavior on robot controller hardware.
- For `MULTI_TARGET`, add deterministic tests for template ranking, per-label confidence, duplicate suppression, result ordering, target disappearance, and mode transitions before relying on it in autonomous control.
