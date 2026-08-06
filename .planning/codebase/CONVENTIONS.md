# Conventions

- Java package root: `org.firstinspires.ftc.teamcode`; reusable code under `core`.
- Classes are mostly `final`; fields/constants use descriptive names and uppercase `static final` constants.
- Hardware-facing classes provide production constructors using `HardwareMap`; offline/test constructors avoid FTC hardware where practical.
- Subsystems expose small imperative APIs (`update`, `reset`, getters) and keep tuning constants near class top.
- FTC lifecycle uses `startAsync()`/`stop()` for camera and `try/finally` cleanup in OpModes.
- Detection and drive behavior expose telemetry/debug getters rather than logging-heavy internals.
- Comments and user-facing documentation are mixed Vietnamese/English.
- Current tests duplicate key constants/logic instead of importing private pipeline internals; changes to filter constants require synchronized test edits.
