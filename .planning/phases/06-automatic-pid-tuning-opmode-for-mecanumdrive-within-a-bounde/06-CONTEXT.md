# Phase 6: Automatic PID tuning OpMode for MecanumDrive within a bounded 50 x 50 cm square - Context

**Gathered:** 2026-08-06
**Status:** Ready for planning

<domain>
## Phase Boundary

Create one FTC OpMode that runs multiple automatic MecanumDrive PID tuning trials inside a 50 x 50 cm field square. Trials vary translation directions, target combinations, and headings. Results remain telemetry-only; no file persistence or automatic constant changes.

</domain>

<decisions>
## Implementation Decisions

### Test matrix
- **D-01:** Run many different test cases, not one tuning maneuver.
- **D-02:** Cover multiple translation directions, multiple headings, and combined X/Y/heading targets.
- **D-03:** Use targets clamped to the permitted 50 x 50 cm square; selected targets must not command outside it.

### Safety and output
- **D-04:** When a target or generated motion approaches/exceeds the boundary, clamp it and continue the tuning sequence.
- **D-05:** Keep heading convention: positive heading means counter-clockwise, negative means clockwise.
- **D-06:** Display measured results through FTC telemetry only. Do not write gains or results to files.

### Claude's Discretion
- Trial ordering, scoring formula, settle detection, gain sweep strategy, telemetry fields, and low-power limits.
- Exact safe margin inside the 50 cm square, provided no generated target exceeds the boundary.

</decisions>

<canonical_refs>
## Canonical References

### Existing project
- `.planning/ROADMAP.md` — Phase 6 scope and project roadmap.
- `.planning/REQUIREMENTS.md` — project requirements and traceability.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/MecanumDrive.java` — current drive API and PID controls.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/Localizer.java` — pose, heading, and reset contract.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/RobotHardware.java` — hardware initialization contract.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/MecanumDriveGamepadTargetTestOpMode.java` — existing bounded manual target test pattern.

</canonical_refs>

<code_context>
## Existing Code Insights

### Reusable Assets
- `MecanumDrive.setPositionGains()` and `setHeadingGains()` expose runtime tuning.
- `MecanumDrive.setPowerLimits()`, `goToPosition()`, `update()`, `getState()`, and error getters support trial execution.
- `Localizer.resetPoseAndHeading()` establishes trial origin.
- `RobotHardware` supplies named motors and Localizer.

### Established Patterns
- LinearOpModes initialize `RobotHardware`, construct `MecanumDrive`, update Localizer before drive, and stop drive in `finally`.
- Existing tests use `A/B/X/Y` edge-triggered buttons and telemetry for pose/power/state.

### Integration Points
- New OpMode belongs under `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/`.
- It must not modify Localizer signs, offsets, or MecanumDrive behavior as part of tuning.

</code_context>

<specifics>
## Specific Ideas

- Tune inside a physically bounded 50 × 50 cm square.
- Test combinations rather than only forward motion.
- Keep output visible to operator through telemetry.

</specifics>

<deferred>
## Deferred Ideas

None — discussion stayed within phase scope.

</deferred>

---

*Phase: 6-automatic-pid-tuning-opmode-for-mecanumdrive-within-a-bounde*
*Context gathered: 2026-08-06*
