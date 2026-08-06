# Requirements: RoboconBN O2 Robot Vision and Autonomous Control

**Defined:** 2026-08-06
**Core Value:** Robot identifies the right cargo and centers the fork accurately enough to lift pallet safely, then places it in the correct factory area.

## v1 Requirements

### Localizer

- [ ] **LOC-01**: `Localizer` uses `rightfront` as strafe encoder and `leftfront` as forward encoder.
- [ ] **LOC-02**: `Localizer` exposes calibrated wheel diameter, ticks per revolution, encoder signs, pod offsets, scale, and deadbands.
- [ ] **LOC-03**: IMU initializes with `LogoFacingDirection.BACKWARD` and `UsbFacingDirection.UP`, and measured yaw direction matches heading sign.
- [ ] **LOC-04**: Forward, strafe, rotation, and combined movement produce correct local-to-global pose signs and magnitudes.
- [ ] **LOC-05**: Update, angle wrapping, pose reset, and offset suggestion calculations are verified with repeatable calibration tests.

### Vision Modes

- [ ] **VIS-01**: `TemplateMatchCamera` exposes explicit `SINGLE_TARGET` and `MULTI_TARGET` modes without duplicating camera lifecycle code.
- [ ] **VIS-02**: `SINGLE_TARGET` tracks the left pallet within the one-block FOV and minimizes camera-to-left-pallet center error fast enough for lateral centering.
- [ ] **VIS-03**: `SINGLE_TARGET` exposes target center, `dxPx`, `dyPx`, confidence, validity, and stale-result age for fork alignment.
- [ ] **VIS-04**: `MULTI_TARGET` classifies the right and left blocks during scan passes, returning one type label and confidence for each block before centering.
- [ ] **VIS-05**: `MULTI_TARGET` extracts threshold-qualified candidates and suppresses overlapping duplicate boxes before ranking.
- [ ] **VIS-06**: Both modes reject invalid templates/frames and keep camera thread safe under start, stop, and failure paths.
- [ ] **VIS-07**: Vision thresholds, active-level crop/region-of-interest, NMS/min-distance, temporal hold, and center policy are measurable constants, not hidden magic behavior.

### Mechanism and Sensors

- [ ] **MECH-01**: Elevator homes toward `endstop1`, stops immediately on activation, and resets logical step position.
- [ ] **MECH-02**: Stepper motion uses `step` pulses and `dir` direction with calibrated travel bounds and timeout.
- [ ] **MECH-03**: Both fork servos support `PLACE` parallel to field and `HOLD` perpendicular to field.
- [ ] **MECH-04**: `leftIR` and `rightIR` confirm cargo-ready position with debounce; they do not classify block type.
- [ ] **MECH-05**: Pickup sequence uses MULTI_TARGET classifications, then SINGLE_TARGET centers the camera on the left pallet; mechanical coupling centers the right fork on the right pallet before forward approach, dual IR confirmation, one-cycle pickup, and lift.

### Autonomous Flow

- [ ] **AUTO-01**: Robot performs right scan → left scan → center both blocks → forward approach → IR confirm → one-cycle dual pickup → transport → place as bounded states.
- [ ] **AUTO-02**: Block types 01–04 route to Samsung, Foxconn, Amkor, and Hana Micron Vina respectively.
- [ ] **AUTO-03**: Placement stops with pallet parallel to field and fully inside target 250 x 250 mm factory area.
- [ ] **AUTO-04**: Task 2 cannot start until task 1 completes 100%.
- [ ] **AUTO-05**: All movement states handle timeout, stop request, missing detection, failed IR confirmation, jam, and safe stop.

### Verification

- [ ] **TEST-01**: Offline tests cover mode selection, center math, candidate ranking, NMS/min-distance, confidence thresholds, and stale results.
- [ ] **TEST-02**: Hardware test OpMode verifies exact device names, directions, encoder signs, servo states, IR polarity, endstop, and stepper pulses.
- [ ] **TEST-03**: Field test covers randomized shelf positions, all block classes, failed pickup, reset, placement, and 240-second budget.

## v2 Requirements

- **ADV-01**: Fuse odometry with IMU or absolute field references.
- **ADV-02**: Add adaptive template scale/rotation/lighting compensation.
- **ADV-03**: Learn route ordering from measured travel time and remaining cargo.

## Out of Scope

| Feature | Reason |
|---------|--------|
| AprilTag localization | Not requested; retain odometry and OpenCV scope |
| Deep-learning detector | Adds dependency and tuning burden; no current requirement |
| Separate `SingleTargetCamera`/`MultiTargetCamera` classes | One shared `TemplateMatchCamera` with modes prevents lifecycle duplication |
| Automatic task 2 before task 1 completion | Forbidden by O2 rules |

## Traceability

| Requirement | Phase | Status |
|-------------|-------|--------|
| LOC-01–LOC-05 | Phase 1 | Pending |
| VIS-01–VIS-07 | Phase 2 | Pending |
| MECH-01–MECH-05 | Phase 3 | Pending |
| AUTO-01–AUTO-05 | Phase 4 | Pending |
| TEST-01–TEST-03 | Phase 5 | Pending |

**Coverage:** 25 v1 requirements; 25 mapped; 0 unmapped.

---
*Requirements defined: 2026-08-06*
