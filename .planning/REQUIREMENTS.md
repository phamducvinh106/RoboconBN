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

### Lifting Sequence

- [ ] **LIFT-01**: State machine homes elevator, selects `READY1`/`READY2`, and returns to `HOME` using bounded `step`/`dir` pulses and `endstop1`.
- [ ] **LIFT-02**: Sequence controls `PLACE`/`HOLD`, scans and centers cargo, approaches only while both IR sensors are valid, and lifts the selected floor safely.
- [ ] **LIFT-03**: Fixed factory coordinates route by left-block class and place left then right cargo, using `READY1` to push existing cargo deeper.
- [ ] **LIFT-04**: Every state handles stop request, stale camera, failed IR, invalid/stale encoder state, and safe shutdown; recovery policy is deferred to a later discuss phase.
- [ ] **LIFT-05**: Every movement transition confirms fresh valid Localizer/odometry and encoder progress, finite pose, target tolerance, configured position/heading error, minimum settle cycles, no-progress detection, and encoder validity; invalid or stale state reaches bounded retry or `SAFE_STOP`.
- [ ] **LIFT-06**: `MOVE_TO_SHELF`, `CENTER`, `APPROACH`, `BACK_OUT`, `MOVE_NEAR_FACTORY`, `MOVE_TO_PLACEMENT_POSITION`, and `BACK_OUT_AFTER_RELEASE` cannot transition until encoder-confirmed target pose gates pass; camera, IR, and stepper gates remain enforced.

### Vision Modes

- [ ] **VIS-01**: Shared ORB vision exposes explicit `SINGLE_TARGET` and `MULTI_TARGET` roles without duplicate pipelines: each physical webcam has exactly one lifecycle/session and one frame ORB extraction, while each logical `OrbTemplateCamera` evaluator owns exactly one immutable target template.
- [ ] **VIS-02**: `SINGLE_TARGET` on `webcam1` tracks the left pallet within the one-block FOV and minimizes camera-to-left-pallet center error fast enough for lateral centering; `webcam2` does not authorize centering.
- [ ] **VIS-03**: `SINGLE_TARGET` on `webcam1` exposes transformed target center, `dxPx`, `dyPx`, hard validity, observation age, detection state, rejection reason, match/inlier counts, inlier ratio, Hamming/reprojection medians, coverage, and projected-area metrics for fork alignment; no ambiguous confidence value authorizes movement.
- [ ] **VIS-04**: Both `webcam1` and `webcam2` support classification during scan passes through logical per-template `OrbTemplateCamera` evaluators fed by their physical webcam session, with one immutable target template and one qualified result per evaluator before centering.
- [ ] **VIS-05**: Per-template evaluators use shared frame ORB descriptors and hard geometric gates, then apply deterministic same-frame ranking and per-target temporal filtering without extracting frame features inside each evaluator.
- [ ] **VIS-06**: Both roles reject invalid templates/frames and keep camera thread safe under generation-safe asynchronous start, idempotent stop, failure, stale result, unsupported control, and native-resource release paths.
- [ ] **VIS-07**: ORB/template assets, match and geometry gates, SEARCH/TRACK ROI policy, temporal policy, camera controls, center policy, and processing limits are measurable named tunable defaults. Full/large search runs every second frame, expanded tracking ROI runs every locked frame, controls expose support/readback fallback, and latency/FPS metrics remain bounded for RK3328/1 GB hardware.

### Mechanism and Sensors

- [x] **MECH-01**: Elevator homes toward `endstop1`, stops immediately on activation, and resets logical step position.
- [x] **MECH-02**: Stepper motion uses `step` pulses and `dir` direction with calibrated travel bounds.
- [x] **MECH-03**: Both fork servos support `PLACE` parallel to field and `HOLD` perpendicular to field.
- [x] **MECH-04**: `leftIR` and `rightIR` confirm cargo-ready position with debounce; they do not classify block type.
- [x] **MECH-05**: Pickup sequence consumes two explicit camera identities/channels (`webcam1` left-centering role and `webcam2` second-camera role) through a validity/freshness-aware result contract; Phase 2 does not implement OpenCV/template matching or physical I2C frame parsing; one I2C device is detectable by Control Hub through HardwareMap, then mechanical coupling centers the right fork before forward approach, dual IR confirmation, one-cycle pickup, and lift. The logical camera payload is packed into 20 bits: unsigned X center bits 0-7, unsigned Y center bits 8-15, left block code bits 16-17, and right block code bits 18-19. Codes 0..3 map configurably to block types 01..04; invalid/reserved, stale, or partial reads cannot authorize movement. Physical frame/register/endian/checksum details, sentinel policy, screen coordinates, and I2C read atomicity remain deferred.

### Autonomous Flow

- [ ] **AUTO-01**: Robot performs right scan → left scan → center both blocks → forward approach → IR confirm → one-cycle dual pickup → transport → place as bounded states.
- [ ] **AUTO-02**: Block types 01–04 route to Samsung, Foxconn, Amkor, and Hana Micron Vina respectively.
- [ ] **AUTO-03**: Placement stops with pallet parallel to field and fully inside target 250 x 250 mm factory area.
- [ ] **AUTO-04**: Task 2 cannot start until task 1 completes 100%.
- [ ] **AUTO-05**: All movement states handle stop request, missing detection, failed IR confirmation, jam, and safe stop; recovery policy is deferred to a later discuss phase.

### Verification

- [ ] **TEST-01**: Deterministic offline tests call production scalar seams for match selection, transformed-center and quadrilateral geometry, explicit quality gates, temporal acquisition/smoothing/velocity/misses, SEARCH/TRACK ROI, same-frame ranking, one-extraction fan-out accounting, processing budget, stale observations, and movement authorization; hardware tuning separately measures controls, jitter, reacquisition, false locks, and timing.
- [x] **TEST-02**: Hardware test OpMode verifies exact device names, directions, encoder signs, servo states, IR polarity, endstop, and stepper pulses.
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
| Camera architecture outside the unified ORB physical-session/logical-evaluator path | Phase 8 is limited to consolidating ORB tracking; `TemplateMatchCamera` remains an isolated benchmark |
| Automatic task 2 before task 1 completion | Forbidden by O2 rules |

## Traceability

| Requirement | Phase | Status |
|-------------|-------|--------|
| LOC-01–LOC-05 | Phase 1 | Pending |
| LIFT-01–LIFT-04, MECH-01–MECH-05, AUTO-01, AUTO-05 | Phase 2 | Pending |
| VIS-01, VIS-04, VIS-05 | Phases 3 and 8 | Pending |
| VIS-02, VIS-03, VIS-06, VIS-07, TEST-01 | Phases 7 and 8 | Pending |
| AUTO-02–AUTO-04 | Phase 4 | Pending |
| TEST-02–TEST-03 | Phase 5 | Pending |

**Coverage:** 25 v1 requirements; 25 mapped; 0 unmapped.

---
*Requirements defined: 2026-08-06*
