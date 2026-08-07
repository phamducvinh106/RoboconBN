# Roadmap: RoboconBN O2 Robot Vision and Autonomous Control

## Overview

Start with a verified hardware contract, then improve the existing `TemplateMatchCamera` into two measurable modes, add safe elevator/fork and IR behavior, compose the autonomous state machine, and validate the complete O2 flow on a full-size field.

## Phases

- [ ] **Phase 1: Localizer Validation** - Validate encoder signs, encoder constants, IMU orientation/turn direction, and localization math only.
- [ ] **Phase 2: Lifting Sequence State Machine** - Execute homing, height selection, camera alignment, IR approach, dual pickup, transport, placement, and return-home safely.
- [ ] **Phase 3: Camera/OpenCV Continuation** - Add shared `SINGLE_TARGET` centering and `MULTI_TARGET` classification for `webcam1`/`webcam2`.
- [ ] **Phase 4: O2 Autonomous Flow** - Connect lifting state machine, odometry, camera, pickup, routing, placement, and recovery.
- [ ] **Phase 5: Verification and Tuning** - Test camera metrics, hardware, failure paths, and match timing.

## Phase Details

### Phase 1: Localizer Validation

**Goal**: Prove `Localizer` converts goBILDA four-bar encoder and IMU measurements into correct robot pose.
**Depends on**: Nothing
**Requirements**: LOC-01, LOC-02, LOC-03, LOC-04, LOC-05
**Success Criteria**:

  1. `rightfront` strafe encoder and `leftfront` forward encoder signs are measured and recorded.
  2. Wheel diameter, ticks per revolution, pod offsets, encoder scale, deadbands, and heading sign are calibrated.
  3. IMU orientation `LogoFacingDirection.BACKWARD` and `UsbFacingDirection.UP` produces correct yaw direction.
  4. Forward, strafe, rotation, and combined motion produce expected local/global pose signs.
  5. `Localizer` update, angle wrapping, pose reset, and offset suggestion algorithms are verified against measured data.

**Plans**: 2 plans

Plans:

- [ ] 01-01: Measure encoder signs/constants and IMU yaw orientation with calibration telemetry.
- [ ] 01-02: Verify and correct Localizer pose transform, heading wrap, reset, and offset calculations.

### Phase 2: Lifting Sequence State Machine

**Goal**: Execute one complete safe lifting cycle using `step`, `dir`, servos, IR sensors, odometry, and validity/freshness-aware camera outputs received through one Raspberry Pi 5 I2C device at 7-bit address 0x42, HardwareMap name `pi5Camera`, and explicit logical channel selection.
**Depends on**: Phase 1
**Requirements**: MECH-01, MECH-02, MECH-03, MECH-04, MECH-05, AUTO-01, AUTO-05
**Success Criteria**:

  1. States cover `START`, `HOMING`, `PLACE`, `MOVE_TO_FAC`, `READY1`, `READY2`, `SCAN_LEFT`, `SCAN_RIGHT`, `CENTER_LEFT`, `APPROACH_IR`, `LIFT1`, `LIFT2`, `BACK_OUT`, `HOLD`, `MOVE_TO_FACTORY`, `PLACE_AT_FACTORY`, `READY1_PUSH`, and `HOME`.
  2. Stepper uses bounded `step` pulses and `dir`; homing stops immediately at `endstop1`.
  3. Ready height matches current pickup floor; lift height raises cargo clear; `HOME` lowers cargo to floor-safe travel height.
  4. Both IR sensors must be active before lift; missing/stale camera data or failed IR confirmation safe-stops.
  5. Fixed factory coordinates route by left-block classification; placement sequence handles left block, then right block, using `READY1` to push existing cargo deeper.
  6. Every state has timeout, stop-request, and safe motor/servo shutdown behavior.

**Plans**: 3/5 plans executed

Execution order: Control Hub I2C detection/address/read-write/heartbeat/channel bench test → centralized JSON → state-machine integration → placement/wiring/acceptance. Phase 2 preserves two explicit logical camera channels on one I2C device, defers register/frame parsing and all OpenCV/template matching, and requires FTC SDK API verification before implementation.

Plans:

- [x] 02-01-PLAN.md — Define one manager per physical subsystem, narrow contracts, seams, and manager tests.
- [x] 02-02-PLAN.md — Run isolated low-power hardware communication OpMode and record wiring/calibration evidence.
- [x] 02-03-PLAN.md — Load centralized strict JSON configuration before any hardware or runtime motion.
- [ ] 02-04-PLAN.md — Integrate manager contracts into pickup state machine, D-02 gates, and production OpMode.
- [ ] 02-05-PLAN.md — Complete serial placement, release/back-out wiring, acceptance, and traceability.

### Phase 3: Camera/OpenCV Continuation

**Goal**: One lifecycle-safe shared camera API exposes `SINGLE_TARGET` centering on `webcam1` and `MULTI_TARGET` classification on `webcam2`.
**Depends on**: Phase 1
**Requirements**: VIS-01, VIS-02, VIS-03, VIS-04, VIS-05, VIS-06, VIS-07, TEST-01
**Plans**: 2 plans

Plans:

- [ ] 03-01: Normalize shared camera contract, lifecycle safety, explicit webcam mapping, and OpMode consumers.
- [ ] 03-02: Add offline camera continuation tests and finalize validation.

### Phase 4: O2 Autonomous Flow

**Goal**: Robot completes bounded classify-to-place cycles and obeys task gating.
**Depends on**: Phase 2, Phase 3
**Requirements**: AUTO-01, AUTO-02, AUTO-03, AUTO-04, AUTO-05
**Success Criteria**:

  1. State machine scans, classifies, aligns, picks, routes, places, and verifies.
  2. Four block classes route to their required factories.
  3. Placement meets pallet-level and 250 x 250 mm containment constraints.
  4. Failed states recover, skip, or safe-stop without indefinite motion.

**Plans**: 2 plans

Plans:

- [ ] 04-01: Implement explicit O2 autonomous state machine and route mapping.
- [ ] 04-02: Integrate scan, left-target centering, dual pickup, placement verification, and recovery.

### Phase 5: Verification and Tuning

**Goal**: Critical logic is tested and tuned against real field behavior.
**Depends on**: Phase 4
**Requirements**: TEST-01, TEST-02, TEST-03
**Success Criteria**:

  1. Offline tests cover both vision modes and autonomous transitions.
  2. Hardware tests verify polarity, sensor active levels, servo poses, and stepper limits.
  3. Full-size field run covers randomized cargo, reset, failures, and 240-second budget.

**Plans**: 2 plans

Plans:

- [ ] 05-01: Add offline and hardware verification coverage for vision, hardware, and state transitions.
- [ ] 05-02: Tune field constants and validate complete 240-second match flow.

## Progress

| Phase | Plans Complete | Status | Completed |
|-------|----------------|--------|-----------|
| 1. Hardware Contract | 0/2 | Not started | - |
| 2. Lifting Sequence State Machine | 3/5 | In Progress|  |
| 3. Camera/OpenCV Continuation | 0/2 | Not started | - |
| 4. O2 Autonomous Flow | 0/2 | Not started | - |
| 5. Verification and Tuning | 0/2 | Not started | - |

### Phase 6: Automatic PID tuning OpMode for MecanumDrive within a bounded 50 x 50 cm square

**Goal:** [To be planned]
**Requirements**: TBD
**Depends on:** Phase 5
**Plans:** 1/2 plans executed

Plans:

- [x] 06-01-PLAN.md
- [ ] 06-02-PLAN.md

- [ ] TBD (run /gsd-plan-phase 6 to break down)

### Phase 7: Camera/OpenCV Continuation (superseded by Phase 3 camera scope)

**Goal:** Continue camera work from `ColorContourCamera.java` with one lifecycle-safe shared API: `SINGLE_TARGET` centering on `webcam1` and `MULTI_TARGET` classification on `webcam2`.
**Requirements**: VIS-01, VIS-02, VIS-03, VIS-04, VIS-05, VIS-06, VIS-07, TEST-01
**Depends on:** Phase 6
**Plans:** 0/2 plans executed

**Success Criteria**:

1. One shared `ColorContourCamera` lifecycle exposes explicit `SINGLE_TARGET` and `MULTI_TARGET` policies with measurable center, confidence, validity, threshold, overlap, and freshness behavior.
2. `webcam1` is explicitly used for left-pallet centering and `webcam2` is explicitly used for multi-target classification; no silent camera fallback exists.
3. Start, stop, open failure, released resources, and stale results are safe; stale/error/closed results cannot command movement.
4. Offline dependency-free tests cover center math, candidate ranking, overlap suppression, confidence thresholds, mode selection, and stale-result rejection.

Plans:

- [ ] 07-01-PLAN.md — Normalize shared camera contract, lifecycle safety, explicit webcam mapping, and OpMode consumers.
- [ ] 07-02-PLAN.md — Add offline camera continuation tests and finalize validation.
