# Roadmap: RoboconBN O2 Robot Vision and Autonomous Control

## Overview

Start with a verified hardware contract, then improve the existing `TemplateMatchCamera` into two measurable modes, add safe elevator/fork and IR behavior, compose the autonomous state machine, and validate the complete O2 flow on a full-size field.

## Phases

- [ ] **Phase 1: Localizer Validation** - Validate encoder signs, encoder constants, IMU orientation/turn direction, and localization math only.
- [ ] **Phase 2: Two-Mode OpenCV Camera** - Add `SINGLE_TARGET` center precision and `MULTI_TARGET` type classification.
- [ ] **Phase 3: Pickup Mechanism** - Add elevator homing, step control, fork states, and IR readiness.
- [ ] **Phase 4: O2 Autonomous Flow** - Connect vision, odometry, pickup, routing, placement, and recovery.
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

### Phase 2: Two-Mode OpenCV Camera

**Goal**: Existing `TemplateMatchCamera` supports two explicit scoring policies without duplicated lifecycle code.
**Depends on**: Phase 1
**Requirements**: VIS-01, VIS-02, VIS-03, VIS-04, VIS-05, VIS-06, VIS-07
**Success Criteria**:

  1. `MULTI_TARGET` classifies right and left blocks before centering.
  2. `SINGLE_TARGET` tracks the left pallet center with low-latency stable error during lateral motion.
  3. Robot strafes until camera center matches left pallet center; mechanical coupling places right fork at right pallet center.
  4. Overlapping multi-target matches collapse to distinct detections and confidence remains raw, not renormalized to 1.
  5. Camera start/stop/error handling works identically in both modes.

**Plans**: 2 plans

Plans:

- [ ] 02-01: Add explicit camera mode and shared result/config contract.
- [ ] 02-02: Implement multi-candidate classification, left-target tracking, and tests.

### Phase 3: Pickup Mechanism

**Goal**: Elevator and fork acquire pallet safely and expose readiness.
**Depends on**: Phase 1
**Requirements**: MECH-01, MECH-02, MECH-03, MECH-04, MECH-05
**Success Criteria**:

  1. Elevator homes once against `endstop1`, then respects calibrated bounds.
  2. Fork reaches PLACE/HOLD positions with both servos coordinated.
  3. Robot drives forward after lateral centering until debounced `leftIR` and `rightIR` confirm both packages are ready.
  4. One fork cycle engages and lifts both pallets clear before drive translation.

**Plans**: 2 plans

Plans:

- [ ] 03-01: Implement elevator homing, bounded step pulses, and endstop timeout safety.
- [ ] 03-02: Implement PLACE/HOLD fork states, IR debounce, and dual-pallet pickup sequence.

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
| 2. Two-Mode OpenCV Camera | 0/2 | Not started | - |
| 3. Pickup Mechanism | 0/2 | Not started | - |
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
