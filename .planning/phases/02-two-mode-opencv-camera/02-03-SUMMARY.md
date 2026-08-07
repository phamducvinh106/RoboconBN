---
phase: 02-two-mode-opencv-camera
plan: 03
subsystem: autonomous-lifting
tags: [ftc, state-machine, factory-routing, placement, safety]
requires:
  - phase: 02-two-mode-opencv-camera
    provides: pickup state machine and cooperative elevator contract
provides:
  - fixed block-type factory routing
  - serial left-before-right placement transaction
  - explicit release and 20 cm back-out gates
affects: [autonomous-lifting, hardware-validation]
tech-stack:
  added: []
  patterns: [named factory config, cooperative placement states, explicit release gate]
key-files:
  created: []
  modified:
    - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/LiftingSequenceConfig.java
    - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/LiftingSequenceStateMachine.java
    - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/LiftingSequenceTest.java
    - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/LiftingSequenceOpMode.java
decisions:
  - "Factory routing uses named block types 01-04; IR remains presence-only."
  - "Left placement must release and back out 20 cm before right placement starts."
  - "Release confirmation is explicit; unavailable hardware contract blocks automatic advancement."
requirements-completed: [LIFT-02, LIFT-03, LIFT-04, MECH-03, MECH-05, AUTO-01, AUTO-05]
metrics:
  duration: 9min
  completed: 2026-08-07
status: complete
---

# Phase 2 Plan 3: Serial Factory Placement Summary

**Fixed factory mapping and guarded serial placement for left and right blocks.**

## Accomplishments

- Added validated factory mapping for block types `01` through `04`, with named near-factory and exact placement poses.
- Replaced pickup-to-cycle shortcut with explicit `HOLD → MOVE_NEAR_FACTORY → PLACE → READY1 → MOVE_TO_PLACEMENT → HOME → release → 20 cm back-out` states for each block.
- Enforced left transaction completion before right transaction; right path restores lift height before transport and does not assume left `HOME` preserved it.
- Added pose gates, finite configuration validation, explicit release confirmation, and bounded SAFE_STOP behavior.
- Added OpMode pose callback and conservative unavailable release/back-out callbacks so hardware cannot falsely advance.

## Verification

- Focused executable test: `RESULT: 5 passed, 0 failed`.
- TeamCode compile: `BUILD SUCCESSFUL`.
- Physical hardware acceptance remains pending for servo calibration, factory coordinates, release sensor wiring, odometry, and six-cycle supervised run.

## Task Commits

1. `d326ae0` — `feat(02-03): add fixed serial placement routing`
2. `ec421a1` — `fix(02-03): require explicit release confirmation`

## Deviations from Plan

### Auto-fixed Issues

**1. [Rule 2 - Missing critical functionality] Removed unconditional release/back-out success from OpMode adapter**
- **Found during:** Task 2
- **Issue:** Placeholder success would allow cycle advancement without physical release confirmation or measured back-out.
- **Fix:** OpMode returns unconfirmed release and zero measured back-out until a real sensor/odometry contract is wired.
- **Files modified:** `LiftingSequenceOpMode.java`
- **Commit:** `ec421a1`

### Intentional simplification

Factory numeric coordinates remain calibration defaults from plan context. Hardware acceptance must validate or replace them before field operation.

## Known Stubs

- `LiftingSequenceOpMode.released()` returns `false` and `backOutDistanceCm()` returns `0.0` because no explicit release sensor or measured odometry callback exists in current hardware contract. This intentionally forces operator reset instead of unsafe automatic advancement.

## Hardware Validation Checklist

- Verify exact FTC device names and directions.
- Verify active-low `endstop1`, `leftIR`, and `rightIR` polarity plus debounce.
- Measure step pulse timing and bounded travel for `HOME`, `READY1`, `READY2`, `LIFT1`, and `LIFT2`.
- Calibrate `PLACE` and `HOLD` servo poses.
- Validate all factory near/placement coordinates and X/Y/heading tolerances.
- Run supervised one-cycle then `3 shelves × 2 levels = 6 cycles`; any failure requires reset-only recovery.

## Self-Check: PASSED

- Modified files exist.
- Task commits `d326ae0` and `ec421a1` exist.
- Focused test and TeamCode compile passed.
