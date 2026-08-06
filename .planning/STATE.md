# Project State

## Project Reference

See: `.planning/PROJECT.md` (updated 2026-08-06)

**Core value:** Robot identifies the right cargo and centers the fork accurately enough to lift pallet safely, then places it in the correct factory area.
**Current focus:** Phase 1 — Localizer Validation

## Current Position

- Planning initialized from O2 rule PDF and robot description.
- Existing codebase mapped under `.planning/codebase/`.
- `TemplateMatchCamera` currently performs one-template best-match detection using `TM_CCOEFF_NORMED` and `minMaxLoc`.
- Two future modes specified: `SINGLE_TARGET` for fast left-pallet center alignment and `MULTI_TARGET` for right/left block type classification.
- Hardware contract code exists in `RobotHardware.java`; calibration OpMode uses `Localizer` and named drive devices.
- Roadmap plans are now decomposed into two executable plans per phase.

## Decisions

- One `TemplateMatchCamera` class with explicit mode, not duplicate camera classes.
- Multi-target candidate extraction requires raw thresholding and overlap suppression.
- IR sensors confirm cargo-ready position only.
- Elevator homes at `endstop1`; fork states are PLACE and HOLD.

## Next Action

Run `/gsd-plan-phase 1` to create executable Localizer validation plans 01-01 and 01-02.

## Blockers / Measurements Needed

- Exact motor/encoder directions and odometry scale.
- Stepper driver timing and active polarity for `step`, `dir`, `endstop1`.
- Servo positions for PLACE/HOLD.
- IR active level and debounce interval.
- Camera mounting transform, template dimensions, thresholds, and block template assets.
- Field poses for shelves and factories.
