# RoboconBN O2 Robot Vision and Autonomous Control

## What This Is

FTC Robot Controller project for Robocon Bắc Ninh mở rộng 2026, bảng O2. Mecanum robot uses four named drive motors, goBILDA four-bar odometry, a stepper-driven elevator, two-servo fork, IR cargo confirmation, and `webcam1` OpenCV vision to classify and pick pallets.

This project records robot behavior and researches improvements to `TemplateMatchCamera` with two explicit modes: `SINGLE_TARGET` for fast, precise center alignment and `MULTI_TARGET` for accurate block-type classification. The camera is mounted above the left fork blade, with a field of view covering one block. Robot uses MULTI_TARGET to classify both blocks, then uses SINGLE_TARGET during lateral centering to put the camera center on the left pallet; mechanical coupling brings the right fork to the right pallet center, making one dual pickup safe.

## Core Value

Robot identifies the right cargo and centers the fork accurately enough to lift pallet safely, then places it in the correct factory area.

## Requirements

### Validated

- Existing FTC Gradle project and Java TeamCode module.
- Existing `TemplateMatchCamera` using `Imgproc.matchTemplate(... TM_CCOEFF_NORMED)` and one `Core.minMaxLoc` result.
- Existing mecanum, PID, and two-wheel odometry classes.

### Active

- [ ] Record exact hardware contract and safe defaults.
- [ ] Verify four-bar odometry mapping: strafe=`rightfront`, forward=`leftfront`.
- [ ] Design `SINGLE_TARGET` center-precision vision mode.
- [ ] Design `MULTI_TARGET` classification mode with thresholding and overlap suppression.
- [ ] Preserve shared camera lifecycle and result API where practical.
- [ ] Integrate elevator, fork, IR confirmation, and camera outputs into autonomous flow.
- [ ] Verify O2 pickup and placement constraints on field.

### Out of Scope

- New vision dependency or deep neural network — OpenCV template matching is current constraint.
- AprilTag localization — odometry and OpenCV block matching are current scope.
- Generic multi-camera support — only `webcam1` required.

## Context

Mechanical system has two fork blades controlled by `servoLeft` and `servoRight`, attached to an extrusion elevator on a linear carriage. Stepper interface uses `step` pulse and `dir` direction. `endstop1` establishes elevator home. Fork states: `PLACE` parallel to field; `HOLD` perpendicular to field. `webcam1` mounts above the left fork blade and points straight ahead. Its FOV covers one block, not both simultaneously. At the active elevator height, MULTI_TARGET performs the right and left classification scans and stores both block labels. During lateral centering, SINGLE_TARGET tracks the left pallet center at high speed and accuracy; the robot strafes until the camera center aligns with that pallet. Mechanical spacing simultaneously places the right fork at the right pallet center. The robot then drives forward until `leftIR` and `rightIR` confirm the pair is ready; the mechanical fork picks both packages in one cycle. IR sensors mount above the camera and only confirm forward pickup position.

O2 rules: match limit 240 seconds; 12 cargo packages per side; four block types map to Samsung, Foxconn, Amkor, and Hana Micron Vina; pallet lifting required; valid placement requires top projection fully inside 250 x 250 mm factory area with pallet parallel to field; task 2 only after task 1 complete.

## Constraints

- **Hardware names**: Device names case-sensitive: `leftfront`, `leftback`, `rightfront`, `rightback`, `servoLeft`, `servoRight`, `leftIR`, `rightIR`, `step`, `dir`, `webcam1`, `endstop1`.
- **Vision**: OpenCV only. Single mode prioritizes center error; multi mode prioritizes type confidence.
- **Safety**: Elevator and drive actions require timeout, stop-request handling, and endstop protection.
- **Performance**: Camera pipeline must avoid unbounded allocations and keep frame latency suitable for FTC control.
- **Compatibility**: Reuse existing Android/FTC/OpenCV stack; no new dependency.

## Key Decisions

| Decision | Rationale | Outcome |
|----------|-----------|---------|
| One `TemplateMatchCamera` with explicit mode | Shared lifecycle, less duplicated camera code | — Pending |
| Separate scoring policy by mode | Center precision and type confidence have different optima | — Pending |
| Multi-target uses threshold + non-maximum suppression | `minMaxLoc` alone cannot classify multiple candidates reliably | — Pending |
| IR confirms ready state, not block type | Physical sensor role is presence/readiness | — Pending |

---
*Last updated: 2026-08-06 after robot description, codebase review, and O2 PDF review*
