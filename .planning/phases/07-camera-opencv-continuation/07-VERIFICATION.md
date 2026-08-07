---
phase: 07-camera-opencv-continuation
verified: 2026-08-08
status: replanned
score: 0/5 revised must-haves verified
---

# Phase 7 Verification: Single-Target ORB Accuracy

## Revised Goal

Make `SingleTargetCamera` produce stable, qualified target centers on `webcam1` without weakening camera lifecycle safety or processing bounds.

## Current Evidence

- User reports ORB has run on hardware; noisy output is the active gap.
- Refreshed research diagnoses weak post-homography qualification and one-hit temporal publication.
- Existing Phase 7 plans and evidence targeted `OrbTemplateCamera`, four-target orchestration, and transport boundaries; those artifacts are stale for this replan.
- Current checkout may lack `SingleTargetCamera.java` because prior quick work deleted it. Plan 07-01 must recover its last repository baseline before scoped improvement, without modifying `OrbTemplateCamera`.
- No revised implementation, deterministic policy run, or revised hardware metrics are claimed yet.

## Revised Requirement Coverage

| Requirement | Planned evidence | Status |
|---|---|---|
| VIS-02 | Qualified, stable `webcam1` target center; measured jitter, acquisition, and movement lag | Pending 07-01 through 07-03 |
| VIS-03 | Raw/filtered center and error, confidence/quality, validity/state, observation age, and rejection telemetry | Pending 07-01 through 07-03 |
| VIS-06 | Preserved INIT lifecycle, idempotent stop, generation/error/resource safety, invalid/stale rejection | Pending 07-01 through 07-03 |
| VIS-07 | Named ORB, match, geometry, temporal, and processing defaults plus hardware-tuned evidence | Pending 07-01 through 07-03 |
| TEST-01 | Dependency-free deterministic production-policy geometry, temporal, stale, and latency checks | Pending 07-03 |

## Revised Must-Haves

1. Weak, duplicate, clustered, high-error, or implausible geometry cannot publish a center.
2. Stable-frame acquisition, adaptive smoothing, outlier confirmation, and bounded miss hold reduce noise without accepting isolated jumps or stale data.
3. Compact telemetry exposes enough quality, state, jitter, and timing evidence to tune named defaults.
4. Offline production-policy checks pass deterministically without dependencies.
5. Hardware acceptance records stationary jitter, false positives, acquisition latency, movement response, FPS, and processing latency over repeatable samples.

## Hardware Evidence Template

| Condition | Frames | Qualified/locks | Jitter or lag p95 | Acquisition | FPS | Processing p50/p95/max | Result |
|---|---:|---:|---:|---:|---:|---:|---|
| Stationary centered | Pending | Pending | Pending | Pending | Pending | Pending | Pending |
| Lateral movement | Pending | Pending | Pending | Pending | Pending | Pending | Pending |
| Negative scene | Pending | Pending | Pending | n/a | Pending | Pending | Pending |

## Final Tuned Defaults

Pending hardware acceptance. Record only values actually applied to named `SingleTargetCamera` constants, reason, and rerun evidence.

## Scope Audit

Revised plans permit production changes only in `SingleTargetCamera.java`; `OrbTarget1TestOpMode.java` and `CameraContinuationTest.java` are support only. No plan modifies `TemplateMatchCamera`, `OrbTemplateCamera`, `FourTargetCameraOrchestrator`, UART, I2C, lifting, autonomous code, unrelated camera consumers, or dependencies.

## Blocking Order

Run `.\gradlew.bat :TeamCode:compileDebugJavaWithJavac :TeamCode:cameraContinuationTest --offline`, then execute the `07-VALIDATION.md` hardware conditions. Status remains pending until measured evidence replaces every `Pending` field above.
