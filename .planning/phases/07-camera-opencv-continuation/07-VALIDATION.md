# Phase 7 Validation Contract: Single-Target ORB Accuracy

## Scope Gate

Allowed production change: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/SingleTargetCamera.java`.

Allowed support changes: `OrbTarget1TestOpMode.java`, `CameraContinuationTest.java`, and Phase 7 planning evidence.

Excluded: `TemplateMatchCamera`, `OrbTemplateCamera`, `FourTargetCameraOrchestrator`, multi-target classification, UART, I2C, packed transport, lifting, autonomous integration, other camera consumers, and dependencies.

## Ordered Gates

1. Compile and deterministic offline policy checks:
   `.\gradlew.bat :TeamCode:compileDebugJavaWithJavac :TeamCode:cameraContinuationTest --offline`
2. Run hardware only after gate 1 passes.
3. Use `ORB Target 1 Test`; camera must open during FTC `INIT` and release safely on stop/error.
4. Collect at least 300 processed frames after warm-up for each condition below.
5. Record actual metrics and final tuned named defaults in `07-VERIFICATION.md`.

## Deterministic Offline Coverage

- Match qualification: absolute Hamming, ratio, deterministic sort, unique scene correspondence, cap, minimum count, template coverage, and quadrants.
- Homography: finite 3x3 matrix, inliers, inlier ratio, median/p90 reprojection, inlier coverage, convexity/winding, area, bounds, edge, diagonal, and transformed template center.
- Temporal: three-frame acquisition, jitter suppression, monotonic movement response, single spike, confirmed relocation, miss hold, time/count expiry, stale/future clock, stop/error, non-finite values, processing budget, and deterministic replay.
- Tests invoke production scalar policy and add no dependency.

## Hardware Conditions and Initial Acceptance

### Stationary Centered Target

- Samples: at least 300 processed frames after warm-up.
- Qualified observations after acquisition: at least 95 percent.
- Filtered radial jitter p95: at most 4 px.
- Accepted center jump: none above 35 px.
- Record raw jitter, filtered jitter, acquisition frames/ms, rejection reasons, FPS, and processing p50/p95/max.

### Lateral Movement

- Samples: at least 300 processed frames at recorded bench speed.
- Filtered lag p95: below 40 px.
- No stale result authorizes movement.
- Coherent movement remains trackable without repeated false relocation rejection.
- Record acquisition/relocation latency, lag, outlier count, FPS, and processing p50/p95/max.

### Negative Scene

- Samples: at least 300 processed frames; include repetitive texture when available.
- LOCKED acquisitions: zero.
- Movement-authorized results: zero.
- Record first-failure rejection distribution, closest quality observed, FPS, and processing p50/p95/max.

### Processing Budget

- Processing p95: at most 100 ms.
- Sustained over-budget run: no longer than 3 consecutive frames.
- If budget fails, reduce ORB feature count before weakening geometry or stale-result gates.

## Tuning Order

1. Eliminate false locks using descriptor, coverage, reprojection, and projected-geometry evidence.
2. Recover positive recall only after negative-scene acceptance passes.
3. Adjust acquisition, smoothing, and outlier constants from measured jitter, movement lag, and FPS.
4. Keep thresholds as named `SingleTargetCamera` constants unless existing project configuration already provides a matching camera-tuning pattern.
5. Rerun offline checks and every failed hardware condition after each final threshold change.

## Required Evidence

Record template asset, lighting/distance, sample counts, stationary jitter, accepted jumps, negative false locks, acquisition latency, movement lag, FPS, processing p50/p95/max, over-budget streak, rejection counts, final constants, lifecycle outcome, and blockers. Hardware remains unapproved until all measurements exist.
