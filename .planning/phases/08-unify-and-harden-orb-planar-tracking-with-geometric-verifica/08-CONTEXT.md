# Phase 8 Context: Unified Hardened ORB Planar Tracking

## Decisions

- **D-01**: `OrbTemplateCamera` is the only production ORB target evaluator. Keep the completed `SingleTargetCamera` removal; do not recreate another ORB pipeline. `TemplateMatchCamera` remains an isolated non-ORB benchmark and never enters ORB production flow.
- **D-02**: Separate physical capture from logical targets. Exactly one explicit session owns each named physical webcam, lifecycle, controls, frame buffers, dynamic ROI, and frame ORB extraction. Logical `OrbTemplateCamera` evaluators own immutable template features and per-target state. Do not use hidden registries or singleton camera sharing.
- **D-03**: Descriptor qualification uses bidirectional KNN Hamming with ratio `0.70`, a tunable absolute Hamming default in the locked `50–60` range (start `60`), mutual best matches, deterministic distance/index sorting, unique template and scene indices, cap after sorting, and at least 12 retained matches.
- **D-04**: Geometry is hard-gated: finite non-empty 3x3 RANSAC homography near `3 px`, at least 10 inliers, at least `0.55` inlier ratio, median reprojection at most `3 px`, broad inlier coverage with at least three quadrants, and finite plausible convex/frame-bounded projected quadrilateral measured against frame area. Publish `perspectiveTransform` of template center, not average corners.
- **D-05**: Hard gates alone determine validity. Replace ambiguous confidence with named match, inlier, Hamming, reprojection, coverage, projected-area, rejection, state, timestamp, ROI, and timing metrics. Any aggregate quality score is ranking/telemetry only and cannot authorize movement.
- **D-06**: Per-target temporal states are `SEARCHING`, `LOCKED`, `COASTING`, and `LOST`. Three coherent qualified observations acquire lock; three actual misses lose it. Intentional skipped search frames are not misses. `COASTING` is display continuity only and never authorizes movement.
- **D-07**: Preserve `lastObservedTimestampMs` separately from publication time. Movement requires a new observation from the current frame, `STREAMING`, `LOCKED`, geometrically valid finite center/error, `webcam1` centering role, verified required camera controls, processing within budget, non-future clock, and age at most `120 ms`.
- **D-08**: Filtering occurs only after geometry and continuity gates: velocity gate using monotonic elapsed time, median of three accepted raw centers, then EMA. Start EMA `0.3` and current velocity limit only as tunable defaults; hardware evidence must tune motion bounds without weakening safety gates.
- **D-09**: Dynamic ROI policy uses full frame or largest safe search region every second frame in `SEARCHING`/`LOST`; `LOCKED` evaluates every frame in projected target bounds expanded `1.5` and clamped to frame. `COASTING` may use the last track ROI briefly but schedules large search. Invalid/small/empty ROI falls back to search.
- **D-10**: Rank only geometrically qualified observations from the same physical webcam and frame, lexicographically by more inliers, higher inlier ratio, lower median reprojection, higher coverage, then stable target ID. Frame IDs and full-frame coordinates remain explicit.
- **D-11**: Apply manual exposure and gain after async open, validate reported support/range/set/readback, and expose actual values. Unsupported or failed required exposure/gain keeps movement authorization false while allowing diagnostic streaming. Apply fixed focus when supported; unsupported focus is explicit telemetry fallback, not a crash or hidden success.
- **D-12**: Keep INIT-time async opening, generation-safe callbacks, idempotent stop, failure invalidation, no queued `Mat` frames, sequential per-target evaluation on the camera pipeline thread, and complete native resource release. Add no dependency or worker thread.
- **D-13**: Offline plain-Java executable tests call production scalar policy seams directly. Hardware tuning uses a dedicated OpMode on the same unified path and reports controls, state, quality, ROI, jitter/lag, reacquisition, false locks, FPS, and processing distributions.
- **D-14**: `target2.png`, `target3.png`, and `target4.png` are missing input assets. Multi-target hardware acceptance must fail closed with an explicit asset blocker until real verified assets and target-to-webcam mappings are supplied; plans must not fabricate images.
- **D-15**: Numeric camera settings, area/skew bounds, velocity limits, EMA, ORB work caps, and performance targets are named initial tunable defaults, not measured claims. Record support and measured values before calling them accepted.

## Assumptions

- Current FTC SDK 11.2.1, EasyOpenCV/OpenCV stack, Java, `webcam1`, `webcam2`, and `target1.png` remain available.
- `OrbTemplateCamera.java` from quick task `260808-5az` is current source truth; Phase 7 documents describing restored `SingleTargetCamera` are historical and must not override D-01.
- Two logical target evaluators attach to each physical webcam in the current four-target mapping; tuning OpMode must expose mapping clearly so supplied assets can be verified rather than inferred.
- Required exposure/gain values and optional fixed-focus value cannot be finalized without webcam support/range evidence.

## Deferred Ideas

- UART, I2C, packed transport, lifting, autonomous state machine, route logic, AprilTags, neural detection, extra camera frameworks, and any `TemplateMatchCamera` production integration are outside Phase 8.
- New dependencies, background ORB worker threads, persistent camera registries, and runtime threshold configuration systems are outside Phase 8 unless existing code already provides the exact mechanism.

## Source Coverage Audit

| Source | ID | Feature / requirement | Plan | Status |
|---|---|---|---|---|
| GOAL | — | One authoritative safe bounded ORB path with shared physical extraction and measurable tracking | 08-01, 08-02, 08-03 | COVERED |
| REQ | VIS-01 | One session/extraction per webcam plus immutable logical evaluators | 08-02 | COVERED |
| REQ | VIS-02 | webcam1-only qualified centering authority | 08-01, 08-02 | COVERED |
| REQ | VIS-03 | Transformed center and explicit quality/age/state metrics | 08-01, 08-02, 08-03 | COVERED |
| REQ | VIS-04 | Both webcams feed externally orchestrated template evaluators | 08-02, 08-03 | COVERED |
| REQ | VIS-05 | Geometric evaluation, same-frame ranking, per-target temporal state | 08-01, 08-02 | COVERED |
| REQ | VIS-06 | Invalid/lifecycle/control/resource fail-closed behavior | 08-01, 08-02, 08-03 | COVERED |
| REQ | VIS-07 | Named ORB/ROI/control/performance policies | 08-01, 08-02, 08-03 | COVERED |
| REQ | TEST-01 | Deterministic production-policy tests plus hardware tuning path | 08-01, 08-03 | COVERED |
| RESEARCH | — | NaN, convexity, frame-area, transformed-center, finite-homography, timestamp, and match-cap defects | 08-01, 08-02 | COVERED |
| RESEARCH | — | Duplicate webcam opens and repeated extraction | 08-02 | COVERED |
| RESEARCH | — | Dynamic ROI, control support fallback, RK3328 bounded work | 08-02, 08-03 | COVERED |
| RESEARCH | — | Missing assets block multi-target hardware acceptance | 08-03 | COVERED |
| CONTEXT | D-01–D-15 | All locked Phase 8 decisions above | 08-01, 08-02, 08-03 | COVERED |
