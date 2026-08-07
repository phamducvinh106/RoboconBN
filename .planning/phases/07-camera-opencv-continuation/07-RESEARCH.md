# Phase 7: SingleTargetCamera ORB Accuracy Replanning - Research

**Researched:** 2026-08-08
**Domain:** FTC EasyOpenCV/OpenCV 4.10 single-template ORB localization, geometric qualification, and temporal stabilization
**Confidence:** HIGH for current-code diagnosis; MEDIUM for proposed initial tuning until fixture and hardware sweeps

<user_constraints>
## User Constraints (from CONTEXT.md)

### Locked Decisions

- **D-01**: Continue from the existing camera lifecycle source; use ORB/template matching only. Do not create or restore a separate camera lifecycle class.
- **D-02**: Use one shared camera lifecycle and result contract for both modes.
- **D-03**: `OrbTemplateCamera` is a shared one-template-per-instance detector. Each instance owns exactly one immutable target template; external orchestration selects targets by constructing four instances, one per target image. `webcam1` and `webcam2` may each host explicit target instances. Both webcams classify through those instances, but only `webcam1` has centering authority. Camera selection and centering authority must be explicit, not inferred from mode.
- **D-04**: Offline tests use plain Java assertions/main-style tests already used under `TeamCode/src/main/java/.../test`; no new dependency.
- **D-05**: Lifecycle safety includes idempotent start/stop, open/start failures, released pipeline resources, and stale-result rejection after stop or camera error.
- **D-06**: Maximize ORB processing speed without weakening correctness: each `OrbTemplateCamera` keeps one immutable template and precomputes its template keypoints/descriptors once; external orchestration constructs four instances. Processing uses latest-frame-only/drop-backlog behavior, grayscale and bounded downscaled ROI before ORB, safe Mat/keypoint-buffer reuse, capped features/levels/matches/candidates, no avoidable per-frame allocations/copies, cheap rejection before homography/geometry, and measured per-frame latency/FPS against an explicit acceptance budget. No unsafe multithreading or skipped validation; fail closed and release resources normally.

### Claude's Discretion

- Existing EasyOpenCV/OpenCV/FTC dependencies remain the implementation boundary.
- Existing `LeftCameraCenteringTestOpMode` remains the webcam1 integration reference; right classification coverage may be updated or replaced only as needed to use the shared contract.
- `RobotHardware` will expose explicit `webcam2` handling while preserving `webcam1` behavior; missing webcam2 should fail at construction with the normal FTC hardware-map error rather than silently aliasing cameras.
- ORB/template assets, descriptor and match thresholds, and any already-scoped homography/scale/rotation constraints remain configurable; no neural detector is introduced.
- Camera result timestamps are wall-clock milliseconds and consumers must treat invalid, closed, error, or over-age results as unusable.

### Deferred Ideas (OUT OF SCOPE)

- AprilTags, neural detection, new adaptive scale/rotation compensation, generic camera multiplexing, and field-tuning calibration are outside Phase 7.
</user_constraints>

> **Replanning scope override:** This research is strictly limited to newly added `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/SingleTargetCamera.java`, `OrbTarget1TestOpMode.java`, and their ORB tests. Do not plan or modify `TemplateMatchCamera`, `OrbTemplateCamera`, four-target orchestration, UART, lifting, or another camera architecture. `[VERIFIED: user request]`

<phase_requirements>
## Phase Requirements

| ID | Description | Research Support |
|---|---|---|
| VIS-02 | `webcam1` single-target centering | Geometric qualification, center-jitter control, acquisition gating, and hardware metrics. |
| VIS-03 | Center/error/confidence/validity/age output | Expanded result quality, observation state, age, rejection, and timing metrics. |
| VIS-06 | Reject invalid frames; lifecycle safety | Fail-closed geometry, latency, miss, stale, stop, and error paths. |
| VIS-07 | Measurable ORB, geometry, temporal, and speed policy | Named thresholds plus deterministic policy tests and hardware sweep procedure. |
| TEST-01 | Offline center, confidence, temporal, stale tests | Dependency-free production policy seam with fixed sequences and exact expected states. |
</phase_requirements>

## Summary

`SingleTargetCamera` already performs correct baseline stages: immutable template descriptors, grayscale bounded ROI, ORB/Hamming KNN matching, ratio filtering, RANSAC homography, inlier ratio, projected corners, and short miss hold. `[VERIFIED: repository source]` Hardware noise is expected from qualification gaps after homography: any finite homography with 8 inliers and 0.35 inlier ratio is accepted; projected quadrilateral shape, area, bounds, inlier spread, reprojection error, absolute descriptor distance, and center continuity are not checked. `[VERIFIED: repository source]` One accidental cluster can therefore publish a distant center, and each accepted center replaces output without acquisition confirmation or smoothing. `[VERIFIED: repository source]`

Current temporal behavior is hold-only, not filtering. A miss republishes the exact prior `Result`; a new raw hit, however implausible, immediately replaces it. `[VERIFIED: repository source]` `TEMPORAL_HOLD_MS=180` and `MAX_RESULT_AGE_MS=300` are separate, but telemetry cannot distinguish observed, tentative, held, rejected, or stale output. `[VERIFIED: repository source]` `MAX_FRAME_LATENCY_MS=100` is declared but never enforced. `[VERIFIED: repository source]` Current confidence equals only `inliers / ratioPassedMatches`, so weak descriptors, clustered inliers, poor reprojection, and implausible quadrilaterals can still report high confidence. `[VERIFIED: repository source]`

**Primary recommendation:** Keep existing ORB pipeline and add one deterministic qualification/tracking seam: quality-filter matches, validate homography geometry, derive center by transforming template center, acquire over stable frames, adaptively smooth accepted centers, require confirmation for outliers, hold misses briefly without refreshing observation time, then reject stale output.

## Architectural Responsibility Map

| Capability | Primary Tier | Secondary Tier | Rationale |
|---|---|---|---|
| Raw ORB detection and matching | `SingleTargetCamera.Pipeline` | OpenCV native API | Owns frame-local evidence. |
| Geometric qualification | `SingleTargetCamera.Pipeline` | Dependency-free policy helper | Uses homography outputs before publication. |
| Temporal state/filter | Production policy helper inside `SingleTargetCamera` | Pipeline | Must be deterministic and testable without webcam callbacks. |
| Movement-facing result | `SingleTargetCamera.Result` | Test OpMode telemetry | Publishes filtered center, observation age, quality, and state. |
| Hardware tuning evidence | `OrbTarget1TestOpMode` | Human bench run | Displays raw versus filtered behavior and rejection counters. |
| Deterministic validation | Existing plain-Java camera test | Production policy helper | Tests sequences without copying policy. |

## Current Code Diagnosis

### Frame-local matching

- ORB uses `ORB.create(300)`, therefore all detailed ORB parameters remain defaults and are not measurable in source. `[VERIFIED: repository source]`
- Installed EasyOpenCV `1.7.3` depends on OpenFTC OpenCV `4.10.0-A`. `[VERIFIED: local Gradle POM cache]`
- OpenCV 4.10 Java exposes the full ORB constructor with `nfeatures`, `scaleFactor`, `nlevels`, `edgeThreshold`, `firstLevel`, `WTA_K`, `scoreType`, `patchSize`, and `fastThreshold`. `[CITED: https://docs.opencv.org/4.10.0/javadoc/org/opencv/features2d/ORB.html]`
- OpenCV documents `HARRIS_SCORE` as more stable and `FAST_SCORE` as slightly faster. Preserve `HARRIS_SCORE` for accuracy-first tuning. `[CITED: https://docs.opencv.org/4.10.0/javadoc/org/opencv/features2d/ORB.html]`
- `BRUTEFORCE_HAMMING` is correct for default `WTA_K=2` ORB binary descriptors. `[CITED: https://docs.opencv.org/4.5.2/dc/dc3/tutorial_py_matcher.html]`
- Ratio filtering alone has no absolute descriptor-distance ceiling; ambiguous bad pairs can pass when both nearest distances are poor. `[VERIFIED: repository source]`
- Retention stops at first 80 passing pairs, not best 80 by distance or ratio margin. `[VERIFIED: repository source]`
- Duplicate `trainIdx` scene keypoints are not explicitly removed. `[VERIFIED: repository source]`

### Homography and center

- OpenCV `findHomography(..., RANSAC, threshold, mask)` returns an inlier/outlier mask; `perspectiveTransform` maps template points through the result. `[CITED: https://docs.opencv.org/4.x/d1/de0/tutorial_py_feature_homography.html]`
- Current code counts mask inliers but does not calculate inlier reprojection residuals. `[VERIFIED: repository source]`
- Current center is arithmetic mean of four projected corners. Under a projective transform, transform of template center is the direct projective center estimate; use `perspectiveTransform` on `(templateWidth/2, templateHeight/2)` instead. `[ASSUMED]`
- No finite, convexity, winding, edge-length, area, frame-bound, or aspect/skew check exists on projected corners. `[VERIFIED: repository source]`
- No spatial-spread test exists for matched template points, allowing a small local patch to determine whole-target homography. `[VERIFIED: repository source]`

### Temporal output

- A valid raw hit publishes immediately; no stable-frame gate or outlier confirmation exists. `[VERIFIED: repository source]`
- No smoothing exists; center noise passes directly to `dxPx`/`dyPx`. `[VERIFIED: repository source]`
- Miss hold does not refresh timestamp, which is correct for stale safety, but `Result` lacks an `observedThisFrame`/tracking state field. `[VERIFIED: repository source]`
- `fresh()` checks result age and finite offsets, but not pipeline state, processing latency, observation state, or confidence. `[VERIFIED: repository source]`

### Resource and speed findings

- `MatOfKeyPoint`, descriptor `Mat`, KNN result list, selected array, point arrays, point Mats, mask, homography, and corner Mat are allocated per qualifying frame. `[VERIFIED: repository source]`
- ROI uses `submat(...).copyTo(roi)`, creating a temporary header and full ROI copy each frame. `[VERIFIED: repository source]`
- Accuracy filters should run cheap-to-expensive: descriptor count, ratio/absolute distance, unique correspondences, spread, homography, inlier count/ratio, reprojection, quadrilateral, temporal gate. `[ASSUMED]`
- Do not optimize allocations before accuracy metrics identify processing-budget failure; current hardware issue is noisy output, not reported latency failure. `[VERIFIED: user report]`

## Standard Stack

| Component | Installed version | Use | Prescription |
|---|---:|---|---|
| FTC SDK Vision | `11.2.1` | FTC camera integration | Keep pinned. `[VERIFIED: build.dependencies.gradle]` |
| EasyOpenCV | `1.7.3` | Webcam lifecycle and pipeline | Keep installed API; no dependency change. `[VERIFIED: local Gradle POM cache]` |
| OpenFTC OpenCV | `4.10.0-A` | ORB, matcher, homography, perspective transform | Use existing native APIs only. `[VERIFIED: local Gradle POM cache]` |
| Java standard library | project toolchain | immutable result and deterministic tracker math | Use arrays/deques only; no filtering dependency. `[VERIFIED: repository build]` |

No package installation. Package legitimacy audit not applicable.

## Implementation-Ready Detection Policy

Values below are **starting bounds**, not universal truth. Keep each named and expose telemetry; tune against positive/negative fixtures and hardware recordings. `[ASSUMED]`

### 1. Explicit ORB configuration

Use the full constructor so policy is visible:

```java
ORB.create(400, 1.2f, 6, 31, 0, 2, ORB.HARRIS_SCORE, 31, 15)
```

- Start with `MAX_FEATURES=400`, `scaleFactor=1.2`, `nlevels=6`, `edgeThreshold=31`, `WTA_K=2`, `HARRIS_SCORE`, `patchSize=31`, `fastThreshold=15`. `[ASSUMED]`
- This spends modest extra work versus current 300/default configuration while retaining stable Harris ranking. `[ASSUMED]`
- If measured p95 processing exceeds budget, reduce features to 320 before weakening geometric gates. `[ASSUMED]`

### 2. Match-quality gate

Apply in this order:

1. KNN `k=2` with Hamming. `[CITED: https://docs.opencv.org/4.x/javadoc/org/opencv/features2d/DescriptorMatcher.html]`
2. Require `best.distance <= 64` and `best.distance < 0.72 * second.distance`. `[ASSUMED]`
3. Sort passing matches by ascending Hamming distance, then query/train index for deterministic ties. `[ASSUMED]`
4. Keep at most one match per `trainIdx`; cap after sorting at 80. `[ASSUMED]`
5. Require at least 12 quality matches before homography, not current 8. `[ASSUMED]`
6. Require template-point spatial coverage before homography: bounding-box area of matched template points at least 15% of template area and at least three of four template quadrants represented. `[ASSUMED]`

Do not add reverse KNN on first pass. Ratio + absolute distance + uniqueness + spatial spread + RANSAC geometry gives stronger rejection with one matcher direction; add mutual matching only if false positives survive and p95 latency has room. `[ASSUMED]`

### 3. Homography-quality gate

After `findHomography`:

- Require non-empty 3x3 homography and finite coefficients. `[ASSUMED]`
- Require at least 10 inliers and inlier ratio at least 0.60. `[ASSUMED]`
- Transform each inlier template point and compute Euclidean residual to scene point; require median residual at most 3 px and p90 residual at most 5 px in ROI/full-frame coordinates. `[ASSUMED]`
- Require inlier template coverage at least 12% and at least three quadrants after applying RANSAC mask; pre-RANSAC spread alone is insufficient. `[ASSUMED]`
- Transform four template corners and template center. Reject any non-finite point. `[ASSUMED]`
- Require projected corner polygon convex with consistent winding and absolute area between 1.5% and 60% of full frame. `[ASSUMED]`
- Require center within frame and corners within a configurable 10% frame margin; margin allows mild perspective clipping but rejects remote projections. `[ASSUMED]`
- Require every projected edge at least 12 px and max/min edge ratio at most 6.0; require diagonal ratio at most 4.0. `[ASSUMED]`
- Publish transformed template center, not corner average. `[ASSUMED]`

Use polygon shoelace area and cross-product convexity in plain Java. No extra OpenCV API or dependency needed. `[ASSUMED]`

### 4. Composite confidence

Do not use inlier ratio alone. Publish components and a bounded composite:

```java
quality = 0.40 * clamp01((inlierRatio - 0.50) / 0.35)
        + 0.25 * clamp01((inliers - 8.0) / 16.0)
        + 0.20 * clamp01((5.0 - medianReprojectionPx) / 4.0)
        + 0.15 * clamp01((64.0 - medianHamming) / 40.0);
```

This exact weighting is a deterministic starting policy requiring fixture/hardware calibration. `[ASSUMED]` Keep raw metrics in `Result`; composite confidence must never hide a failed hard gate. `[ASSUMED]`

## Adaptive Temporal Tracking Policy

### States

Use four states in one package-visible production helper: `SEARCHING`, `TENTATIVE`, `LOCKED`, `HOLDING`. `[ASSUMED]`

- `SEARCHING`: no usable published target.
- `TENTATIVE`: qualified raw detections exist but acquisition count is below threshold; result remains invalid for movement.
- `LOCKED`: filtered center is valid and observed or briefly held.
- `HOLDING`: no raw observation this frame; last filtered center is published for display/control continuity only while hold and freshness limits pass.

### Stable-frame acquisition

- Require 3 consecutive geometrically qualified detections. `[ASSUMED]`
- Consecutive acquisition centers must stay within `ACQUIRE_RADIUS_PX=18`; otherwise restart tentative sequence at current candidate. `[ASSUMED]`
- Initialize filtered center with median X and median Y of the 3 acquisition centers, reducing one-frame acquisition spikes. `[ASSUMED]`
- Acquisition never reuses miss-held output as a new observation. `[ASSUMED]`

### Adaptive smoothing

For a locked, continuity-qualified observation, compute distance `d` from prior filtered center and quality `q` in `[0,1]`:

```java
motion = clamp01(d / 30.0);
alpha = clamp(0.12 + 0.53 * motion + 0.15 * (1.0 - q), 0.12, 0.70);
filteredX += alpha * (rawX - filteredX);
filteredY += alpha * (rawY - filteredY);
```

Small motion gets strong smoothing; genuine movement responds faster. Lower-quality accepted observations get slightly faster correction in this initial formula, which can amplify noise; preferred accuracy-first form is `alpha = clamp(0.12 + 0.53 * motion + 0.15 * q, 0.12, 0.70)`, where low quality contributes less. Use preferred form. `[ASSUMED]`

Add a 2 px deadband only to control-facing `dxPx`/`dyPx`, not the tracked center or telemetry, so measurement remains honest. `[ASSUMED]`

### Outlier confirmation

- Continuity gate starts at `BASE_JUMP_PX=24` plus `2.5 * recentJitterPx`, capped at 60 px. `[ASSUMED]`
- Estimate `recentJitterPx` as median of last five accepted raw-to-filter residual distances; arrays and sorting suffice. `[ASSUMED]`
- Candidate outside gate does not replace locked output. Store as pending outlier. `[ASSUMED]`
- Accept relocation only after 2 consecutive qualified outliers within 18 px of each other; then reinitialize filter at their median. `[ASSUMED]`
- A single outlier increments rejection metrics and leaves prior observation timestamp unchanged. `[ASSUMED]`

### Miss hold and stale rejection

- Start with `MISS_HOLD_MS=120`, shorter than current 180 ms, and `MAX_RESULT_AGE_MS=250`. `[ASSUMED]`
- During hold, retain last filtered center but set `observedThisFrame=false`, state `HOLDING`, and preserve `lastObservedTimestampMs`; never timestamp held data as new. `[ASSUMED]`
- After 120 ms or 3 consecutive processed misses, whichever occurs first, publish invalid `SEARCHING`. `[ASSUMED]`
- Reject movement whenever state is not `LOCKED`, `observedAgeMs > 250`, values are non-finite, camera state is not `STREAMING`, or processing exceeds latency budget. `[ASSUMED]`
- Current `MAX_FRAME_LATENCY_MS` measures only processing unless a capture timestamp exists. Rename/report it as processing budget or add actual capture age only if EasyOpenCV supplies trustworthy frame timestamps; do not claim capture latency from completion timestamps. `[VERIFIED: repository source]`

## Result and Metrics Contract

Add fields only needed to tune and fail closed:

| Metric | Purpose |
|---|---|
| `rawCenterX/Y` | Separate detector noise from filter behavior. |
| `centerX/Y`, `dxPx/dyPx` | Filtered control-facing location. |
| `trackingState`, `observedThisFrame`, `stableFrames`, `missFrames` | Explain validity and gating. |
| `lastObservedTimestampMs`, `observedAgeMs` | Prevent held-result freshness refresh. |
| `keypointCount`, `knnCount`, `qualityMatchCount`, `uniqueMatchCount` | Diagnose descriptor stages. |
| `inlierCount`, `inlierRatio` | Diagnose RANSAC support. |
| `medianHamming`, `medianReprojectionPx`, `p90ReprojectionPx` | Diagnose match and geometry quality. |
| `projectedAreaRatio`, `templateCoverageRatio` | Diagnose implausible/clustered geometry. |
| `rawJitterPx`, `filteredJitterPx` | Quantify stabilization. |
| `rejectionReason` | Enum first failed hard gate; deterministic telemetry. |
| `processingMs`, `fps`, `overBudgetCount` | Preserve speed acceptance. |

`OrbTarget1TestOpMode` should display compact groups: tracking state/age, raw versus filtered center, raw versus filtered jitter, matches/inliers/ratio, reprojection/area/coverage, confidence/rejection reason, processing p50/p95/max and FPS. `[ASSUMED]` Do not print every match or allocate formatted debug images per frame. `[ASSUMED]`

## Deterministic Test Architecture

Current `CameraContinuationTest` checks constants and null freshness only; it does not test geometry, filtering, outliers, misses, or stale behavior. `[VERIFIED: repository source]`

Keep one dependency-free production helper accepting scalar `Observation` values. Native OpenCV integration can convert homography output to this helper; tests invoke same policy, not a copied filter. `[ASSUMED]`

### Required deterministic sequences

1. **Geometry pass:** rectangular projected corners, broad inlier spread, 12 matches/10 inliers, low residual; expect accepted raw observation. `[ASSUMED]`
2. **Cluster reject:** 20 inliers confined to one template corner; expect `INSUFFICIENT_COVERAGE`. `[ASSUMED]`
3. **Quad reject:** non-convex, tiny, huge, off-frame, NaN, short-edge, and extreme-skew polygons; exact rejection enum asserted. `[ASSUMED]`
4. **Reprojection reject:** high inlier ratio but median/p90 residual over bounds; expect invalid. `[ASSUMED]`
5. **Acquisition gate:** two good frames remain tentative; third coherent frame locks at median center. `[ASSUMED]`
6. **Jitter suppression:** fixed input sequence around `(320,240)`; assert filtered RMS displacement is below raw RMS and final error remains bounded. `[ASSUMED]`
7. **Real movement response:** monotonic 5 px/frame sequence; assert no outlier rejection and filtered lag below a named bound. `[ASSUMED]`
8. **Single spike:** one 120 px jump between coherent observations; assert locked center unchanged except normal smoothing and rejection count increments. `[ASSUMED]`
9. **Confirmed relocation:** two coherent large jumps; assert relocation only on second outlier. `[ASSUMED]`
10. **Miss hold:** one/two timed misses enter `HOLDING`, preserve last-observed timestamp, and never claim observed. `[ASSUMED]`
11. **Hold expiry:** time or miss-count boundary invalidates exactly at configured limit. `[ASSUMED]`
12. **Stale clock:** future timestamp, over-age timestamp, stop/error state, non-finite center, and over-budget processing reject. `[ASSUMED]`
13. **Determinism:** same observations and timestamps produce byte-for-byte equal scalar results and rejection states. `[ASSUMED]`
14. **Native fixture smoke:** when Android/OpenCV runtime is available, run one positive transformed template and several negative images through production matcher; plain JVM policy tests remain required even if native fixture execution needs Gradle Android classpath. `[ASSUMED]`

### Test commands

| Property | Value |
|---|---|
| Framework | Plain Java `main` + `AssertionError`; no new dependency. `[VERIFIED: CONTEXT D-04]` |
| Compile | `.\gradlew.bat :TeamCode:compileDebugJavaWithJavac --offline` |
| Policy test | Existing `:TeamCode:cameraContinuationTest` task after it invokes new production seam. |
| Hardware | Run `ORB Target 1 Test` only after compile/policy tests pass. |

### Hardware acceptance metrics

Record at least three conditions: target stationary/centered, robot or target moving laterally, and negative scene without target. `[ASSUMED]` For each, collect at least 300 processed frames after warm-up. `[ASSUMED]`

Initial acceptance targets:

- Positive stationary: at least 95% qualified observations after acquisition; no accepted center jump above 35 px; filtered center p95 radial jitter at most 4 px. `[ASSUMED]`
- Negative scene: zero locked acquisitions across test sample. `[ASSUMED]`
- Movement: p95 filtered lag below 40 px at observed bench speed and no stale result authorizes movement. `[ASSUMED]`
- Speed: p95 processing at most 100 ms, no sustained over-budget run longer than 3 frames. `[ASSUMED]`

Tune hard gates using false positives first, then recover recall. Never lower geometry gates solely to improve visible detection rate. `[ASSUMED]`

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---|---|---|---|
| Feature descriptor | Custom binary descriptor | Installed OpenCV ORB | Existing native implementation. `[CITED: https://docs.opencv.org/4.10.0/javadoc/org/opencv/features2d/ORB.html]` |
| Robust planar transform | Custom least-squares transform | `Calib3d.findHomography(...RANSAC..., mask)` | Supplies robust estimate and inlier mask. `[CITED: https://docs.opencv.org/4.x/d1/de0/tutorial_py_feature_homography.html]` |
| Corner/center mapping | Manual homography equations | `Core.perspectiveTransform` | Installed API already handles projective mapping. `[CITED: https://docs.opencv.org/4.x/d1/de0/tutorial_py_feature_homography.html]` |
| Heavy tracker | Kalman dependency or second vision architecture | Small adaptive EMA state machine | Scope needs scalar center stability, not a new subsystem. `[ASSUMED]` |
| Test framework | New JUnit/image framework | Existing main/assertion task | Locked no-dependency policy. `[VERIFIED: CONTEXT D-04]` |

## Common Pitfalls

1. **Confidence remains inlier ratio.** High ratio from a tiny clustered patch looks strong. Publish descriptor, spread, residual, and geometry metrics separately. `[ASSUMED]`
2. **Smoothing before rejection.** EMA does not make false locations safe; geometry and continuity gates must run first. `[ASSUMED]`
3. **Held result gets new timestamp.** This converts misses into indefinitely fresh movement data. Preserve observation timestamp. `[ASSUMED]`
4. **Corner average used as projective center.** Transform template center directly. `[ASSUMED]`
5. **Stable-frame gate blocks real movement.** Coherence compares inter-frame displacement against acquisition/continuity radius; use separate acquisition and locked policies. `[ASSUMED]`
6. **Outlier threshold fixed too tightly.** Fast genuine motion gets rejected. Adapt gate using recent jitter and confirm coherent relocation. `[ASSUMED]`
7. **Tuning only on positive target.** Accuracy claim needs hard negatives, repetitive textures, partial target, blur, glare, and scale/perspective cases. `[ASSUMED]`
8. **Telemetry changes timing.** Keep scalar metrics and aggregate percentiles; avoid per-frame image drawing or verbose logs. `[ASSUMED]`
9. **Unused latency constant treated as enforcement.** Add actual gate/counter; current code only declares it. `[VERIFIED: repository source]`
10. **Tests assert constants only.** Feed exact sequences into production policy and assert transitions/numbers. `[VERIFIED: repository test source]`

## Recommended Task Boundaries

1. **Production policy seam and geometry helpers:** Modify only `SingleTargetCamera.java`; add match sorting/uniqueness, spread, reprojection, quad checks, transformed center, structured rejection metrics, tracker state, adaptive EMA, outlier confirmation, miss hold, and stale gate. Keep lifecycle intact. `[ASSUMED]`
2. **Deterministic tests:** Modify existing camera assertion test or add one ORB-specific main-style test under existing test package; invoke production scalar policy. No copied algorithm and no new dependency. `[ASSUMED]`
3. **Hardware telemetry and tuning:** Modify only `OrbTarget1TestOpMode.java`; expose metrics, gather fixed-condition samples, tighten/relax named constants from evidence. `[ASSUMED]`

No task should touch `TemplateMatchCamera`, `OrbTemplateCamera`, UART, lifting, four-target orchestration, or unrelated camera consumers. `[VERIFIED: user request]`

## Project Constraints (from `.cursor/rules/`)

No project-local `.cursor/rules/` directory exists. `[VERIFIED: workspace glob]` Workspace-global rules require minimal diff, existing dependencies/stdlib first, no speculative abstractions, read before edit, and one runnable check for non-trivial logic. `[VERIFIED: injected workspace rules]`

## Environment Availability

| Dependency | Required By | Available | Version | Fallback |
|---|---|---:|---:|---|
| FTC SDK Vision | compile/hardware | yes | `11.2.1` | none `[VERIFIED: Gradle file]` |
| EasyOpenCV | webcam pipeline | cached/compiled in repository history | `1.7.3` | none `[VERIFIED: local POM cache]` |
| OpenCV Java | ORB/homography | cached through EasyOpenCV | `4.10.0-A` | none `[VERIFIED: local POM cache]` |
| FTC webcam/Robot Controller | hardware acceptance | user reports current ORB runs | hardware-specific | deterministic policy tests `[VERIFIED: user report]` |

Build execution was not rerun during research because required Gradle wrapper command was blocked by available shell allowlist; prior Phase 7 artifacts report compile/offline pass, but planner should require a fresh compile after implementation. `[VERIFIED: tool result and planning artifacts]`

## Security Domain

Vision output crosses into movement control, so stale, false, or discontinuous centers are safety-relevant input-validation failures. `[ASSUMED]`

| ASVS Category | Applies | Standard control |
|---|---:|---|
| V2 Authentication | no | No authentication boundary. |
| V3 Session Management | no | No session boundary. |
| V4 Access Control | yes | Only current streaming camera result can authorize movement. `[ASSUMED]` |
| V5 Input Validation | yes | Descriptor, homography, geometry, finite, temporal, freshness, and state gates. `[ASSUMED]` |
| V6 Cryptography | no | No cryptographic operation. |

Threat patterns: false correspondence is spoofed target evidence; stale hold is replay; excessive per-frame allocation/feature work is denial of service. Mitigate with hard qualification, preserved observation age, bounded work, metrics, and fail-closed transitions. `[ASSUMED]`

## Assumptions Log

| # | Claim | Risk if wrong |
|---|---|---|
| A1 | Proposed numeric thresholds fit `target1.png`, lens, distance, and lighting. | Hardware/fixture sweep must adjust values. |
| A2 | Transforming template center best represents desired pallet center. | Template crop may not align with physical centering point; add configured anchor point if measured. |
| A3 | Adaptive EMA gives enough response without a velocity model. | Fast motion may require alpha-beta filter later, based on measured lag. |
| A4 | 3-frame acquisition and 2-frame relocation confirmation fit control latency. | Adjust only from FPS and motion evidence. |
| A5 | Plain-Java scalar policy seam can compile/run in current Gradle task. | Build task/classpath may need narrow adjustment. |
| A6 | 100 ms processing p95 remains reasonable on target hardware. | Use measured p95; reduce ORB features before weakening safety gates. |

## Open Questions

1. What stationary raw-center p50/p95 jitter, positive detection rate, negative false-lock count, and processing p95 does current hardware produce? These measurements choose final thresholds.
2. Does `target1.png` crop center equal physical fork alignment point? If not, transform a configured template anchor instead of geometric center.
3. What target scale and perspective range occurs at centering distance? Use it to tighten projected area and skew limits.
4. Can deterministic positive/negative frame fixtures be committed from hardware? Without them, native matching accuracy remains hardware-only evidence.

## Sources

### Primary repository evidence

- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/SingleTargetCamera.java` — current matching, geometry, hold, freshness, allocation, and metric behavior.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/OrbTarget1TestOpMode.java` — current hardware telemetry.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/CameraContinuationTest.java` — current assertion coverage.
- Phase 7 context, research, plans, summaries, validation, and verification — stale architecture versus current replanning scope.
- `build.dependencies.gradle` and local Gradle POM cache — FTC/EasyOpenCV/OpenCV versions.

### Official documentation

- [CITED: https://docs.opencv.org/4.10.0/javadoc/org/opencv/features2d/ORB.html] — installed-version ORB constructor and stability/speed parameters.
- [CITED: https://docs.opencv.org/4.x/javadoc/org/opencv/features2d/DescriptorMatcher.html] — KNN matching API and ordered distances.
- [CITED: https://docs.opencv.org/4.x/d1/de0/tutorial_py_feature_homography.html] — ratio filtering, minimum correspondences, RANSAC mask, homography, and perspective transform.
- [CITED: https://docs.opencv.org/4.5.2/dc/dc3/tutorial_py_matcher.html] — Hamming matcher for ORB binary descriptors.

## Metadata

**Confidence breakdown:**
- Current-code diagnosis: HIGH — direct source inspection.
- Installed API surface: HIGH — local dependency metadata plus OpenCV 4.10 official Javadocs.
- Architecture/task scope: HIGH — explicit user restriction.
- Numeric starting thresholds: MEDIUM/LOW — reasoned initial values marked `[ASSUMED]`; fixture and hardware calibration required.
- Temporal policy: MEDIUM — deterministic and testable, but response/jitter targets need measured FPS and movement.

**Research date:** 2026-08-08
**Valid until:** 2026-09-07, or immediately after template/lens/resolution changes.
