# Phase 8: Unified Hardened ORB Planar Tracking - Research

**Researched:** 2026-08-08  
**Domain:** FTC EasyOpenCV/OpenCV planar ORB tracking on constrained Control Hub hardware  
**Confidence:** HIGH for current-code/API findings; MEDIUM for architecture; LOW for untuned numeric hardware controls

<user_constraints>
## User Constraints

### Locked Decisions

- Merge `SingleTargetCamera` geometry behavior into `OrbTemplateCamera`; stop divergent ORB behavior. `[VERIFIED: user-supplied Phase 8 architecture]`
- Replace confidence-as-validity with explicit hard validity gates. `[VERIFIED: user-supplied Phase 8 architecture]`
- Match with KNN Hamming, ratio `0.70`, absolute Hamming `50–60`, and mutual matching. `[VERIFIED: user-supplied Phase 8 architecture]`
- Require at least 12 good matches, 10 inliers, `0.55` inlier ratio, RANSAC near `3 px`, median reprojection at most `3 px`, broad template coverage/quadrants, and plausible convex/area/frame-bounded projected quadrilateral. `[VERIFIED: user-supplied Phase 8 architecture]`
- Track `SEARCHING`, `LOCKED`, `COASTING`, and `LOST`; acquire after 3 coherent frames and lose after 3 misses. `COASTING` never authorizes movement. Movement-facing observation age must be at most `100–120 ms`. `[VERIFIED: user-supplied Phase 8 architecture]`
- Apply median-of-3, EMA, and velocity gating only after geometric verification. `[VERIFIED: user-supplied Phase 8 architecture]`
- Search full/large ROI every 2 frames; while tracking, process projected bounding box expanded by `1.5` every frame. `[VERIFIED: user-supplied Phase 8 architecture]`
- Extract ORB once per physical webcam/frame for multi-target matching. `[VERIFIED: user-supplied Phase 8 architecture]`
- Rank qualified targets by inliers, inlier ratio, reprojection error, and coverage. `[VERIFIED: user-supplied Phase 8 architecture]`
- Fix manual exposure, gain, and focus. `[VERIFIED: user-supplied Phase 8 architecture]`
- Design for RK3328/1 GB Control Hub constraints. `[VERIFIED: user-supplied Phase 8 architecture]`
- Identify offline-testable behavior separately from hardware-only acceptance. Do not edit production source during research. `[VERIFIED: user request]`

### Existing Project Constraints

- Keep FTC SDK, EasyOpenCV, OpenCV, Java, `webcam1`, and `webcam2` as implementation boundary; add no dependency. `[VERIFIED: repository build and prior Phase 7 context]`
- Preserve INIT-time async opening, generation-safe callbacks, idempotent stop, native resource release, and stale/error/closed fail-closed behavior. `[VERIFIED: Phase 7 context and current source]`
- Plain-Java `main`/assertion tests remain project test convention. `[VERIFIED: TeamCode/build.gradle and current tests]`

### Deferred Ideas (OUT OF SCOPE)

- AprilTags, neural detection, transport protocol changes, lifting/autonomous behavior, and unrelated camera implementations. `[VERIFIED: REQUIREMENTS.md and Phase 7 scope]`
</user_constraints>

<phase_requirements>
## Phase Requirements

Phase 8 currently says `Requirements: TBD`; planner must update roadmap/traceability before execution. `[VERIFIED: .planning/ROADMAP.md]`

| Candidate ID | Existing description supported by Phase 8 | Research support |
|---|---|---|
| VIS-01 | Shared two-mode lifecycle and one immutable template per logical tracker | Two physical capture sessions fan one extracted frame-feature set to logical per-template trackers. `[VERIFIED: REQUIREMENTS.md; ASSUMED architecture]` |
| VIS-02 | `webcam1` centering authority | LOCKED/current-observation-only movement predicate and filtered center. `[VERIFIED: REQUIREMENTS.md; user architecture]` |
| VIS-03 | Center, error, quality, validity, age | Explicit quality metrics and observation timestamp replace ambiguous confidence. `[VERIFIED: REQUIREMENTS.md; user architecture]` |
| VIS-04 | Both webcams classify through target instances | Shared per-webcam extraction plus per-target geometric verification. `[VERIFIED: REQUIREMENTS.md; user architecture]` |
| VIS-05 | Deterministic ranking and stable temporal behavior | Lexicographic ranking and per-target state machines. `[VERIFIED: REQUIREMENTS.md; user architecture]` |
| VIS-06 | Invalid frame/template and lifecycle rejection | Fail-closed frame, geometry, camera-control, freshness, and lifecycle gates. `[VERIFIED: REQUIREMENTS.md; current source]` |
| VIS-07 | Measurable bounded ORB policy | Named limits, dynamic ROI, one extraction, fixed controls, latency counters, and hardware budget. `[VERIFIED: REQUIREMENTS.md; user architecture]` |
| TEST-01 | Offline center/ranking/threshold/stale tests | Pure scalar geometry/tracker/ranking/ROI policy seams plus Android-native fixture and hardware gates. `[VERIFIED: REQUIREMENTS.md; current test structure]` |
</phase_requirements>

## Summary

Current quick-task code has correctly consolidated homography into `OrbTemplateCamera` and removed `SingleTargetCamera`, but it is not safe or complete enough to plan as finished. `[VERIFIED: current source and quick-task summary]` It performs forward/reverse KNN Hamming, ratio `0.70`, Hamming `60`, mutual filtering, RANSAC `3 px`, inlier/reprojection/coverage/quadrant/quad checks, median-3/EMA intent, and four detection states. `[VERIFIED: OrbTemplateCamera.java]` However, Java `NaN` comparisons (`filteredX == Double.NaN` and `filteredX != Double.NaN`) make filter initialization impossible and condition checks incorrect; the filtered value stays `NaN`, so `fresh()` rejects movement output. `[VERIFIED: OrbTemplateCamera.java and Java floating-point semantics]`

Current four-target orchestration constructs four webcam-owning `OrbTemplateCamera` objects, including two independent opens for each physical webcam. `[VERIFIED: FourTargetCameraOrchestrator.java]` This duplicates grayscale/ORB extraction and competes for the same USB camera instead of sharing a frame. `[VERIFIED: current architecture]` Current static center crop cannot reacquire outside its 480×360 center region, target ranking does not exist, fixed controls do not exist, tests mostly assert constants, and `MultiTargetCameraTest` copies a separate policy that production does not use. `[VERIFIED: current source/tests]`

**Primary recommendation:** Keep one authoritative `OrbTemplateCamera` target-evaluation/tracker policy, but move physical capture and ORB extraction to exactly one session per webcam; fan immutable frame features to per-template trackers, then rank only geometrically qualified observations. `[ASSUMED: implementation architecture derived from user constraints and OpenCV PlaneTracker sample]`

## Architectural Responsibility Map

| Capability | Primary tier | Secondary tier | Rationale |
|---|---|---|---|
| Webcam lifecycle and fixed controls | One physical `WebcamSession` per webcam | EasyOpenCV/FTC UVC controls | Camera can be opened and configured only once. `[CITED: EasyOpenCV OpenCvWebcam implementation]` |
| Grayscale, ROI selection, frame ORB | Physical webcam pipeline | OpenCV ORB | Extraction must run once per physical frame. `[VERIFIED: user architecture]` |
| Immutable template features | Logical `OrbTemplateCamera` target model | OpenCV ORB | One target owns precomputed keypoints/descriptors. `[VERIFIED: REQUIREMENTS.md]` |
| Match and geometric verification | `OrbTemplateCamera` evaluator | OpenCV matcher/calib3d | Each target needs independent correspondences/homography from shared frame descriptors. `[ASSUMED]` |
| Per-target temporal tracking | `OrbTemplateCamera` scalar tracker | Java stdlib | State must survive frames independently for each target. `[ASSUMED]` |
| Multi-target ranking | `FourTargetCameraOrchestrator` | Target results | Ranking is cross-target policy, not descriptor extraction. `[ASSUMED]` |
| Movement authorization | One shared result predicate | Consumer | Only current observed `LOCKED` result on `webcam1` single-target role may move. `[VERIFIED: user architecture and REQUIREMENTS.md]` |
| Offline validation | Production scalar policy seams | Plain-Java executable test | Tests must avoid copied policy. `[VERIFIED: Phase 7 context]` |
| Native/hardware validation | Android/OpenCV fixture plus FTC OpMode | Human bench run | Webcam controls, USB ownership, native ORB timing, and physical image quality require target hardware. `[ASSUMED]` |

## Current Code Diagnosis

### Critical correctness defects

1. `filteredX == Double.NaN` is always false and `filteredX != Double.NaN` is always true; use `Double.isNaN(filteredX)` or `Double.isFinite(filteredX)`. `[VERIFIED: OrbTemplateCamera.java and Java semantics]`
2. Because initialization never runs, EMA computes from `NaN`; LOCKED results can be marked valid with non-finite center, although `fresh()` later rejects them. `[VERIFIED: OrbTemplateCamera.java]`
3. `quadSane()` compares projected area to template pixel area rather than frame area, making accepted scale depend on template asset dimensions instead of camera occupancy. `[VERIFIED: OrbTemplateCamera.java]`
4. `isConvex()` uses one boolean as both “sign initialized” and sign value; negative cross products repeatedly look uninitialized and mixed winding can be misclassified. `[VERIFIED: OrbTemplateCamera.java]`
5. Projected center is mean of four projected corners, not `perspectiveTransform(template center)`. `[VERIFIED: OrbTemplateCamera.java]`
6. Homography coefficients and every projected point are not explicitly checked finite before publication. `[VERIFIED: OrbTemplateCamera.java]`
7. Velocity limit `180 px/s` equals roughly `6 px/frame` at 30 FPS, and uses wall-clock frame completion interval; final value needs measured motion/FPS or it may reject normal lateral movement. `[VERIFIED: OrbTemplateCamera.java; ASSUMED hardware impact]`
8. Matching keeps forward matches in template-keypoint iteration order and caps before quality sorting, so retained 80 are not guaranteed best 80. `[VERIFIED: OrbTemplateCamera.java]`
9. `confidence` still means only inlier ratio and is copied through COASTING; hard gates prevent some misuse, but field semantics remain misleading for ranking and telemetry. `[VERIFIED: OrbTemplateCamera.java]`
10. COASTING output receives a new result timestamp each processed frame, while no separate `lastObservedTimestampMs` exists. Movement is currently blocked by state/validity, but observation age cannot be measured honestly. `[VERIFIED: OrbTemplateCamera.java]`

### Architecture defects

- `FourTargetCameraOrchestrator` opens `webcam1` twice and `webcam2` twice. `[VERIFIED: FourTargetCameraOrchestrator.java]`
- Every instance owns grayscale buffers, ORB, frame descriptors, matcher work, and lifecycle state; no physical-camera sharing exists. `[VERIFIED: OrbTemplateCamera.java and FourTargetCameraOrchestrator.java]`
- No caller uses `FourTargetCameraOrchestrator` outside constant tests, so current multi-target path has no integration evidence. `[VERIFIED: repository grep]`
- Only `target1.png` is committed; four-target hardware acceptance cannot run until target assets/mappings exist. `[VERIFIED: repository assets and ORB structure map]`
- `ORB-ALGORITHM-STRUCTURE.md` and several Phase 7 artifacts still describe deleted `SingleTargetCamera`, so planner must treat them as historical evidence, not current source truth. `[VERIFIED: repository grep]`

## Standard Stack

### Core

| Component | Version | Purpose | Prescription |
|---|---:|---|---|
| FTC SDK | `11.2.1` | Hardware map and UVC controls | Keep pinned. `[VERIFIED: build.dependencies.gradle]` |
| EasyOpenCV | `1.7.3` | Async webcam lifecycle and pipeline | Keep existing dependency. `[VERIFIED: Phase 7 dependency evidence and Maven Central]` |
| OpenFTC OpenCV | `4.10.0-A` | ORB, Hamming matcher, RANSAC homography | Keep existing dependency. `[VERIFIED: Phase 7 dependency evidence]` |
| Java stdlib | Project toolchain | Sorting, fixed arrays, immutable snapshots, scalar tracker | Add no library. `[VERIFIED: TeamCode build/test convention]` |

No package installation is needed; package legitimacy audit does not apply. `[VERIFIED: repository stack and user scope]`

## Recommended Architecture

### Data-flow diagram

```text
webcam1 USB ──> WebcamSession(webcam1, controls, lifecycle) ──> frame N
                                                           ├─> choose SEARCH/TRACK ROI
                                                           ├─> grayscale + ORB once
                                                           ├─> target1 evaluator/tracker
                                                           └─> target2 evaluator/tracker
                                                                    └─> deterministic rank
                                                                         └─> webcam1 Result
                                                                              └─> movement gate

webcam2 USB ──> WebcamSession(webcam2, controls, lifecycle) ──> same shared extraction pattern
                                                           ├─> target3 evaluator/tracker
                                                           └─> target4 evaluator/tracker
                                                                    └─> classification only
```

This structure follows OpenCV’s `PlaneTracker` pattern: extract current-frame features once, match against stored target descriptors, group/evaluate by target, and estimate per-target homography. `[CITED: https://github.com/opencv/opencv/blob/4.x/samples/python/plane_tracker.py]`

### Minimal component boundaries

1. **Physical session:** owns one `OpenCvWebcam`, lifecycle generation, controls, frame buffers, ORB extractor, selected ROI, and frame sequence. `[ASSUMED]`
2. **Logical target tracker:** `OrbTemplateCamera` owns one immutable template descriptor set, one matcher/evaluator policy, one temporal state, and one latest result; it does not open duplicate webcam handles when attached to shared session. `[ASSUMED]`
3. **Orchestrator:** constructs two sessions and four target trackers, assigns two targets per webcam, receives frame callbacks, and ranks per-webcam qualified results. `[ASSUMED]`
4. **Standalone test OpMode:** uses one physical session with one target tracker through same code path; no second production implementation. `[ASSUMED]`

Avoid static webcam registries or hidden singleton sharing; lifecycle ownership and release order become non-local and hard to test. `[ASSUMED]`

### Shared frame contract

Use one immutable frame-local view passed sequentially on EasyOpenCV pipeline thread. `[ASSUMED]`

```java
final class FrameFeatures {
    final long frameId;
    final long observedAtMs;
    final Rect roi;
    final KeyPoint[] keypoints;
    final Mat descriptors;
}
```

Do not retain `Mat` references past callback unless ownership is explicit; evaluate all targets before reusable buffers are overwritten. `[ASSUMED: OpenCV native resource safety]`

## Implementation-Ready Frame Policy

### SEARCH/TRACK ROI

- `SEARCHING` or `LOST`: process full frame or largest safe ROI every second frame; skipped frames publish no new observation and never refresh age. `[VERIFIED: user architecture]`
- `LOCKED`: project target quadrilateral, take axis-aligned bounding box, expand width/height by `1.5`, clamp to frame, and process every frame. `[VERIFIED: user architecture]`
- `COASTING`: use last qualified expanded TRACK ROI for at most two misses, but schedule large SEARCH ROI on next eligible search frame; never authorize movement. `[ASSUMED]`
- If expanded ROI becomes too small, non-finite, off-frame, or has too few descriptors, fall back to SEARCH policy rather than repeatedly processing empty ROI. `[ASSUMED]`
- Add ROI origin to scene keypoints before geometry; all published center/quad coordinates remain full-frame coordinates. `[VERIFIED: current source pattern]`
- Keep `frameId`, `roiMode`, `roiRect`, and `searchFrameSkipped` metrics so target ranking never compares observations from different frames as if simultaneous. `[ASSUMED]`

### Match gate, cheap to expensive

1. Require non-empty template and frame descriptors. `[ASSUMED]`
2. Run Hamming KNN `k=2` in both directions because ORB uses `WTA_K=2`. `[CITED: https://docs.opencv.org/4.5.2/dc/dc3/tutorial_py_matcher.html]`
3. Forward match passes only when pair size is 2, `best.distance <= MAX_HAMMING`, and `best.distance < 0.70 * second.distance`. `[VERIFIED: user architecture]`
4. Require reverse best match to map back to original template descriptor. `[VERIFIED: user architecture; CITED: OpenCV matcher tutorial]`
5. Sort by Hamming distance, then query index, then train index; deduplicate both template and scene indices; cap after sorting. `[ASSUMED]`
6. Start `MAX_HAMMING=60`; record distance distribution and test `50`, `55`, and `60` on fixtures/hardware before locking final value. `[VERIFIED: user range; ASSUMED tuning procedure]`
7. Require at least 12 retained correspondences. `[VERIFIED: user architecture]`
8. Require broad template support before RANSAC: at least 3 quadrants plus named X/Y or bounding-area coverage. `[VERIFIED: user architecture]`

OpenCV documents that `knnMatch` returns each query’s matches in increasing distance order. `[CITED: https://docs.opencv.org/4.x/javadoc/org/opencv/features2d/DescriptorMatcher.html]`

### Homography and geometry gate

- Use `Calib3d.findHomography(src, dst, Calib3d.RANSAC, 3.0, mask)`. `[VERIFIED: current API use; CITED: https://docs.opencv.org/4.x/d7/dff/tutorial_feature_homography.html]`
- Require non-empty `3×3` homography and all coefficients finite. `[ASSUMED]`
- Require at least 10 inliers and inlier ratio at least `0.55`. `[VERIFIED: user architecture]`
- Recompute inlier template coverage/quadrants from RANSAC mask; pre-RANSAC spread does not prove geometric support. `[ASSUMED]`
- Transform all inlier template points in one `MatOfPoint2f`, calculate Euclidean residuals, and require median at most `3 px`; avoid one native Mat allocation per point. `[VERIFIED: user architecture; current allocation audit]`
- Transform four corners and template center. Publish transformed template center, not corner mean. `[ASSUMED: projective-center policy]`
- Require all projected values finite, four corners convex with consistent non-zero winding, area within named fraction of frame area, center within frame, corners within a small configured margin, and bounded edge/diagonal ratios. `[VERIFIED: user architecture; ASSUMED exact bounds]`
- Initial area/bounds/skew numbers must remain tuning defaults until target scale/perspective is measured. `[ASSUMED]`

### Validity and quality semantics

Hard gates determine `geometryValid`; no score may override a failed gate. `[VERIFIED: user architecture]`

Replace ambiguous `confidence` with separate metrics:

| Field | Meaning |
|---|---|
| `goodMatches` | Count after descriptor/mutual/uniqueness gates. `[ASSUMED]` |
| `inliers` | RANSAC mask support. `[CITED: OpenCV homography docs]` |
| `inlierRatio` | `inliers / goodMatches`. `[ASSUMED]` |
| `medianHamming` | Descriptor quality diagnostic. `[ASSUMED]` |
| `medianReprojectionPx` | Geometric residual diagnostic. `[ASSUMED]` |
| `coverageX`, `coverageY`, `quadrants` | Spatial support. `[ASSUMED]` |
| `quadAreaRatio` | Projected target occupancy relative to full frame. `[ASSUMED]` |
| `rejectionReason` | First failed hard gate. `[ASSUMED]` |
| `qualityScore` | Ranking/display only; never validity. `[VERIFIED: user architecture]` |

Rank geometrically valid targets lexicographically, not with an uncalibrated weighted confidence: more inliers, higher inlier ratio, lower median reprojection, higher coverage, then stable target ID. `[VERIFIED: user architecture; ASSUMED deterministic tie-break]` Lexicographic ranking preserves metric meaning and avoids arbitrary unit mixing. `[ASSUMED]`

## Temporal Policy

### State transitions

| Current | Input | Next | Movement |
|---|---|---|---|
| SEARCHING/LOST | 1st or 2nd coherent qualified observation | SEARCHING | denied `[VERIFIED: user architecture]` |
| SEARCHING/LOST | 3rd coherent qualified observation | LOCKED | allowed only if current/fresh and webcam1 centering role `[VERIFIED: user architecture]` |
| LOCKED | qualified continuity-pass observation | LOCKED | eligible `[VERIFIED: user architecture]` |
| LOCKED | first miss/rejected observation | COASTING | denied `[VERIFIED: user architecture]` |
| COASTING | qualified continuity-pass observation | LOCKED | eligible only for new observation `[ASSUMED]` |
| COASTING | third consecutive miss | LOST | denied `[VERIFIED: user architecture]` |
| Any | lifecycle error/stop/non-finite clock | LOST | denied `[ASSUMED]` |

Keep `lastObservedTimestampMs` separate from `publishedTimestampMs`; COASTING must preserve observation timestamp. `[ASSUMED]` `fresh()` must require `STREAMING`, `LOCKED`, `observedThisFrame`, finite center/error, `now >= lastObservedTimestampMs`, age at most `120 ms`, and processing within budget. `[VERIFIED: user architecture; ASSUMED exact predicate]`

### Filter order

1. Geometric hard gates. `[VERIFIED: user architecture]`
2. Continuity/velocity gate against last accepted raw center using measured `dt`. `[VERIFIED: user architecture]`
3. Median X/Y of latest 3 accepted raw observations. `[VERIFIED: user architecture]`
4. EMA update from median output. `[VERIFIED: user architecture]`
5. Publish filtered center and movement predicate. `[ASSUMED]`

Use `Double.isFinite`, monotonic `System.nanoTime()` deltas for velocity/processing, and wall-clock milliseconds only for externally consumed result age if existing contract requires it. `[ASSUMED]` Clamp or reject zero/negative/large `dt`; do not let a scheduling pause permit an unlimited jump. `[ASSUMED]`

Start EMA `alpha=0.3` only as current baseline. `[VERIFIED: current source]` Velocity limit cannot be finalized without measured lateral speed and FPS. `[ASSUMED]`

## Fixed Camera Controls

EasyOpenCV `OpenCvWebcam` exposes `getExposureControl()`, `getGainControl()`, and `getFocusControl()` after camera open; implementation throws if control getters are called before open. `[CITED: https://github.com/OpenFTC/EasyOpenCV/blob/master/easyopencv/src/main/java/org/openftc/easyopencv/OpenCvWebcamImpl.java]`

Apply controls in `onOpened()` before or immediately after stream start, never in constructor. `[ASSUMED: EasyOpenCV lifecycle]`

```java
ExposureControl exposure = webcam.getExposureControl();
GainControl gain = webcam.getGainControl();
FocusControl focus = webcam.getFocusControl();

if (!exposure.isModeSupported(ExposureControl.Mode.Manual)
        || !exposure.isExposureSupported()) {
    throw new IllegalStateException("manual exposure unsupported");
}
exposure.setMode(ExposureControl.Mode.Manual);
exposure.setExposure(exposureMs, TimeUnit.MILLISECONDS);
gain.setGain(gainValue);

if (focus.isModeSupported(FocusControl.Mode.Fixed)
        && focus.isFocusLengthSupported()) {
    focus.setMode(FocusControl.Mode.Fixed);
    focus.setFocusLength(focusLength);
}
```

Source APIs and support checks are documented by FTC webcam-control docs. `[CITED: https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/visionportal_camera_controls/eval/eval.html]`

Implementation must validate configured values against reported min/max, check boolean setter returns, read values/modes back, and publish `controlsApplied` plus actual values. `[CITED: FTC webcam-control evaluation docs; ASSUMED fail-closed policy]` Webcam firmware may report or accept unsupported settings inconsistently, and some webcams do not support fixed focus. `[CITED: FTC webcam-control evaluation docs]` Therefore exact exposure/gain/focus values and whether unsupported focus blocks operation are hardware decisions; recommendation is to block movement authorization until required exposure/gain are verified, while report-and-continue for unsupported fixed focus only if hardware acceptance proves focus is physically fixed. `[ASSUMED]`

## Control Hub Performance Policy

RK3328 provides four Cortex-A53 cores, but phase hardware is user-specified as 1 GB RAM; native allocation churn and duplicate camera sessions remain primary avoidable costs. `[CITED: RK3328 datasheet; VERIFIED: user hardware constraint]`

Prescriptions:

- Exactly one webcam open and one ORB extraction per physical camera/frame. `[VERIFIED: user architecture]`
- Process target evaluators sequentially on pipeline thread first; do not add unsafe worker threads until profiling proves need. `[ASSUMED]`
- Reuse grayscale, ROI, keypoint, descriptor, homography input/output, and residual buffers where OpenCV Java ownership permits. `[ASSUMED]`
- Cap ORB features, levels, retained matches, targets per webcam, and ROI area. `[VERIFIED: existing VIS-07 and current constants]`
- Drop frame backlog through EasyOpenCV’s callback model; never queue Mats. `[ASSUMED]`
- Disable preview for production if measured viewport overhead matters; preserve preview only for tuning OpMode. `[ASSUMED]`
- Measure total shared-extraction time and per-target match/geometry time separately. `[ASSUMED]`
- Keep initial movement processing budget `100 ms`, but require hardware p95 and no sustained overruns before acceptance. `[VERIFIED: current constant and user age range]`

## Don't Hand-Roll

| Problem | Do not build | Use instead | Why |
|---|---|---|---|
| Binary features | Custom descriptor | OpenCV ORB | Installed, bounded API. `[CITED: OpenCV ORB Javadocs]` |
| Descriptor distance | Manual XOR/popcount | `BRUTEFORCE_HAMMING` | Correct matcher for `WTA_K=2`. `[CITED: OpenCV matcher tutorial]` |
| Robust plane transform | Custom solver | `findHomography(...RANSAC..., mask)` | Produces robust estimate and inlier mask. `[CITED: OpenCV homography docs]` |
| Projective point mapping | Manual matrix division | `Core.perspectiveTransform` | Existing native API. `[CITED: OpenCV homography docs]` |
| General tracking framework | Kalman/filter dependency | Fixed median-3 + EMA + bounded state | Requested policy is small and deterministic. `[VERIFIED: user architecture]` |
| Camera-control wrapper library | New dependency | FTC `ExposureControl`, `GainControl`, `FocusControl` | Already available through EasyOpenCV. `[CITED: EasyOpenCV/FTC docs]` |
| Test framework | New JUnit/image package | Existing `cameraContinuationTest` plus narrow native/hardware checks | Existing no-dependency convention. `[VERIFIED: TeamCode/build.gradle]` |

## Common Pitfalls

1. **Logical target owns physical webcam.** Two targets reopen same USB device. Physical session must own webcam; target owns template/tracker only. `[VERIFIED: current defect; ASSUMED fix]`
2. **ROI shared across competing targets.** One target’s TRACK ROI can hide another target. In multi-target mode, choose ROI from current winning lock only and force periodic large SEARCH; otherwise process union bounded ROI or each target’s ROI from same full-frame extraction policy. `[ASSUMED]`
3. **Search skip counted as miss.** Intentional every-other-frame skip must not increment miss streak or refresh observation timestamp. `[ASSUMED]`
4. **Ranking includes failed geometry.** Rank only hard-valid observations from same webcam/frame. `[VERIFIED: user validity requirement]`
5. **Confidence authorizes movement.** Scores rank/display; hard gates and temporal predicate authorize. `[VERIFIED: user architecture]`
6. **COASTING timestamp refreshed.** Held data becomes apparently fresh. Preserve last observed time and deny movement. `[ASSUMED]`
7. **Smoothing before geometry.** False homography contaminates filter state. Keep filter after geometry and continuity. `[VERIFIED: user architecture]`
8. **NaN compared with equality.** Use `Double.isFinite`/`Double.isNaN`. `[VERIFIED: current defect]`
9. **Projected area compared to template area.** Use frame-area ratio for camera plausibility; use template coverage separately. `[VERIFIED: current defect; ASSUMED fix]`
10. **Manual controls set before open.** EasyOpenCV throws. Configure from open callback and verify readback. `[CITED: EasyOpenCV implementation]`
11. **Hardware settings copied across webcam models.** Support and ranges vary by firmware/model. Calibrate each named webcam. `[CITED: FTC webcam-control docs]`
12. **Offline test copies production algorithm.** Current `MultiTargetCameraTest` does this. Extract scalar production policy and test it directly. `[VERIFIED: MultiTargetCameraTest.java]`
13. **Per-inlier native allocation.** Current reprojection helper allocates/releases a `MatOfPoint2f` for every inlier. Transform all points once. `[VERIFIED: OrbTemplateCamera.java]`
14. **Search and track observations mixed without coordinates.** Always add ROI offset and retain full-frame center/quad. `[VERIFIED: current coordinate pattern]`

## Code Examples

### Deterministic ranking comparator

```java
Comparator<TargetObservation> BEST_FIRST =
        Comparator.comparingInt((TargetObservation o) -> o.inliers).reversed()
                .thenComparingDouble(o -> -o.inlierRatio)
                .thenComparingDouble(o -> o.medianReprojectionPx)
                .thenComparingDouble(o -> -o.coverage)
                .thenComparing(o -> o.targetId);
```

This is proposed project code, not an official OpenCV API. `[ASSUMED]`

### Safe observation freshness

```java
return result != null
        && result.cameraState == State.STREAMING
        && result.detectionState == DetectionState.LOCKED
        && result.observedThisFrame
        && result.geometryValid
        && result.controlsApplied
        && Double.isFinite(result.dxPx)
        && Double.isFinite(result.dyPx)
        && nowMs >= result.lastObservedTimestampMs
        && nowMs - result.lastObservedTimestampMs <= MAX_RESULT_AGE_MS
        && result.processingMs <= MAX_FRAME_LATENCY_MS;
```

This predicate encodes user-required fail-closed movement policy; exact controls requirement depends on hardware support decision. `[VERIFIED: user architecture; ASSUMED controls policy]`

## Runtime State Inventory

| Category | Items found | Action required |
|---|---|---|
| Stored data | None; camera classes contain no DB/shared-preference persistence. `[VERIFIED: camera-source grep]` | None. |
| Live service config | Robot Configuration stores `webcam1`/`webcam2` outside these source files. `[VERIFIED: HardwareMap names in source]` | Verify both names map to distinct physical webcams on hardware; no rename planned. `[ASSUMED]` |
| OS-registered state | None identified for camera class names. `[VERIFIED: repository scope]` | None. |
| Secrets/env vars | None referenced by camera sources. `[VERIFIED: camera-source grep]` | None. |
| Build artifacts | Deleted `SingleTargetCamera` may remain in stale build outputs until clean rebuild/install. `[ASSUMED]` | Run clean compile/APK install before hardware acceptance. |

After repository edits, remaining runtime state is webcam firmware controls: webcams may retain exposure/focus modes after unplugging. `[CITED: FTC webcam-control evaluation docs]` Always reapply and read back controls on each open. `[CITED: FTC webcam-control evaluation docs]`

## Environment Availability

| Dependency | Required by | Available | Version | Fallback |
|---|---|---:|---:|---|
| FTC SDK | compile/hardware | yes in Gradle config | `11.2.1` | none `[VERIFIED: build.dependencies.gradle]` |
| EasyOpenCV | camera pipeline | resolved by project history | `1.7.3` | none `[VERIFIED: Phase 7 evidence]` |
| OpenCV | ORB/homography | resolved by project history | `4.10.0-A` | none `[VERIFIED: Phase 7 evidence]` |
| Gradle wrapper | compile/test | present | project wrapper | none `[VERIFIED: workspace glob]` |
| Webcam/Control Hub | controls/timing | unavailable to research session | RK3328/1 GB user target | hardware checkpoint `[VERIFIED: user constraint]` |
| Four target assets | multi-target acceptance | incomplete | only `target1.png` found | add verified assets before hardware gate `[VERIFIED: repository assets]` |

Build/test execution was not possible through available command allowlist; planner must require fresh Gradle evidence after implementation. `[VERIFIED: command-tool result]`

## Validation Architecture

### Test framework

| Property | Value |
|---|---|
| Framework | Plain Java `main` with assertions; Android/OpenCV compile through Gradle. `[VERIFIED: TeamCode/build.gradle]` |
| Config | `TeamCode/build.gradle` task `cameraContinuationTest`. `[VERIFIED: TeamCode/build.gradle]` |
| Quick command | `.\gradlew.bat :TeamCode:cameraContinuationTest --offline` `[VERIFIED: existing task]` |
| Full compile command | `.\gradlew.bat :TeamCode:compileDebugJavaWithJavac :TeamCode:cameraContinuationTest --offline` `[VERIFIED: Phase 7 validation convention]` |

### Offline-testable without webcam

- NaN/finite handling, quad convexity/winding/area/bounds, percentile residuals, template coverage/quadrants, transformed-center scalar expectations, and rejection order. `[ASSUMED]`
- Match selection policy from synthetic scalar match records: ratio, Hamming, mutual relation, sort, uniqueness, cap, and minimum count. `[ASSUMED]`
- SEARCH/LOCKED/COASTING/LOST transitions, 3-frame acquire, 3-miss loss, no timestamp refresh, age boundary, velocity gate, median-3, EMA, and deterministic replay. `[ASSUMED]`
- ROI expansion/clamping, every-second-frame SEARCH scheduling, skipped-search-not-miss behavior, full-frame coordinate offsets, and fallback conditions. `[ASSUMED]`
- Multi-target lexicographic ranking, frame-ID equality requirement, tie-breaks, and webcam1-only movement authority. `[ASSUMED]`
- Lifecycle predicate from scalar state snapshots and camera-control status. `[ASSUMED]`

### Android/OpenCV fixture-testable without robot motion

- Template/frame ORB extraction, forward/reverse KNN semantics, homography/mask, projected center/quad, positive warped fixture, repetitive negative fixture, blur/brightness/perspective cases. `[ASSUMED]`
- One extraction count per physical frame using instrumented fake/shared session around production fan-out. `[ASSUMED]`
- Native resource release and repeated frame processing under Android test/runtime. `[ASSUMED]`

Plain desktop JVM may fail loading EasyOpenCV/OpenCV native classes; isolate scalar policy so core tests do not load webcam/native classes. `[VERIFIED: Phase 7 offline blocker]`

### Hardware-only validation

- Two distinct webcams can open concurrently once each; no duplicate ownership/USB failure. `[ASSUMED]`
- Exposure/gain/focus support, min/max, set return, readback, image stability, and retained modes. `[CITED: FTC docs; ASSUMED test requirement]`
- Stationary jitter, acquisition latency, lateral movement lag, velocity threshold, reacquisition after ROI loss, negative false locks, multi-target ranking, FPS, processing p50/p95/max, memory pressure, and stop/restart safety. `[ASSUMED]`
- At least 300 processed observations after warm-up per stationary, moving, negative, ROI-reacquisition, and multi-target condition. `[ASSUMED: prior Phase 7 acceptance pattern]`

### Phase requirement test map

| Requirement | Behavior | Test type | Automated command/file status |
|---|---|---|---|
| VIS-01/04 | One extraction and one open per webcam, multiple logical target trackers | integration/scalar | ❌ Wave 0 production seam and tests needed |
| VIS-02/03 | Qualified filtered center and fresh movement gate | scalar + hardware | ❌ current tests only constants |
| VIS-05 | Same-frame valid-target ranking | scalar | ❌ Wave 0 |
| VIS-06 | Lifecycle/control/invalid/stale fail closed | scalar + hardware | partial lifecycle source, ❌ behavioral tests |
| VIS-07 | ROI/control/performance metrics | scalar + hardware | ❌ Wave 0 |
| TEST-01 | Production policy behavior | plain Java | ❌ replace copied/constant-only tests |

### Sampling rate

- Per task: quick `cameraContinuationTest`. `[ASSUMED]`
- Per wave: compile plus full camera test. `[ASSUMED]`
- Phase gate: full compile/tests green, then blocking hardware acceptance. `[ASSUMED]`

### Wave 0 gaps

- [ ] Refactor production scalar policy so tests call actual match-selection, geometry, tracker, ROI, and ranking code. `[ASSUMED]`
- [ ] Replace constant-only `CameraContinuationTest` assertions with deterministic sequences. `[VERIFIED: current test weakness]`
- [ ] Remove or rewrite `MultiTargetCameraTest` copied policy to call production seam. `[VERIFIED: current copied policy]`
- [ ] Add native fixture path only if current Android/OpenCV runtime can execute it without a new dependency. `[ASSUMED]`
- [ ] Add all four target assets and explicit webcam/target mapping before multi-target hardware acceptance. `[VERIFIED: missing assets]`

## Security Domain

Vision output is safety-relevant untrusted input because it can command robot movement. `[ASSUMED]`

### Applicable ASVS categories

| ASVS category | Applies | Standard control |
|---|---:|---|
| V2 Authentication | no | No authentication boundary. `[ASSUMED]` |
| V3 Session Management | no | No user session. `[ASSUMED]` |
| V4 Access Control | yes | Only webcam1 centering role with current observed LOCKED result may authorize. `[VERIFIED: REQUIREMENTS.md and user architecture]` |
| V5 Input Validation | yes | Descriptor, geometry, finite, ROI, time, lifecycle, and controls gates. `[ASSUMED]` |
| V6 Cryptography | no | No cryptographic operation. `[ASSUMED]` |

### Threat patterns

| Pattern | STRIDE | Mitigation |
|---|---|---|
| Repetitive texture impersonates target | Spoofing | Mutual descriptor gate, coverage, RANSAC, reprojection, quad sanity, temporal acquisition. `[ASSUMED]` |
| Held center appears fresh | Replay | Separate last-observed time; COASTING denied. `[ASSUMED]` |
| Duplicate webcam sessions exhaust/lock USB | Denial of service | One physical session per webcam. `[VERIFIED: current risk; ASSUMED mitigation]` |
| ROI loses target permanently | Denial of service | Periodic large SEARCH and fallback. `[VERIFIED: user architecture]` |
| Auto camera controls alter descriptors | Tampering | Manual supported controls, readback, telemetry. `[ASSUMED]` |
| Native allocation churn causes latency | Denial of service | Buffer reuse, bounded work, p95 gate. `[ASSUMED]` |

## Project Constraints (from `.cursor/rules/`)

No project-local `.cursor/rules/`, `.cursor/skills/`, or `.agents/skills/` files exist. `[VERIFIED: workspace glob]` Applied workspace constraints: use existing dependencies/stdlib first, smallest working diff, no speculative abstraction, preserve safety/input validation, and leave one runnable check for non-trivial logic. `[VERIFIED: injected workspace rules]`

## Assumptions Log

| # | Claim | Risk if wrong |
|---|---|---|
| A1 | Physical-session plus logical-target split is simplest way to preserve one-template trackers and one extraction. | API reshaping may touch consumers more than expected. |
| A2 | Lexicographic ranking matches desired behavior better than weighted score. | User may require calibrated weighted aggregate. |
| A3 | Full/large SEARCH every second frame fits processing budget. | RK3328 hardware timing may require lower resolution, smaller large ROI, or staggered webcams. |
| A4 | `MAX_HAMMING=60`, EMA `0.3`, and current ORB 400/6 are viable starts. | Hardware may require tuning without weakening hard safety gates. |
| A5 | Unsupported fixed focus can be report-and-continue when lens is physically fixed. | User may require fail-closed for every control. |
| A6 | Frame callback serial fan-out is fast enough for two targets/webcam. | Profiling may require descriptor collection matcher or staggered target evaluation. |
| A7 | Transforming template geometric center equals physical fork alignment point. | Template may need named anchor point. |
| A8 | 100 ms processing and 120 ms observation age are compatible with actual FPS. | Hardware measurements may require tighter/lower-latency processing. |

## Open Questions

1. **What exact Phase 8 requirement IDs should be locked?**
   - Known: roadmap says `TBD`; work directly supports VIS-01 through VIS-07 and TEST-01. `[VERIFIED: ROADMAP/REQUIREMENTS]`
   - Recommendation: update Phase 8 requirement mapping before plan tasks. `[ASSUMED]`
2. **What are exact exposure, gain, and focus values per webcam?**
   - Known: APIs and support queries exist; ranges vary by firmware/model. `[CITED: FTC docs]`
   - Recommendation: add hardware calibration checkpoint; do not invent values. `[ASSUMED]`
3. **Does target asset center equal fork alignment point?**
   - Recommendation: measure; otherwise store immutable template anchor and transform it. `[ASSUMED]`
4. **How should multi-target ROI behave when two targets are simultaneously visible?**
   - Recommendation: periodic large shared extraction plus TRACK ROI for current winner; switch only through ranked temporal confirmation. `[ASSUMED]`
5. **Can four target assets be supplied before implementation validation?**
   - Known: only `target1.png` exists now. `[VERIFIED: repository assets]`
6. **What actual Control Hub p50/p95 timing and memory behavior results from both webcams?**
   - Recommendation: profile shared extraction before lowering geometric gates or adding threads. `[ASSUMED]`

## Sources

### Primary

- `OrbTemplateCamera.java`, `FourTargetCameraOrchestrator.java`, `OrbTarget1TestOpMode.java`, `CameraContinuationTest.java`, `MultiTargetCameraTest.java` — current behavior and defects. `[VERIFIED: repository source]`
- `.planning/ROADMAP.md`, `.planning/REQUIREMENTS.md`, `.planning/STATE.md`, Phase 7 artifacts, quick-task artifacts, and `ORB-ALGORITHM-STRUCTURE.md` — scope/history/constraints. `[VERIFIED: repository documents]`
- [OpenCV 4.10 ORB Javadocs](https://docs.opencv.org/4.10.0/javadoc/org/opencv/features2d/ORB.html) — explicit ORB Java API and parameter semantics. `[CITED: official docs]`
- [OpenCV DescriptorMatcher Javadocs](https://docs.opencv.org/4.x/javadoc/org/opencv/features2d/DescriptorMatcher.html) — KNN overloads and ordered results. `[CITED: official docs]`
- [OpenCV feature homography tutorial](https://docs.opencv.org/4.x/d7/dff/tutorial_feature_homography.html) — ratio matching, RANSAC, mask, and perspective transform. `[CITED: official docs]`
- [OpenCV PlaneTracker sample](https://github.com/opencv/opencv/blob/4.x/samples/python/plane_tracker.py) — one frame extraction feeding multiple targets. `[CITED: official OpenCV repository]`
- [EasyOpenCV webcam implementation](https://github.com/OpenFTC/EasyOpenCV/blob/master/easyopencv/src/main/java/org/openftc/easyopencv/OpenCvWebcamImpl.java) — control getters and open-state requirement. `[CITED: official project source]`
- [FTC webcam-control evaluation](https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/visionportal_camera_controls/eval/eval.html) — support, ranges, setters/readback, and firmware caveats. `[CITED: official docs]`

### Secondary

- RK3328 datasheet — Cortex-A53 architecture context. `[CITED: vendor datasheet]`

## Metadata

**Confidence breakdown:**
- Current code diagnosis: HIGH — direct source inspection. `[VERIFIED: repository source]`
- FTC/OpenCV API surface: HIGH/MEDIUM — official docs and project source; exact EasyOpenCV 1.7.3 generated Javadocs unavailable. `[CITED: official sources]`
- Shared extraction architecture: MEDIUM — user-locked goal plus official OpenCV sample, but project API refactor unimplemented. `[ASSUMED]`
- Numeric tracking/control values: LOW until fixtures and hardware measurements. `[ASSUMED]`
- Offline/hardware split: HIGH — direct dependency/runtime boundaries and prior blocker evidence. `[VERIFIED: repository evidence]`

**Research date:** 2026-08-08  
**Valid until:** 2026-09-07, or immediately after camera model, template assets, stream resolution, or OpenCV version changes. `[ASSUMED]`
