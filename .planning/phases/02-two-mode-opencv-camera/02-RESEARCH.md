# Phase 2: Two-Mode OpenCV Camera - Research

**Researched:** 2026-08-06
**Domain:** FTC Android Java, EasyOpenCV, OpenCV template matching
**Confidence:** HIGH for repository architecture; MEDIUM for OpenCV implementation details

## Summary

`TemplateMatchCamera` already owns webcam creation, async open, stream start, stop/close, reusable Mats, template loading, result publication through `AtomicReference`, temporal filtering, telemetry, and resource release. Phase 2 should extend this class rather than add camera classes. [VERIFIED: repository `TemplateMatchCamera.java`]

Current detection downsamples the frame, runs `TM_CCOEFF_NORMED`, repeatedly selects peaks from a cloned response map, masks a template-sized neighborhood, then chooses the candidate closest to frame center Y. This is a useful extraction primitive, but it currently returns one detection and does not expose explicit mode, stale age, or a multi-detection result. [VERIFIED: repository `TemplateMatchCamera.java`]

Use one shared lifecycle and pipeline. `SINGLE_TARGET` should select one left-pallet candidate using center-error policy and existing temporal hold/filter. `MULTI_TARGET` should scan all configured block templates, generate threshold-qualified candidates, apply deterministic overlap suppression, and return raw match confidence per retained detection. Keep OpenCV and existing Android/FTC dependencies; no new package. [VERIFIED: `PROJECT.md`, `REQUIREMENTS.md`, `ROADMAP.md`]

**Primary recommendation:** Add mode/config/result contracts first, then extract candidate generation and NMS as package-visible pure helpers so offline tests can cover them without camera hardware.

## Architectural Responsibility Map

| Capability | Primary Tier | Secondary Tier | Rationale |
|---|---|---|---|
| Camera lifecycle | FTC hardware / camera tier | Pipeline | Webcam ownership and async callbacks already live in `TemplateMatchCamera`. |
| Frame preprocessing and matching | Pipeline | OpenCV native | `TemplateMatchPipeline.processFrame` owns reusable Mats and frame latency. |
| SINGLE_TARGET centering | Pipeline result API | Autonomous caller | Pipeline computes target center, `dxPx`, `dyPx`, validity, confidence, and age; caller decides strafing. |
| MULTI_TARGET classification | Pipeline result API | Template registry/config | Pipeline emits label plus raw confidence for each retained candidate. |
| NMS/min-distance | Pipeline utility | Offline test | Deterministic geometry policy must be shared by runtime and tests. |
| Lifecycle/error state | Camera wrapper | Pipeline release | Wrapper owns OPENING/STREAMING/ERROR/CLOSED; pipeline must tolerate stop and frame races. |

## User Constraints

No `02-CONTEXT.md` was present. Locked constraints come from project requirements and roadmap: one shared `TemplateMatchCamera`; explicit `SINGLE_TARGET` and `MULTI_TARGET`; OpenCV template matching only; no deep-learning detector, AprilTag, or new vision dependency; camera mounted over left fork with one-block FOV; multi scan classifies both blocks before centering; single mode centers left pallet and mechanical coupling centers right fork. [VERIFIED: repository planning documents]

## Standard Stack

| Component | Version | Purpose | Guidance |
|---|---|---|---|
| Java / FTC SDK | Repository-pinned | OpMode, `HardwareMap`, `WebcamName` | Reuse existing Gradle/FTC stack. [VERIFIED: repository] |
| EasyOpenCV | Repository-pinned | `OpenCvWebcam`, `OpenCvPipeline`, async lifecycle | Reuse existing API and callback model. [VERIFIED: `TemplateMatchCamera.java`] |
| OpenCV Java API | Repository-pinned | `Imgproc`, `Core`, `Mat`, `Point`, `Scalar` | Use existing `TM_CCOEFF_NORMED`, masks, and reusable Mats. [VERIFIED: `TemplateMatchCamera.java`] |
| Java stdlib | Platform | `AtomicReference`, collections, immutable data | No dependency installation. [VERIFIED: source imports] |

**Installation:** None.

## Architecture Patterns

### Shared pipeline with mode policy

Add an enum such as `CameraMode { SINGLE_TARGET, MULTI_TARGET }` and immutable measurable config. Pass mode at construction or expose a mode setter that resets tracking state. `processFrame` remains common; `detect` dispatches to single or multi policy. Do not duplicate camera open/close callbacks. [VERIFIED: existing wrapper/pipeline split]

### SINGLE_TARGET precision path

1. Generate candidates from response map at `MIN_CONFIDENCE`.
2. Convert scaled coordinates back to full-frame coordinates before center math.
3. Rank by center error, with deterministic tie-breaks: confidence, then stable scan order.
4. Apply existing outlier rejection/EMA/hold only to selected target.
5. Publish immutable detection containing center, `dxPx = centerX - frame.cols()/2`, `dyPx = centerY - frame.rows()/2`, confidence, validity, and timestamp. Expose `staleAgeMs` or compute it from result timestamp plus last-success timestamp. [VERIFIED: existing center math/filter; [ASSUMED] exact API shape]

Do not steer inside camera code. Caller strafes until fresh valid `abs(dxPx)` is within configured center policy; mechanical coupling handles right pallet alignment. [VERIFIED: `PROJECT.md`, `REQUIREMENTS.md`]

### MULTI_TARGET classification path

Maintain a configured list of templates/labels (`target.png` through `target4.png` are current assets used by test OpModes). For each template: match, enumerate response peaks above threshold, create boxes/corners and raw confidence, then suppress overlaps. Across labels, retain candidates according to explicit policy: likely one best candidate per physical box, selecting highest raw confidence; document tie-break and minimum center distance. Return all retained detections, not only one `Detection`. [VERIFIED: current assets and `matchTemplate`; [ASSUMED] physical-box cross-template association needs confirmation]

### NMS policy

Use IoU threshold for overlapping boxes; optionally enforce center-distance threshold for near-identical boxes where IoU is unreliable. Sort candidates by descending raw confidence, then stable label/order; greedy keep, suppress if IoU exceeds configured `nmsIoU` or center distance is below configured `minDistancePx`. Never divide retained confidence by maximum; requirement explicitly demands raw confidence. [VERIFIED: `VIS-05`, `VIS-07`; [ASSUMED] exact threshold values]

### Lifecycle and thread safety

Keep callback state transitions identical for both modes. Make mode/template updates synchronized with frame reads, or publish immutable template snapshots. `latestResult` remains atomic. `stop()` must prevent post-release frame processing, release all Mats once, and tolerate repeated stop/error paths. Current `setTarget()` clears tracking then releases and reloads template; preserve this reset behavior when mode or template registry changes. [VERIFIED: source lifecycle/release code]

## Don't Hand-Roll

| Problem | Don't Build | Use Instead | Why |
|---|---|---|---|
| Camera lifecycle | Separate mode-specific camera classes | Existing `OpenCvWebcam` callbacks and wrapper | Prevent duplicated race/error handling. |
| Template matching | Custom pixel search or neural detector | `Imgproc.matchTemplate(... TM_CCOEFF_NORMED)` | Existing dependency and fixed-FOV constraint. |
| Thread publication | Ad-hoc shared mutable result | Immutable result objects + `AtomicReference` | Avoid torn reads between camera thread and OpMode. |
| NMS | Unbounded repeated peak loop | Bounded candidate count, reusable mask/Mat, deterministic greedy NMS | Protect FTC frame latency and allocations. |

## Common Pitfalls

- **Single result reused for multi mode:** classification loses second block. Return immutable list/array of detections. [VERIFIED: current API has one `detection`]
- **Confidence renormalized after NMS:** violates raw-confidence requirement. Preserve `mm.maxVal`. [VERIFIED: VIS-05]
- **Scaled/full-resolution mismatch:** compute center and corners consistently after multiplying peak coordinates by `FRAME_SCALE`. Add tests with known scaled coordinates. [VERIFIED: current downscale code]
- **Overlapping peaks become duplicates:** mask response neighborhood or use NMS; cap candidates to avoid pathological loops. [VERIFIED: current peak loop; VIS-05]
- **Wrong target selected during centering:** rank SINGLE_TARGET by center error, not confidence alone or Y only. [VERIFIED: roadmap success criterion; current code uses Y-only tie policy]
- **Stale result treated fresh:** result validity alone is insufficient during misses. Expose timestamp/age and test hold expiry at both frame-count and millisecond limits. [VERIFIED: existing hold constants and `lastSuccessfulDetectionMs`]
- **Race during `setTarget`/stop:** frame may observe released Mats/template. Synchronize template replacement and guard `running`; release outside/inside lock consistently. [VERIFIED: existing synchronized template access and running guard]
- **Magic thresholds:** place confidence, ROI/crop, IoU/min-distance, hold, and center deadband in named config/constants and telemetry. [VERIFIED: VIS-07]

## Code Examples

### Result contract direction

```java
public enum CameraMode { SINGLE_TARGET, MULTI_TARGET }

public static final class CameraResult {
    public final long timestampMs;
    public final Detection detection;
    public final List<Detection> detections;
    public final long staleAgeMs;
    public final boolean valid;
}
```

Use immutable copies; avoid exposing mutable OpenCV `Mat` objects. Exact API remains planner discretion. [ASSUMED]

### NMS test geometry

```java
List<Candidate> kept = suppressOverlaps(candidates, 0.50, 12.0);
assert kept.size() == 2;
assert kept.get(0).confidence == 0.91; // raw score retained
```

Test helper should use plain Java assertions or existing executable test style, not require camera hardware. [VERIFIED: `TemplateMatchCameraTest.java` style; [ASSUMED] exact helper names]

## Validation Architecture

| Property | Value |
|---|---|
| Framework | Plain Java executable tests; no unit-test framework detected for vision |
| Existing test | `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/TemplateMatchCameraTest.java` |
| Quick run | Existing project-specific Java/Gradle test invocation; verify with project build scripts |
| Full suite | Existing TeamCode Gradle verification task |

| Requirement | Behavior | Test |
|---|---|---|
| VIS-01 | Mode selection shares lifecycle | Construct pipeline/config; assert each mode dispatches without separate camera class |
| VIS-02 | Single target center precision | Synthetic candidate set: closest center wins; filter tracks lateral movement |
| VIS-03 | Center/error/confidence/validity/stale age | Assert fields, NaN/invalid miss, hold age and expiry |
| VIS-04 | Multi classification | Two labels/two boxes returned before centering |
| VIS-05 | Candidate threshold + NMS | Below-threshold removed; overlap collapsed; distinct boxes retained |
| VIS-06 | Invalid input and lifecycle | Empty template/frame, repeated stop, frame after release, open error |
| VIS-07 | Measurable constants | Assert configured threshold, ROI, NMS/min-distance, hold, center policy are surfaced |

Existing tests cover temporal jitter, smooth motion, spike rejection, miss hold, label stability, and target reset. Extend rather than copy filter logic; current test duplicates constants and filter implementation, so prefer extracting pure logic or add contract tests that exercise production helpers. [VERIFIED: `TemplateMatchCameraTest.java`]

## Security Domain

No network/authentication/data-storage surface. Input validation still required for asset names, empty/invalid templates, empty frames, and candidate bounds. Avoid unbounded candidate loops and allocations because camera frames are untrusted runtime input. [VERIFIED: project scope; [ASSUMED] ASVS categories otherwise not applicable]

## Open Questions

1. **Template registry and physical pairing:** Are left/right pallet labels represented by separate assets or must one template classify both? Existing assets show four target files, but requirements do not define label-to-scan association. Planner should preserve generic label/template mapping and leave robot-side pairing explicit.
2. **Threshold values:** Existing single threshold is `0.55`; IoU, min-distance, ROI, and stale-age values are not specified. Start with named defaults, telemetry, and offline tests; tune on hardware in Phase 5.
3. **Test build command:** Repository test classes are executable Java under TeamCode, but exact Gradle task was not established during research. Planner should inspect `TeamCode/build.gradle` and root Gradle tasks before adding commands.

## Sources

### Primary (HIGH confidence)
- Repository `.planning/ROADMAP.md`, `.planning/REQUIREMENTS.md`, `.planning/PROJECT.md` — Phase 2 scope and constraints.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/TemplateMatchCamera.java` — current lifecycle, matching, filtering, and resource patterns.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/TemplateMatchCameraTest.java` — existing offline test style and temporal expectations.
- `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/TemplateMatchSingleTest.java`, `TemplateMatchTest.java` — current telemetry and target assets.

### Tertiary (LOW confidence)
- Exact public API names and NMS numeric defaults are recommendations, not locked decisions. [ASSUMED]

## Assumptions Log

| # | Claim | Risk if Wrong |
|---|---|---|
| A1 | Multi-mode can use configured templates from current `target*.png` assets. | Planner may need asset discovery/registry work. |
| A2 | One retained candidate should represent one physical box across labels. | Cross-template matching may need explicit association policy. |
| A3 | Plain Java assertions/executable tests remain accepted. | Gradle/JUnit integration may be required. |
| A4 | Exact result API can evolve while preserving existing callers where practical. | Existing OpModes may require compatibility adapters. |

## Metadata

**Confidence breakdown:**
- Standard stack: HIGH — directly verified in source and project constraints.
- Architecture: HIGH — current wrapper/pipeline and lifecycle directly inspected.
- Pitfalls: MEDIUM — current implementation and requirements verify most risks; numeric tuning remains open.

**Research date:** 2026-08-06
**Valid until:** 2026-09-05
