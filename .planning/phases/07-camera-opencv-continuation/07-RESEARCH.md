# Phase 7 Research: Realtime Vision With Minimal Samples

## Goal

Select OpenCV-only algorithms for O2 pallet classification and high-precision realtime centering on REV Control Hub. Minimize sample images, CPU cost, latency, and tuning surface.

## Hardware/scene assumptions

- `webcam1` views one pallet/fork target during centering.
- `webcam2` is reserved for multi-target classification when available.
- Object appearance changes mainly by translation, moderate scale change, lighting, and partial occlusion; arbitrary rotation is not expected during pickup.
- Target must be centered relative to camera optical center, not merely detected as a large contour.
- Existing FTC, EasyOpenCV, and OpenCV stack remains mandatory. No neural model or new dependency.

## Candidate comparison

### 1. Color contour + bounding rectangle

- Samples: zero, if color/shape thresholds are known.
- Cost: lowest.
- Strength: fast and easy on-device.
- Weakness: cannot reliably distinguish four visually similar cargo labels; bounding rectangle center is biased by glare, holes, occlusion, and merged contours.
- Use: coarse candidate generation and presence check, not final high-precision center alone.

### 2. Single-scale template matching

- Samples: one clean template per visual class; one target template is enough for centering.
- Cost: low to medium; reduce ROI and image resolution.
- Strength: direct subpixel-friendly response peak, no training, works with few samples, good for fixed camera/object geometry.
- Weakness: sensitive to scale, rotation, perspective, blur, and lighting; `minMaxLoc` returns only one candidate.
- Use: primary centering detector after coarse ROI/candidate gating.

### 3. Multi-scale/rotated template bank

- Samples: still one source image per class, but many transformed templates.
- Cost: high and latency grows with template count.
- Strength: more tolerant to scale/rotation.
- Weakness: unnecessary complexity for fixed fork camera; more false matches and harder confidence comparison.
- Use: deferred only if field tests prove fixed-scale matching insufficient.

### 4. ORB/features

- Samples: one or more reference images.
- Cost: medium/high and less deterministic on small repetitive cargo faces.
- Strength: some scale/rotation tolerance.
- Weakness: sparse texture, lighting, and low-resolution pallet images produce unstable keypoints; homography is overkill for centering.
- Use: reject for primary realtime pickup alignment.

### 5. CamShift/mean-shift tracking

- Samples: one initial detection or color histogram.
- Cost: low after initialization.
- Strength: excellent frame-to-frame latency when target remains visible.
- Weakness: drifts under occlusion, similar colors, and target disappearance; needs a reliable detector for reacquisition.
- Use: optional tracker between periodic template detections, never sole safety source.

### 6. Neural detector/classifier

- Samples: many labeled images, model conversion, runtime memory/CPU.
- Strength: robust to appearance variation when trained well.
- Weakness: violates current minimal-sample/no-new-dependency constraint and increases integration risk.
- Use: out of scope.

## Decision

Use a hybrid two-stage pipeline:

1. **Coarse candidate gate**
   - Convert frame to grayscale and/or HSV.
   - Restrict search to configured ROI where pallet is expected.
   - Use contour/edge/brightness gate to reject empty background.
   - For classification, run one template per cargo type only inside candidate ROIs.

2. **Template matching**
   - `Imgproc.matchTemplate(..., Imgproc.TM_CCOEFF_NORMED)`.
   - Use one clean frontal template per class. No image dataset or training.
   - For centering, use one left-pallet template and track only the best candidate.
   - For classification, score all class templates at each candidate and select label only when margin over second-best score is sufficient.

3. **Local refinement**
   - Refine best match in a small full-resolution ROI around coarse result.
   - Estimate center from response peak, then apply a short temporal median/EMA filter.
   - Keep raw confidence separate from smoothed center.

4. **Realtime tracking**
   - After a valid match, search a small window around previous center.
   - Periodically reacquire globally or when confidence drops.
   - Reject stale results by timestamp; no movement command from invalid/error/closed result.

5. **Motion stop rule**
   - Use camera center error `dxPx`, confidence, freshness, and stability.
   - Stop strafing only when `abs(dxPx) <= centerTolerancePx` for N consecutive fresh frames.
   - Never stop on one noisy frame.

## Why this choice

- One reference image per class is minimum practical sample count under fixed-template constraints.
- Template matching gives direct location, unlike color-only contour center.
- ROI and local search reduce CPU and latency.
- Temporal stability prevents motor chatter without adding a heavyweight tracker.
- Periodic reacquisition prevents permanent drift.
- No neural dependency, training set, or network access.

## Required implementation contract

`ColorContourCamera` should expose explicit modes:

- `SINGLE_TARGET`: `webcam1`, one target, center precision priority.
- `MULTI_TARGET`: explicit camera name, all configured class templates, classification priority.

Result must include:

- `timestampMs`
- `valid`
- `label`
- `centerX`, `centerY`
- `dxPx`, `dyPx`
- `rawConfidence`
- `smoothedConfidence` only if needed by consumers
- `stableFrames`
- `fresh(maxAgeMs)` helper or equivalent consumer check
- `cameraState`

Mode must not silently select a different webcam.

## Tuning order

1. Fix camera mount, focus, resolution, and ROI.
2. Capture one clean frontal template per class at expected distance.
3. Tune grayscale/normalization and confidence threshold.
4. Tune local search radius and reacquisition interval.
5. Tune center tolerance and consecutive stable frames.
6. Validate under field lighting, blur, glare, pallet height, and partial occlusion.
7. Add scale variants only if measured failure rate requires them.

## Metrics

Record per frame and per run:

- processing time and FPS
- raw confidence
- confidence margin between top two classes
- center error before/after smoothing
- center error standard deviation while robot is stationary
- frames to stable detection
- stale/invalid frame count
- false lock count
- reacquisition count

Suggested acceptance targets require field measurement; do not hard-code them before testing. Initial engineering targets: processing under 30 ms/frame, stable center error under 3% of frame width, and zero movement command from stale/error results.

## Risks

- `TM_CCOEFF_NORMED` degrades with scale/rotation/perspective changes; fix camera geometry before adding templates.
- Color contour implementation currently allocates submat/temporary objects inside each contour path; allocation must be bounded before final realtime tuning.
- Existing mode names (`LEFT_CENTERING`, `RIGHT_CLASSIFICATION`) differ from planned names (`SINGLE_TARGET`, `MULTI_TARGET`). Use explicit compatibility mapping or update all consumers in one change.
- Existing `ColorContourCamera` uses HSV class ranges for generic blocks; O2 image labels need actual class templates and validated assets.
- A single camera cannot simultaneously classify both blocks if FOV covers one block; classification and centering must be sequential as described in project context.

## Sources

- OpenCV Template Matching tutorial: https://docs.opencv.org/4.13.0/d4/dc6/tutorial_py_template_matching.html
- OpenCV `matchTemplate` API: https://docs.opencv.org/4.13.0/df/dfb/group__imgproc__object.html

## Scope boundary

Do not add neural detection, AprilTags, ORB, a new dependency, or a large template bank in Phase 7. Implement the hybrid pipeline first, then measure before expanding scope.
