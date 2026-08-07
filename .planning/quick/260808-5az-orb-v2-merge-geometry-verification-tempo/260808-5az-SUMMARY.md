---
status: complete
quick_id: 260808-5az
---

# Summary: ORB v2 geometry verification

## Done

- **`OrbTemplateCamera` v2:** ORB 400/6 levels, ratio 0.70, Hamming ≤60, mutual match, RANSAC 3px
- **Geometry gates:** ≥12 good, ≥10 inliers, ≥0.55 ratio, reprojection ≤3px, axis coverage ≥0.35, 3/4 quadrants, quad sanity
- **`DetectionState`:** SEARCHING → LOCKED (3 frames), COASTING on miss, LOST after 3 misses; `authorizesMovement` only LOCKED + ≤120ms
- **Smoothing:** median-3 + EMA 0.3 + velocity gate 180 px/s after geometry pass
- **Removed** `SingleTargetCamera.java`; `OrbTarget1TestOpMode` uses `OrbTemplateCamera.loadAsset`
- **`CameraContinuationTest`** updated for new constants

## Commits

- Code: OrbTemplateCamera, OpMode, test, delete SingleTargetCamera

## Deferred

- Dynamic tracking ROI
- Shared ORB per webcam in orchestrator
- Webcam exposure/gain/focus lock
