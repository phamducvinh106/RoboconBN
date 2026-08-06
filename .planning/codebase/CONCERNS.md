# Concerns

- `TemplateMatchCamera` is single-template by construction; adding `MULTI_TARGET` without a result/API decision will create ambiguous control behavior.
- Template matching assumes fixed viewpoint and limited target rotation/scale; confidence can degrade under lighting, perspective, occlusion, or size changes.
- Multi-target matching multiplies `matchTemplate` work and memory use; 640x480 stream and current downscaling may not meet loop latency with many templates.
- One global temporal filter and `activeLabel` cannot safely track several objects. Per-target filter state or an explicit selected-target policy is required.
- `setTarget()` releases/reloads Mats while processing is asynchronous; synchronization is narrow. Mode/template changes need race and lifecycle testing.
- Camera errors are represented by strings and numeric codes; callers must handle stale/invalid results and `NaN` offsets explicitly.
- Offline tests copy production constants/logic, so drift can produce false confidence.
- PID, odometry signs, offsets, and wheel constants are hand-tuned; incorrect hardware names/orientation silently prevent valid runtime behavior or produce bad pose.
- `compileSdkVersion 30` with AGP `8.13.2` and target SDK 28 is an aging compatibility combination; validate with installed SDK/JDK before upgrades.
- No automated CI, integration harness, or OpenCV frame fixture suite is present.
