# Phase 7 Research: Camera/OpenCV Continuation

## Canonical Implementation

`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/ColorContourCamera.java` is current production camera implementation. It already owns EasyOpenCV creation, async open, streaming, stop, pipeline release, result publication, color-contour classification, temporal label stability, center error, confidence, and processing telemetry.

`TemplateMatchCamera.java` is absent. `MultiTargetCamera.java` is an older ORB/template implementation and should not become a second lifecycle path.

## Existing Integrations

- `LeftCameraCenteringTestOpMode` constructs `ColorContourCamera` with `webcam1` and `Mode.LEFT_CENTERING`, applies a 300 ms freshness limit, and strafes from `dxPx`.
- `RightCameraClassificationTestOpMode` constructs `ColorContourCamera` directly with `webcam2` and `Mode.RIGHT_CLASSIFICATION`.
- `RobotHardware` currently maps `webcam1` only.
- `MultiTargetCameraTest` demonstrates offline plain-Java temporal-filter testing without hardware.

## Constraints and Risks

1. Mode names currently differ from requirement language (`LEFT_CENTERING`/`RIGHT_CLASSIFICATION` versus `SINGLE_TARGET`/`MULTI_TARGET`). Plan must choose a compatibility strategy and make public semantics explicit.
2. Camera callbacks are asynchronous; callbacks after stop must not publish usable results or mutate closed resources.
3. A result can remain in the atomic reference after stream failure. Consumers need state/error/timestamp checks, not only `valid`.
4. OpenCV `Mat` and contour ownership must remain bounded and released on every frame path.
5. `webcam2` requires an explicit hardware-map contract; no silent fallback to `webcam1`.

## Recommended Approach

Keep `ColorContourCamera` as the single lifecycle owner. Add explicit mode semantics and camera identity/configuration at its public boundary, centralize freshness/lifecycle validity, map both webcams in `RobotHardware`, update test OpModes, and extend plain-Java offline checks for mode selection, center math, classification ranking, stale results, and lifecycle transitions.

## Package Legitimacy Audit

No package installation planned. Existing FTC, EasyOpenCV, and OpenCV dependencies are reused.
