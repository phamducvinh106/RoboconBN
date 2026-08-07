# Phase 7 Validation Contract

## Plan Coverage

- `07-01-PLAN.md`: shared `ColorContourCamera` ORB/template lifecycle, explicit `webcam1`/`webcam2`, centering authority, deterministic candidate policy.
- `07-02-PLAN.md`: offline plain-Java assertions and this executable validation contract.
- `07-03-PLAN.md`: transport boundary remains deferred; no UART/20-bit/I2C production implementation.

## Requirement and Decision Coverage

- `VIS-01`: one shared explicit two-mode API in `ColorContourCamera`.
- `VIS-02`/`VIS-03`: signed center metrics, finite-value and freshness movement gates.
- `VIS-04`/`VIS-05`: both webcams classify; ORB/template thresholds, deterministic ranking, bounded overlap/NMS/min-distance suppression, stable candidate policy.
- `VIS-06`/`VIS-07`: lifecycle generation guards, idempotent start/stop, resource release, invalid/error/stale fail-closed results.
- `TEST-01`: `CameraContinuationTest` covers modes, webcam identity, thresholds, ranking bound, lifecycle states, source constraints, and fail-closed boundaries.
- `D-01`: existing lifecycle owner retained; no separate camera class.
- `D-02`: shared lifecycle/result contract.
- `D-03`: both webcams classify; only `webcam1` permits `SINGLE_TARGET` centering.
- `D-04`: plain Java `main`/`AssertionError`; no JUnit or dependency.
- `D-05`: generation, invalidation, idempotent lifecycle, and stale/error rejection.

## Research Constraint Audit

- ORB/template matching only; no HSV/YCrCb thresholding or contour extraction.
- No new dependency; existing pinned FTC/EasyOpenCV/OpenCV stack only.
- No I2C, UART wiring, packed 20-bit decoder, or production `DigitalUartRx` path.
- `DigitalUartRx` remains bench-only. Protocol framing, byte order, checksum, timeout, timestamp units, and 20-bit payload remain gated for future decision.
- Deferred AprilTags, neural detection, adaptive compensation, camera multiplexing, and calibration remain out of scope.

## Automated Checks

- Windows Gradle compile: `./gradlew.bat :TeamCode:compileDebugJavaWithJavac`.
- Offline assertions: `java -ea -cp TeamCode/build/intermediates/javac/debug/compileDebugJavaWithJavac/classes org.firstinspires.ftc.teamcode.test.CameraContinuationTest`.
- Expected output: `BUILD SUCCESSFUL`; `CameraContinuationTest passed 20 checks`.
- Source audit: one `ColorContourCamera` lifecycle owner, explicit names, ORB/descriptor matcher, no HSV/YCrCb/contours, no I2C/UART camera path.

## Manual FTC Acceptance

- Physically confirm configured `webcam1` and `webcam2` identities; missing `webcam2` must fail normal `HardwareMap` lookup and never alias `webcam1`.
- Confirm both cameras classify configured templates; only `webcam1` `SINGLE_TARGET` may authorize centering.
- Confirm stale, invalid, closed, and error results never authorize movement.
- Confirm start twice opens one stream; stop twice does not throw; camera stop releases pipeline resources.

## Deferred Transport Boundary: 07-03

Phase 7 has one transport path: no production `DigitalUartRx`, `CameraFrameContract`, `CameraAdapterManager`, packed 20-bit decoder, I2C wiring, or UART wiring. Physical protocol evidence must precede any future transport implementation.


## Automated Checks

- Compile TeamCode with existing Windows Gradle wrapper: `.\\gradlew.bat :TeamCode:compileDebugJavaWithJavac`.
- Run offline camera test main class through repository's existing Java test convention.
- Verify source contains one camera lifecycle owner and explicit `webcam1`/`webcam2` hardware mappings.
- Verify only ORB/template matching is referenced for detection; verify no HSV/YCrCb thresholding, contour extraction/classification, candidate segmentation, or related scope is added, and no new dependency is added.
- Verify plan mapping includes `07-01`, `07-02`, and `07-03`.
- Verify Phase 7 adds no production `DigitalUartRx`, `CameraFrameContract`, `CameraAdapterManager`, packed 20-bit decoder, I2C wiring, or UART wiring.
- Verify `PlaceholderCameraTransport` remains invalid/fail-closed and `DigitalUartRx` remains bench-only.

## Behavior Checks

- Both `webcam1` and `webcam2` use configured ORB/template assets for block-type classification, applying descriptor/match thresholds and already-scoped geometric constraints, then returning distinct stable labels after deterministic ranking and overlap/NMS/duplicate suppression.
- Only `webcam1` may run `SINGLE_TARGET` for left-pallet centering and expose center/error/confidence/validity/timestamp; `webcam2` classification never authorizes centering.
- Start called twice does not open duplicate streams; stop called twice does not throw.
- Open/start error yields error state; closed/error/stale results cannot command movement.
- Pipeline resources release on stop and invalid frames do not publish usable detections.
- Existing left centering and right classification OpModes compile against final API.

## Deferred Transport Boundary: 07-03

Phase 7 has one transport path: no production DigitalUartRx, CameraFrameContract, CameraAdapterManager, packed 20-bit decoder, I2C wiring, or UART wiring. DigitalUartRx remains bench-only. PlaceholderCameraTransport must remain invalid and fail-closed. Framing, endian order, checksum, idle level, timestamp source/units, timeout, channel identity, partial-read handling, and packed 20-bit decoding require explicit physical protocol evidence and a future phase decision.

## Manual FTC Acceptance

- Confirm configured `webcam1` and `webcam2` identities physically; missing webcam2 must not alias webcam1.
- Confirm stale, invalid, closed, and error camera results never authorize movement.
