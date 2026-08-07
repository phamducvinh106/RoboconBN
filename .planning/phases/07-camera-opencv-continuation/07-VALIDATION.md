# Phase 7 Validation Contract

## Automated Checks

- Compile TeamCode with existing Gradle wrapper/task.
- Run offline camera test main class through the repository's existing Java test convention.
- Verify source contains one camera lifecycle owner and explicit `webcam1`/`webcam2` hardware mappings.
- Verify no `TemplateMatchCamera.java` is created and no new dependency is added.

## Behavior Checks

- `SINGLE_TARGET` selects left-centering policy on webcam1 and exposes center/error/confidence/validity/timestamp.
- `MULTI_TARGET` selects classification policy on webcam2 and returns distinct qualified labels after overlap suppression.
- Start called twice does not open duplicate streams; stop called twice does not throw.
- Open/start error yields error state; closed/error/stale results cannot command movement.
- Pipeline resources release on stop and invalid frames do not publish usable detections.
- Existing left centering and right classification OpModes compile against the final API.
