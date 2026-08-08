---
phase: quick-260808-cuy-camera-classification-only
plan: 01
type: execute
wave: 1
depends_on: []
files_modified:
  - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/LiftingSequenceStateMachine.java
  - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/Pi5GameplayCameraResult.java
  - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/LiftingSequenceOpMode.java
  - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/LeftCameraCenteringTestOpMode.java
  - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/LiftingSequenceConfig.java
  - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/LiftingSequenceConfigLoader.java
  - TeamCode/src/main/assets/robot-config.json
  - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/LiftingSequenceTest.java
  - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/LiftingSequenceConfigTest.java
autonomous: true
requirements:
  - QUICK-260808-CUY
must_haves:
  truths:
    - Camera data can only latch left/right block types; it cannot issue horizontal drive commands or gate horizontal alignment.
    - Block types latch only when both camera channels are simultaneously valid, fresh, and mapped to known factories.
    - After classification, pickup proceeds directly to the IR-guided approach while IR debounce, safe-stop behavior, and shelf pose capture remain intact.
    - Runtime config and telemetry contain no horizontal camera-centering calibration or status values.
    - Existing unrelated stepper, servo, config-value, and test-runner workspace changes remain untouched and excluded from this task.
  artifacts:
    - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/LiftingSequenceStateMachine.java
    - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/Pi5GameplayCameraResult.java
    - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/LiftingSequenceOpMode.java
    - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/LiftingSequenceConfig.java
    - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/LiftingSequenceConfigLoader.java
    - TeamCode/src/main/assets/robot-config.json
    - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/LiftingSequenceTest.java
    - TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/LiftingSequenceConfigTest.java
  key_links:
    - LiftingSequenceStateMachine.dualScanReady validates both channels before setBlockTypes and transition to APPROACH_IR_SLOW.
    - Pi5GameplayCameraResult exposes freshness, validity, and block type only through CameraResult.
    - LiftingSequenceConfigLoader required keys and robot-config.json stay synchronized after centering-only keys are removed.
---

<objective>
Remove camera-based horizontal centering and related calibration while retaining dual-channel block classification.

Purpose: Camera becomes a read-only classification input; IR and pose/navigation contracts continue controlling pickup movement and safety.
Output: Reduced state-machine camera contract, classification-only adapter/telemetry, cleaned config, deleted centering test OpMode, and updated runnable tests.

Workspace isolation: Before each edit, inspect current diff. Preserve existing user changes, especially servo values/assertions in `robot-config.json` and `LiftingSequenceConfigTest.java`, plus all stepper/servo/test-runner files. Do not revert them, rewrite whole dirty files, stage them as part of this task, or include unrelated hunks in any task commit.
</objective>

<execution_context>
@$HOME/.cursor/gsd-core/workflows/execute-plan.md
@$HOME/.cursor/gsd-core/templates/summary.md
</execution_context>

<context>
@.planning/STATE.md
@TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/LiftingSequenceStateMachine.java
@TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/Pi5GameplayCameraResult.java
@TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/LiftingSequenceOpMode.java
@TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/LeftCameraCenteringTestOpMode.java
@TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/LiftingSequenceConfig.java
@TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/LiftingSequenceConfigLoader.java
@TeamCode/src/main/assets/robot-config.json
@TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/LiftingSequenceTest.java
@TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/LiftingSequenceConfigTest.java
</context>

<tasks>

<task type="auto" tdd="true">
  <name>Task 1: Reduce gameplay camera contract to dual classification</name>
  <files>TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/LiftingSequenceStateMachine.java, TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/Pi5GameplayCameraResult.java, TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/LiftingSequenceTest.java</files>
  <behavior>
    - Either channel being invalid or stale keeps the machine in SCAN_RIGHT and leaves previously latched types unchanged.
    - Two valid, fresh channels with known block types latch both values atomically and advance directly from SCAN_RIGHT to APPROACH_IR_SLOW.
    - Unknown block type on either channel cannot authorize advancement.
    - Classification path performs no strafe command and requires no target stability or pixel displacement.
    - IR confirmation still records a valid ShelfPose before selecting LIFT1 or LIFT2.
  </behavior>
  <action>Write the classification tests first, then remove `SCAN_LEFT`, `CENTER_LEFT_SLOW`, and `CALIBRATE_SHELF_COORDINATE` from `State` and switch branches. Keep `SCAN_RIGHT` as the single dual-channel classification gate: call `setBlockTypes` only after `dualScanReady(now)` proves both readings valid, fresh, and known, then transition directly to `APPROACH_IR_SLOW`. Remove `stableLeftTarget` and `leftDxPx` from `CameraResult`, delete centering helpers/drive behavior, and reduce `Pi5GameplayCameraResult` to reading both frames plus classification validity/freshness/type access. Preserve `SAVE_SHELF_POSE`, `ShelfPose`, pose validation, IR approach/debounce, timeout handling, and all non-camera navigation states. Update the test camera fake and replace centering tests with the full dual-channel validity/freshness/unknown-type matrix and direct-transition/no-strafe assertions.</action>
  <verify>
    <automated>./gradlew.bat :TeamCode:compileDebugJavaWithJavac --offline &amp;&amp; java -ea -cp TeamCode/build/intermediates/javac/debug/compileDebugJavaWithJavac/classes org.firstinspires.ftc.teamcode.test.LiftingSequenceTest</automated>
  </verify>
  <done>Camera interface exposes classification evidence only; state machine cannot center from camera data; both channels must pass freshness, validity, and known-type gates before one atomic latch; pose capture remains in pickup path.</done>
</task>

<task type="auto" tdd="true">
  <name>Task 2: Remove centering-only configuration</name>
  <files>TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/LiftingSequenceConfig.java, TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/LiftingSequenceConfigLoader.java, TeamCode/src/main/assets/robot-config.json, TeamCode/src/main/java/org/firstinspires/ftc/teamcode/test/LiftingSequenceConfigTest.java</files>
  <behavior>
    - Config loads without horizontal scan/centering speed, deadband, or stable-frame keys.
    - Frame width/height and sensor staleness remain available for camera transport and freshness checks.
    - IR approach speed and all non-camera motion settings remain validated.
    - Config tests retain pre-existing servo symmetry assertions and values.
  </behavior>
  <action>Remove unused `scanSpeed`, `centerSpeed`, `centerDeadbandPx`, and `centerStableFrames` fields from `LiftingSequenceConfig`, constructor assignment, snapshots, loader required-key list, range validation, and `robot-config.json`. Keep `frameWidth`, `frameHeight`, `sensorStaleNs`, and `approachSpeed`: transport still needs dimensions, classification needs freshness, and IR approach still drives forward. Update `LiftingSequenceConfigTest` to stop asserting removed centering values and to range-test an existing relevant field such as `approachSpeed`. Make surgical replacements in the two already-dirty files: preserve current `placeLeft`/`placeRight` values and both fork symmetry checks exactly; do not stage or attribute those pre-existing hunks to this task.</action>
  <verify>
    <automated>./gradlew.bat :TeamCode:compileDebugJavaWithJavac --offline &amp;&amp; java -ea -cp TeamCode/build/intermediates/javac/debug/compileDebugJavaWithJavac/classes org.firstinspires.ftc.teamcode.test.LiftingSequenceConfigTest</automated>
  </verify>
  <done>Config schema contains only camera values still required for classification transport/freshness, loads successfully, rejects invalid retained motion values, and preserves unrelated servo changes.</done>
</task>

<task type="auto">
  <name>Task 3: Remove centering OpMode and telemetry</name>
  <files>TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/LiftingSequenceOpMode.java, TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmode/LeftCameraCenteringTestOpMode.java</files>
  <action>Delete `LeftCameraCenteringTestOpMode.java` completely. In `LiftingSequenceOpMode`, keep polling both camera channels and running the state machine, but replace stable-target/pixel-offset telemetry with classification-only evidence: per-channel valid/fresh status and latched block types. Adapt the camera update call only if Task 1 removes its now-unused timestamp parameter. Preserve camera transport cleanup, drive stop, actuator stop, IR updates, pose/navigation telemetry, and safe-stop reporting.</action>
  <verify>
    <automated>./gradlew.bat :TeamCode:compileDebugJavaWithJavac --offline &amp;&amp; java -ea -cp TeamCode/build/intermediates/javac/debug/compileDebugJavaWithJavac/classes org.firstinspires.ftc.teamcode.test.LiftingSequenceTest &amp;&amp; java -ea -cp TeamCode/build/intermediates/javac/debug/compileDebugJavaWithJavac/classes org.firstinspires.ftc.teamcode.test.LiftingSequenceConfigTest</automated>
  </verify>
  <done>No registered horizontal-centering test OpMode or centering telemetry remains; production gameplay reports classification health and still closes/stops every hardware resource.</done>
</task>

</tasks>

<threat_model>
## Trust Boundaries

| Boundary | Description |
|----------|-------------|
| Pi5 camera transport → gameplay state machine | External frame validity, age, channel, and block codes may authorize classification latch. |
| Classification state → robot motion state | Removing centering must not bypass IR-guided approach, IR debounce, pose validation, or safe-stop gates. |
| JSON asset → runtime control values | Removed calibration keys must not desynchronize loader, immutable config, tests, and deployed asset. |

## STRIDE Threat Register

| Threat ID | Category | Component | Severity | Disposition | Mitigation Plan |
|-----------|----------|-----------|----------|-------------|-----------------|
| T-260808-CUY-01 | Tampering | `dualScanReady` / `CameraResult` | high | mitigate | Require both named channels valid and fresh at the same tick, plus known factory mappings, before atomic type latch. |
| T-260808-CUY-02 | Elevation of Privilege | `SCAN_RIGHT` transition | high | mitigate | Transition only to `APPROACH_IR_SLOW`; preserve IR debounce, pose validation, and later lift gates rather than jumping to lift movement. |
| T-260808-CUY-03 | Denial of Service | stale or partial camera data | medium | accept | Hold in bounded `SCAN_RIGHT`; existing state timeout enters `SAFE_STOP` with outputs stopped. |
| T-260808-CUY-04 | Tampering | dirty workspace config/test files | high | mitigate | Preserve pre-existing servo hunks and stage/include only camera-removal hunks; verify scoped diff before any execution commit. |
</threat_model>

<source_coverage>
- GOAL — Camera classifies block type only: covered by Tasks 1 and 3.
- REQ — Both left/right fresh and valid before latching: covered by Task 1 tests and gate.
- REQ — Remove horizontal states, branches, stable/dx interface, centering OpMode, config, and telemetry: covered by Tasks 1-3.
- REQ — Remove centering calibration branch but preserve necessary pose capture: covered by Task 1 direct `SAVE_SHELF_POSE` to lift selection.
- REQ — Update tests and compile/test commands: covered by every task verification.
- RESEARCH — none supplied or required; change reuses existing Java/FTC patterns and adds no dependency.
- CONTEXT — unrelated dirty changes remain untouched and excluded: enforced in objective and Tasks 2-3.
</source_coverage>

<verification>
Run `./gradlew.bat :TeamCode:compileDebugJavaWithJavac --offline`, then both assertion-based test mains. Review final scoped diff to confirm centering-only deletion, retained dual-classification freshness/validity gates, retained `SAVE_SHELF_POSE`, and no changes to unrelated stepper/servo/test-runner work. Because `robot-config.json` and `LiftingSequenceConfigTest.java` were dirty before execution, confirm their existing servo hunks remain outside task staging/history.
</verification>

<success_criteria>
- Dual camera classification requires both channels valid, fresh, and known before types change.
- Classification advances to IR approach without any camera-derived strafe, target-stability, or pixel-offset logic.
- Shelf pose capture remains and the no-op camera/centering calibration state is gone.
- Centering test OpMode, config fields/keys, and telemetry are gone; frame dimensions, sensor freshness, and IR approach config remain.
- TeamCode compiles offline and both `LiftingSequenceTest` and `LiftingSequenceConfigTest` pass.
- Unrelated workspace changes are neither reverted nor included.
</success_criteria>

<output>
Create `.planning/quick/260808-cuy-b-y-gi-camera-ch-c-d-ng-x-c-nh-lo-i-bloc/260808-cuy-SUMMARY.md` when done.
</output>
