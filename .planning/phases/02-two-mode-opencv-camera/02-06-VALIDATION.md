# Pi5 I2C Robot Controller Registration Validation

## Verdict

**BLOCKED — implementation cannot proceed.** Repository pins FTC SDK `11.2.1`, but repository-local SDK source/javadocs and binary class artifacts do not expose an evidence-backed public custom-device registration and configuration-persistence path. No production Java, Gradle, manifest, or TeamCode source was changed.

The only exact registration-related symbols found are release-note references in `README.md`:

- `com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties`
- `com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType`
- `I2cSensorType` renamed to `I2cDeviceType`
- `com.qualcomm.robotcore.hardware.HardwareMap` is used by existing TeamCode source, but this proves runtime lookup only, not registration.
- `I2cDeviceSynchImpl`, `I2cDeviceSync`, and `I2cDevice` appear in SDK release notes, but no pinned class/member signature was available locally for implementation authorization.

These release-note names are **not sufficient evidence** of package visibility, annotation members, registration discovery, configuration serialization, or Robot Controller/Driver Station transport.

## Evidence inventory

| Required proof | Exact symbol/path found | Classification | Result |
|---|---|---|---|
| Custom device interface/base type | No pinned source/javadoc/class signature found | Missing | Blocker |
| Device-type metadata and selectable label | `com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties`; `com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType` in `README.md` release notes only | Package/name reference only; member contract unavailable | Blocker |
| Robot Controller registration/discovery | No exact registry, scanner, or module registration symbol found | Missing | Blocker |
| I2C address/settings editor | No exact configuration-control symbol found | Missing | Blocker |
| Saved configuration format/serializer/parser | No exact XML/JSON schema or serializer/parser symbol found | Missing | Blocker |
| Robot Controller endpoint serving Driver Station config | No exact endpoint/service symbol found | Missing | Blocker |
| Saved config to runtime `HardwareMap` reconstruction | Existing `HardwareMap` imports/lookup patterns found in TeamCode; no custom registration reconstruction path | Public runtime consumer only | Blocker |
| I2C transport symbols | Release-note names `I2cDeviceSynchImpl`, `I2cDeviceSync`, `I2cDevice`; no pinned signatures | Unverified | Blocker |

Pinned dependency declarations are exact and present in `build.dependencies.gradle`: `Inspection`, `Blocks`, `RobotCore`, `RobotServer`, `OnBotJava`, `Hardware`, `FtcCommon`, and `Vision`, all version `11.2.1`. Local Gradle cache inspection found `.module` and `.pom` metadata but no locally discoverable source/javadoc JAR evidence. Tracked repository files contain no FTC SDK source tree or SDK binary artifacts.

## Ownership and unsupported workaround

Driver Station-visible hardware configuration is Robot Controller-owned. Later work must prove this chain:

1. Robot Controller module exposes custom device metadata.
2. Robot Controller configuration UI discovers/selects the type and edits name/address/settings.
3. Robot Controller persists configuration and serves it to Driver Station.
4. Robot Controller reconstructs the configured device.
5. TeamCode resolves exact configured name `pi5Camera` through `HardwareMap`.

TeamCode-only annotation or registration is explicitly rejected. It cannot be treated as sufficient for Driver Station selectable-device behavior without SDK evidence that Robot Controller scans and persists TeamCode metadata. No such evidence exists here.

## Locked future scope after blocker closure

Implementation, only after exact SDK evidence is supplied, belongs in the Robot Controller/FtcRobotController registration/configuration path plus a TeamCode runtime consumer. It must cover:

- Device-type metadata and registration in Robot Controller.
- Selectable configuration entry, exact name `pi5Camera`, and 7-bit I2C address `0x42`.
- Save/reload round-trip and Driver Station visibility from Robot Controller-served configuration.
- TeamCode `HardwareMap` lookup of the configured device by `pi5Camera`.
- Isolated address, register read/write only where verified, heartbeat, validity/freshness, and channel smoke checks.
- Two explicit logical channels `webcam1` and `webcam2`; no fallback or identity merge.
- Packed 20-bit logical placeholder: `X_MASK=0x000FF` shift `0`, `Y_MASK=0x0FF00` shift `8`, `LEFT_TYPE_MASK=0x30000` shift `16`, `RIGHT_TYPE_MASK=0xC0000` shift `18`; configurable code `0..3` to block types `01..04`.
- Invalid, incomplete, partial, stale, or reserved data fails closed and cannot authorize movement.

Deferred: physical frame/register layout, endian order, checksum, sentinel policy, screen coordinates, axis origin, and atomic I2C read semantics. No OpenCV, template matching, or physical frame parser belongs here.

## Validation gates

1. Obtain pinned SDK `11.2.1` source/javadocs or inspect exact resolved class files.
2. Record exact public/internal package and member signatures for metadata, registration, configuration controls, persistence, Robot Controller serving, and runtime reconstruction.
3. Compile the smallest Robot Controller registration proof against `11.2.1`; reject invented or internal-only symbols.
4. Build/install matching Robot Controller and Driver Station versions.
5. Verify custom type selectable, `pi5Camera` and `0x42` save/reload, Driver Station visibility, and `HardwareMap` reconstruction.
6. Run hardware-first communication smoke test before JSON/state integration: both channels, heartbeat, validity/freshness, and fail-closed invalid/partial/stale cases.
7. Only then authorize a separate source implementation plan.

## Compatibility risks

- SDK internal API drift can break registration across versions.
- Robot Controller versus TeamCode module placement can hide metadata from configuration discovery.
- Configuration schema migration can discard custom fields or break round-trip persistence.
- Device-type name collisions can make selectable identity ambiguous.
- UI/runtime address representation can confuse decimal and 7-bit `0x42` semantics.
- Robot Controller/Driver Station version mismatch can hide or reject the type.
- HardwareMap may require a different verified SDK abstraction than currently assumed.
- Physical transport details remain unresolved and must not be smuggled into registration work.

## References

- `build.dependencies.gradle`: pinned FTC SDK dependency versions.
- `README.md` lines 677–679, 1210, and 1617: release-note symbol/name references only.
- `.planning/phases/02-two-mode-opencv-camera/02-CONTEXT.md`: D-03, D-04, D-06, D-07, D-08.
- `.planning/phases/02-two-mode-opencv-camera/02-06-PLAN.md`: evidence gate and no-source requirement.
- `.planning/phases/02-two-mode-opencv-camera/02-VALIDATION.md`: hardware-first acceptance contract.
