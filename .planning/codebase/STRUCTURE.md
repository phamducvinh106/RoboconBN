# Structure

- `build.gradle`: root Android Gradle plugins and repositories.
- `settings.gradle`: includes `FtcRobotController` and `TeamCode`.
- `build.common.gradle`: shared Android/FTC SDK build settings.
- `build.dependencies.gradle`: FTC SDK and AndroidX dependencies.
- `FtcRobotController/`: upstream FTC Robot Controller module and samples.
- `TeamCode/build.gradle`: applies shared build scripts and depends on Robot Controller.
- `TeamCode/src/main/assets/`: vision templates; currently `target.png`.
- `TeamCode/src/main/java/.../core/`: `MecanumDrive`, `PidController`, `TemplateMatchCamera`, `TwoWheelOdometry`.
- `TeamCode/src/main/java/.../opmode/`: registered `TemplateMatchSingleTest` and `TemplateMatchTest` OpModes.
- `TeamCode/src/main/java/.../test/`: offline checks for drive and camera filter behavior.
- `TeamCode/src/main/res/`: manifest, strings, webcam calibration XML.
- `.planning/codebase/`: generated codebase analysis documents.
