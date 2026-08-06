# Stack

- Android FTC Robot Controller app, Gradle multi-module project.
- Modules: `FtcRobotController` SDK app/library support and `TeamCode` robot code.
- Android Gradle Plugin `8.13.2`; compile SDK 30; min SDK 24; target SDK 28.
- Java 8 source/target compatibility; NDK `21.3.6528147`; debug/release ABIs `armeabi-v7a`, `arm64-v8a`.
- FTC SDK dependencies version `11.2.1`: RobotCore, Hardware, Vision, RobotServer, Inspection, Blocks, OnBotJava, FtcCommon.
- AndroidX AppCompat `1.2.0`; AndroidX enabled, Jetifier disabled.
- Vision stack: EasyOpenCV/OpenCV classes plus FTC `Vision`; webcam input via `WebcamName`.
- No external test framework. Offline Java-style executable tests live in `TeamCode/src/main/java/.../test`.
