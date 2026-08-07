package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.core.OrbTemplateCamera;

@TeleOp(name = "ORB Target 1 Test", group = "Vision")
public final class OrbTarget1TestOpMode extends OpMode {
    private static final String WEBCAM = "webcam1";
    private static final String TARGET_ASSET = "target1.png";
    private OrbTemplateCamera camera;

    @Override
    public void init() {
        try {
            camera = new OrbTemplateCamera(hardwareMap, WEBCAM, true,
                    OrbTemplateCamera.Mode.SINGLE_TARGET, "target1",
                    OrbTemplateCamera.loadAsset(TARGET_ASSET));
            camera.startAsync();
            telemetry.addLine("Opening ORB camera during INIT...");
        } catch (IllegalArgumentException error) {
            telemetry.addData("init error", error.getMessage());
        }
        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addData("camera", WEBCAM);
        telemetry.addData("target", TARGET_ASSET);
        if (camera != null) {
            telemetry.addData("state", camera.getCameraState());
            telemetry.addData("error", camera.getCameraErrorCode());
            telemetry.addData("template loaded", camera.templateLoaded());
        }
        telemetry.update();
    }

    @Override
    public void loop() {
        if (camera == null) return;
        OrbTemplateCamera.Result result = camera.getLatestResult();
        telemetry.addData("state", camera.getCameraState());
        telemetry.addData("detection", result.detectionState);
        telemetry.addData("valid", result.valid);
        telemetry.addData("authorize", result.authorizesMovement);
        telemetry.addData("fresh", OrbTemplateCamera.fresh(result, System.currentTimeMillis()));
        telemetry.addData("dx / dy", "%.1f / %.1f px", result.dxPx, result.dyPx);
        telemetry.addData("center", "%.1f / %.1f", result.centerX, result.centerY);
        telemetry.addData("inlier ratio", "%.3f", result.confidence);
        telemetry.addData("processing", "%.1f ms", result.processingMs);
        telemetry.addData("fps", "%.1f", result.fps);
        telemetry.addData("age", "%d ms", System.currentTimeMillis() - result.timestampMs);
        telemetry.addData("error", camera.getCameraErrorCode());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (camera != null) camera.stop();
    }
}
