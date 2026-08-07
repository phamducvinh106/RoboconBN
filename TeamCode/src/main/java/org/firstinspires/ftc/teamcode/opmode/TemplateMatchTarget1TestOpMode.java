package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.core.TemplateMatchCamera;

@TeleOp(name = "Template Match Target 1 Test", group = "Vision")
public final class TemplateMatchTarget1TestOpMode extends OpMode {
    private static final String WEBCAM = "webcam1";
    private static final String TARGET_ASSET = "target1.png";
    private TemplateMatchCamera camera;

    @Override
    public void init() {
        camera = new TemplateMatchCamera(hardwareMap, WEBCAM, true, TARGET_ASSET);
        camera.startAsync();
        telemetry.addData("camera", WEBCAM);
        telemetry.addData("target", TARGET_ASSET);
        telemetry.addData("template loaded", camera.templateLoaded());
        telemetry.addLine("Opening camera during INIT...");
        telemetry.update();
    }

    @Override
    public void init_loop() {
        telemetry.addData("state", camera.getCameraState());
        telemetry.addData("error", camera.errorCode());
        telemetry.addData("template loaded", camera.templateLoaded());
        telemetry.addLine("Camera must stream before START");
        telemetry.update();
    }

    @Override
    public void loop() {
        TemplateMatchCamera.Result result = camera.latest();
        telemetry.addData("state", camera.getCameraState());
        telemetry.addData("valid", result.valid);
        telemetry.addData("dx / dy", "%.1f / %.1f px", result.dxPx, result.dyPx);
        telemetry.addData("center", "%.1f / %.1f", result.centerX, result.centerY);
        telemetry.addData("confidence", "%.3f", result.confidence);
        telemetry.addData("processing", "%.1f ms", result.processingMs);
        telemetry.addData("age", "%d ms", System.currentTimeMillis() - result.timestampMs);
        telemetry.addData("error", camera.errorCode());
        telemetry.update();
    }

    @Override
    public void stop() {
        if (camera != null) camera.stop();
    }
}
