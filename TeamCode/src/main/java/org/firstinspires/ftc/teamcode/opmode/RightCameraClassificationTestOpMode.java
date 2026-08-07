package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.core.ColorContourCamera;

@TeleOp(name = "Right Camera Sticker Classification Test", group = "Test")
public final class RightCameraClassificationTestOpMode extends LinearOpMode {
    @Override public void runOpMode() throws InterruptedException {
        ColorContourCamera camera = new ColorContourCamera(hardwareMap, "webcam2", true, ColorContourCamera.Mode.MULTI_TARGET);
        camera.startAsync();
        telemetry.addLine("webcam2 only: sticker shape/texture classification"); telemetry.update();
        waitForStart();
        try {
            while (opModeIsActive()) {
                ColorContourCamera.Result r = camera.getLatestResult();
                telemetry.addData("camera", "%s (%d)", camera.getCameraState(), camera.getCameraErrorCode());
                telemetry.addData("classification", r.valid ? r.label : "NONE");
                telemetry.addData("stable", "%d/5", r.stableFrames);
                telemetry.addData("fast ready", r.fastCentering);
                telemetry.addData("center", "%.0f, %.0f", r.centerX, r.centerY);
                telemetry.addData("confidence", "%.2f", r.confidence);
                telemetry.addData("contours / ms", "%d / %.1f", r.contourCount, camera.getProcessingMs());
                telemetry.update(); sleep(50);
            }
        } finally { camera.stop(); }
    }
}
