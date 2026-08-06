package org.firstinspires.ftc.teamcode.opmode;

import org.firstinspires.ftc.teamcode.core.MultiTargetCamera;
import org.firstinspires.ftc.teamcode.core.TwoWheelOdometry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "Camera + Odometry Modular")
public final class CameraOdometryMain extends LinearOpMode {

    private TwoWheelOdometry odometry;
    private MultiTargetCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        odometry = new TwoWheelOdometry(
                hardwareMap,
                "ypod",
                "xpod",
                "imu",
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        camera = new MultiTargetCamera(
                hardwareMap,
                "Webcam 1",
                true,
                // Put these files in TeamCode/src/main/assets/.
                "target.png"
        );
        camera.startAsync();

        telemetry.addLine("Initialized. A = reset pose; Y = reset pose + yaw");
        telemetry.addData("Templates loaded", camera.getLoadedTemplateCount());
        for (String error : camera.getLoadErrors()) {
            telemetry.addData("Template error", error);
        }
        telemetry.update();

        waitForStart();

        boolean previousA = false;
        boolean previousY = false;

        try {
            while (opModeIsActive()) {
                boolean currentA = gamepad1.a;
                boolean currentY = gamepad1.y;

                if (currentA && !previousA) {
                    odometry.resetPose();
                }
                if (currentY && !previousY) {
                    odometry.resetPoseAndHeading();
                }

                previousA = currentA;
                previousY = currentY;

                odometry.update();
                sendCameraTelemetry();
                sendOdometryTelemetry();
                telemetry.update();

                sleep(20);
            }
        } finally {
            camera.stop();
        }
    }

    private void sendCameraTelemetry() {
        MultiTargetCamera.CameraResult result = camera.getLatestResult();
        long ageMs = camera.getLastSuccessfulDetectionMs() == 0
                ? Long.MAX_VALUE
                : System.currentTimeMillis()
                - camera.getLastSuccessfulDetectionMs();
        boolean fresh = result.isValid() && ageMs < 500;

        telemetry.addLine("--- CAMERA ---");
        telemetry.addData("State", camera.getCameraState());
        if ("ERROR".equals(camera.getCameraState())) {
            telemetry.addData("Camera error", camera.getCameraErrorCode());
        }
        telemetry.addData("Templates", camera.getLoadedTemplateCount());
        telemetry.addData("Fresh target", fresh);
        telemetry.addData("Detected objects", result.detections.size());
        telemetry.addData("Camera FPS", "%.1f", camera.getFrameFps());
        telemetry.addData("Processing FPS", "%.1f", camera.getProcessingFps());
        telemetry.addData("Processing ms", "%.1f", camera.getProcessingMs());

        if (result.selected != null) {
            telemetry.addData("Selected", result.selected.label);
            telemetry.addData("dx px", "%.1f", result.dxPx);
            telemetry.addData("dy px (down +)", "%.1f", result.dyPx);
            telemetry.addData("Distance center px", "%.1f",
                    result.selected.distanceToCenter);
            telemetry.addData("Confidence", "%.2f",
                    result.selected.confidence);
            telemetry.addData("Matches / inliers", "%d / %d",
                    result.selected.goodMatches,
                    result.selected.inliers);
        }

        // --- Filter debug telemetry ---
        telemetry.addLine("--- FILTER DEBUG ---");
        telemetry.addData("rawDx", "%.1f", camera.getRawDx());
        telemetry.addData("smoothedDx", "%.1f", camera.getSmoothedDx());
        telemetry.addData("outlierStreak", camera.getOutlierStreak());
        telemetry.addData("missStreak", camera.getMissStreak());
        telemetry.addData("filterInit", camera.isFilterInitialized());
        telemetry.addData("activeLabel", camera.getActiveLabel());

        List<MultiTargetCamera.Detection> detections = result.detections;
        for (int i = 0; i < detections.size(); i++) {
            MultiTargetCamera.Detection detection = detections.get(i);
            telemetry.addData(
                    "Object " + i,
                    "%s center=(%.0f,%.0f) conf=%.2f",
                    detection.label,
                    detection.centerX,
                    detection.centerY,
                    detection.confidence
            );
        }
    }

    private void sendOdometryTelemetry() {
        telemetry.addLine("--- ODOMETRY ---");
        telemetry.addData("A", "reset pose/calibration");
        telemetry.addData("Y", "reset pose + IMU yaw");
        telemetry.addData("X right cm", "%.2f", odometry.getX());
        telemetry.addData("Y forward cm", "%.2f", odometry.getY());
        telemetry.addData("Heading deg", "%.2f",
                odometry.getHeadingDeg());
        telemetry.addData("Forward local cm", "%.4f",
                odometry.getLastForwardLocalCm());
        telemetry.addData("Left local cm", "%.4f",
                odometry.getLastLeftLocalCm());

        telemetry.addLine("--- ODOMETRY CALIBRATION ---");
        telemetry.addData("Accum heading rad", "%.5f",
                odometry.getAccumHeadingRad());
        telemetry.addData("Suggested PARALLEL_Y", "%.4f",
                odometry.getSuggestedParallelYOffsetCm());
        telemetry.addData("Suggested PERP_X", "%.4f",
                odometry.getSuggestedPerpendicularXOffsetCm());
    }
}
