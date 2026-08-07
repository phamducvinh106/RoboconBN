package org.firstinspires.ftc.teamcode.opmode;

import org.firstinspires.ftc.teamcode.core.ColorContourCamera;
import org.firstinspires.ftc.teamcode.core.TwoWheelOdometry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Camera + Odometry Modular")
public final class CameraOdometryMain extends LinearOpMode {

    private TwoWheelOdometry odometry;
    private ColorContourCamera camera;

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

        camera = new ColorContourCamera(
                hardwareMap,
                "webcam1",
                true,
                ColorContourCamera.Mode.SINGLE_TARGET
        );
        camera.startAsync();

        telemetry.addLine("Initialized. A = reset pose; Y = reset pose + yaw");
        telemetry.addData("Contour detector", "HSV/YCrCb + contours");
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
        ColorContourCamera.Result result = camera.getLatestResult();
        boolean fresh = result.valid
                && System.currentTimeMillis() - result.timestampMs < 500;

        telemetry.addLine("--- CAMERA ---");
        telemetry.addData("State", camera.getCameraState());
        telemetry.addData("Fresh target", fresh);
        telemetry.addData("Detected contours", result.contourCount);
        telemetry.addData("Processing ms", "%.1f", camera.getProcessingMs());
        telemetry.addData("Classification", result.valid ? result.label : "NONE");
        telemetry.addData("Center", "%.0f, %.0f", result.centerX, result.centerY);
        telemetry.addData("dx / dy px", "%.1f / %.1f", result.dxPx, result.dyPx);
        telemetry.addData("Confidence", "%.2f", result.confidence);
        telemetry.addData("Stable", "%d/5", result.stableFrames);
        telemetry.addData("Fast centering", result.fastCentering);
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
