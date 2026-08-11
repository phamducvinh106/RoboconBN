package org.firstinspires.ftc.teamcode.opmode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.odometry.Localizer;
import org.firstinspires.ftc.teamcode.core.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.RobotHardware;

@TeleOp(name = "Odometry Calibration Clockwise", group = "Calibration")
public final class OdometryCalibrationOpMode extends LinearOpMode {
    private static final double ROTATE_POWER = 0.18;
    private static final long ROTATE_TIMEOUT_MS = 15_000;
    private static final double TARGET_HEADING_DEG = -360.0;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        Localizer localizer = robot.localizer;
        MecanumDrive drive = new MecanumDrive(
                hardwareMap,
                "leftfront", "rightfront", "leftback", "rightback",
                localizer
        );

        try {
            Localizer.Calibration calibration = localizer.getCalibration();
            telemetry.addLine("CLOCKWISE ODOMETRY CALIBRATION");
            telemetry.addLine("Secure robot. Pods down. START to rotate 360 degrees.");
            telemetry.addData("parallel", "leftfront / forward");
            telemetry.addData("perpendicular", "rightfront / strafe");
            telemetry.addData("wheel", "%.3f cm, %.1f ticks/rev",
                    calibration.wheelDiameterCm,
                    calibration.ticksPerRevolution);
            telemetry.addData("signs", "parallel %.0f, perpendicular %.0f, heading %.0f",
                    calibration.parallelEncoderSign,
                    calibration.perpendicularEncoderSign,
                    calibration.headingSign);
            telemetry.addData("offsets", "Y %.3f cm, X %.3f cm",
                    calibration.parallelYOffsetCm,
                    calibration.perpendicularXOffsetCm);
            telemetry.addData("IMU", calibration.imuOrientation);
            telemetry.update();

            waitForStart();
            if (isStopRequested()) return;

            localizer.resetPoseAndHeading();
            long deadline = System.nanoTime() + ROTATE_TIMEOUT_MS * 1_000_000L;
            boolean headingMovedClockwise = false;
            double previousHeadingDeg = localizer.getHeadingDeg();

            while (opModeIsActive()
                    && !isStopRequested()
                    && System.nanoTime() < deadline) {
                drive.setRawPowers(
                        -ROTATE_POWER,
                        ROTATE_POWER,
                        -ROTATE_POWER,
                        ROTATE_POWER
                );

                localizer.update();
                double headingDeg = Math.toDegrees(localizer.getAccumHeadingRad());
                double currentHeadingDeg = localizer.getHeadingDeg();
                if (currentHeadingDeg < previousHeadingDeg) headingMovedClockwise = true;
                previousHeadingDeg = currentHeadingDeg;
                telemetry.addData("direction", headingMovedClockwise ? "CLOCKWISE confirmed" : "waiting for clockwise heading");
                telemetry.addData("heading", "%.2f / %.2f deg", headingDeg, TARGET_HEADING_DEG);
                telemetry.addData("encoder delta", "forward %.3f cm, left %.3f cm",
                        localizer.getLastForwardLocalCm(),
                        localizer.getLastLeftLocalCm());
                telemetry.addData("encoder total", "parallel %.3f cm, perpendicular %.3f cm",
                        localizer.getAccumParallelCm(),
                        localizer.getAccumPerpendicularCm());
                telemetry.addData("pose", "X %.3f, Y %.3f cm",
                        localizer.getX(), localizer.getY());
                telemetry.update();

                if (headingMovedClockwise && headingDeg <= TARGET_HEADING_DEG) break;
                idle();
            }
        } finally {
            drive.stop();
        }

        double suggestedY = localizer.getSuggestedParallelYOffsetCm();
        double suggestedX = localizer.getSuggestedPerpendicularXOffsetCm();
        telemetry.addLine("CALIBRATION COMPLETE");
        telemetry.addData("actual heading", "%.2f deg",
                Math.toDegrees(localizer.getAccumHeadingRad()));
        telemetry.addData("heading direction", "%.0f", localizer.getCalibration().headingSign);
        telemetry.addData("residual local", "forward %.3f / left %.3f cm",
                localizer.getLastForwardLocalCm(), localizer.getLastLeftLocalCm());
        telemetry.addData("parallel sign", "%.0f", localizer.getCalibration().parallelEncoderSign);
        telemetry.addData("perpendicular sign", "%.0f", localizer.getCalibration().perpendicularEncoderSign);
        telemetry.addData("heading sign", "%.0f", localizer.getCalibration().headingSign);
        telemetry.addData("suggested PARALLEL_Y_OFFSET_CM", "%.4f", suggestedY);
        telemetry.addData("suggested PERPENDICULAR_X_OFFSET_CM", "%.4f", suggestedX);
        telemetry.addLine("Copy values only after checking encoder signs and physical measurements.");
        telemetry.addLine("Press STOP to exit.");
        telemetry.update();

        while (opModeIsActive() && !isStopRequested()) idle();
    }
}
