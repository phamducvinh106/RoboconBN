package org.firstinspires.ftc.teamcode.opmode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.odometry.Localizer;
import org.firstinspires.ftc.teamcode.core.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.RobotHardware;

@TeleOp(name = "Mecanum Drive Gamepad Test", group = "Test")
public final class MecanumDriveGamepadTestOpMode extends LinearOpMode {
    private static final double MAX_POWER = 0.30;
    private static final double DEADZONE = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        Localizer localizer = robot.localizer;
        MecanumDrive drive = new MecanumDrive(
                hardwareMap, "leftfront", "rightfront", "leftback", "rightback", localizer
        );
        drive.setPowerLimits(MAX_POWER, MAX_POWER);

        telemetry.addLine("MECANUM DRIVE GAMEPAD TEST");
        telemetry.addLine("Left stick Y: forward/back | Left stick X: strafe");
        telemetry.addLine("Right stick X: rotate | A: stop | Y: reset pose");
        telemetry.addData("max power", "%.0f%%", MAX_POWER * 100.0);
        telemetry.update();
        waitForStart();

        try {
            while (opModeIsActive()) {
                double forward = clip(-gamepad1.left_stick_y);
                double strafe = clip(gamepad1.left_stick_x);
                double rotate = clip(gamepad1.right_stick_x);

                if (gamepad1.a) {
                    forward = 0.0;
                    strafe = 0.0;
                    rotate = 0.0;
                }
                if (gamepad1.y) localizer.resetPoseAndHeading();

                localizer.update();
                drive.driveRobotCentric(forward, strafe, rotate);

                telemetry.addData("command", "forward %.2f / strafe %.2f / rotate %.2f",
                        forward, strafe, rotate);
                telemetry.addData("motors", "FL %.2f FR %.2f BL %.2f BR %.2f",
                        drive.getLastFlPower(), drive.getLastFrPower(),
                        drive.getLastBlPower(), drive.getLastBrPower());
                telemetry.addData("pose", "X %.2f / Y %.2f / H %.2f",
                        localizer.getX(), localizer.getY(), localizer.getHeadingDeg());
                telemetry.addData("local delta", "forward %.2f / left %.2f",
                        localizer.getLastForwardLocalCm(), localizer.getLastLeftLocalCm());
                telemetry.update();
                sleep(20);
            }
        } finally {
            drive.stop();
        }
    }

    private static double clip(double value) {
        if (Math.abs(value) < DEADZONE) return 0.0;
        return Math.max(-MAX_POWER, Math.min(MAX_POWER, value * MAX_POWER));
    }
}
