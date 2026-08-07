package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Localizer;
import org.firstinspires.ftc.teamcode.core.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.RobotHardware;

@TeleOp(name = "PID Tuning", group = "Calibration")
public final class PidTuningOpMode extends LinearOpMode {
    private double positionKp = MecanumDrive.DEFAULT_POS_KP;
    private double positionKi = MecanumDrive.DEFAULT_POS_KI;
    private double positionKd = MecanumDrive.DEFAULT_POS_KD;
    private double headingKp = MecanumDrive.DEFAULT_HEAD_KP;
    private double headingKi = MecanumDrive.DEFAULT_HEAD_KI;
    private double headingKd = MecanumDrive.DEFAULT_HEAD_KD;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        Localizer localizer = robot.localizer;
        MecanumDrive drive = new MecanumDrive(
                hardwareMap, "leftfront", "rightfront", "leftback", "rightback", localizer
        );
        drive.setTolerance(2.0, 2.0);
        drive.setPowerLimits(0.3, 0.3);
        applyGains(drive);

        telemetry.addLine("PID TUNING");
        telemetry.addLine("A: forward target | B: strafe-right target | X: rotate target");
        telemetry.addLine("A: forward 50cm | B: strafe right 50cm | X: rotate 90deg");
        telemetry.addLine("D-pad up/down: position Kp +/-");
        telemetry.addLine("D-pad left/right: heading Kp +/-");
        telemetry.addLine("LB/RB: position Kd +/- | LT/RT: heading Kd +/-");
        telemetry.addLine("Y: reset pose/gains | 30% power limit");
        telemetry.update();
        waitForStart();

        boolean previousA = false;
        boolean previousB = false;
        boolean previousX = false;
        boolean previousY = false;
        long lastTuneMs = 0;

        try {
            while (opModeIsActive()) {
                localizer.update();
                long now = System.currentTimeMillis();
                boolean a = gamepad1.a;
                boolean b = gamepad1.b;
                boolean x = gamepad1.x;
                boolean y = gamepad1.y;

                if (a && !previousA) {
                    localizer.resetPose();
                    drive.goToPosition(0, 50, 0);
                }
                if (b && !previousB) {
                    localizer.resetPose();
                    drive.goToPosition(50, 0, 0);
                }
                if (x && !previousX) {
                    localizer.resetPoseAndHeading();
                    drive.goToPosition(0, 0, 90);
                }
                if (y && !previousY) {
                    localizer.resetPoseAndHeading();
                    positionKp = MecanumDrive.DEFAULT_POS_KP;
                    positionKi = MecanumDrive.DEFAULT_POS_KI;
                    positionKd = MecanumDrive.DEFAULT_POS_KD;
                    headingKp = MecanumDrive.DEFAULT_HEAD_KP;
                    headingKi = MecanumDrive.DEFAULT_HEAD_KI;
                    headingKd = MecanumDrive.DEFAULT_HEAD_KD;
                    applyGains(drive);
                }
                previousA = a;
                previousB = b;
                previousX = x;
                previousY = y;

                if (now - lastTuneMs >= 150) {
                    double step = gamepad1.left_bumper || gamepad1.right_bumper
                            || gamepad1.left_trigger > 0.5 || gamepad1.right_trigger > 0.5
                            ? 0.001 : 0.005;
                    if (gamepad1.dpad_up) positionKp += step;
                    if (gamepad1.dpad_down) positionKp = Math.max(0, positionKp - step);
                    if (gamepad1.dpad_right) headingKp += step;
                    if (gamepad1.dpad_left) headingKp = Math.max(0, headingKp - step);
                    if (gamepad1.left_bumper) positionKd += 0.001;
                    if (gamepad1.right_bumper) positionKd = Math.max(0, positionKd - 0.001);
                    if (gamepad1.left_trigger > 0.5) headingKd += 0.001;
                    if (gamepad1.right_trigger > 0.5) headingKd = Math.max(0, headingKd - 0.001);
                    applyGains(drive);
                    lastTuneMs = now;
                }

                drive.update();
                telemetry.addData("target", "configured through A/B/X buttons");
                telemetry.addData("pose", "X %.2f / Y %.2f / H %.2f", localizer.getX(), localizer.getY(), localizer.getHeadingDeg());
                telemetry.addData("position PID", "Kp %.4f Ki %.4f Kd %.4f", positionKp, positionKi, positionKd);
                telemetry.addData("heading PID", "Kp %.4f Ki %.4f Kd %.4f", headingKp, headingKi, headingKd);
                telemetry.addData("power limit", "position 30%% / heading 30%%");
                telemetry.addData("at target", drive.atTarget());
                telemetry.update();
                sleep(20);
            }
        } finally {
            drive.stop();
        }
    }

    private void applyGains(MecanumDrive drive) {
        drive.setPositionGains(positionKp, positionKi, positionKd);
        drive.setHeadingGains(headingKp, headingKi, headingKd);
    }
}
