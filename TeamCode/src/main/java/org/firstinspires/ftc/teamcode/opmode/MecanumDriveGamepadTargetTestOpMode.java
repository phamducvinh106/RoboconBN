package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Localizer;
import org.firstinspires.ftc.teamcode.core.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.RobotHardware;

/** Low-power gamepad target test with encoder jump safety stop. */
@TeleOp(name = "Mecanum Target Gamepad Test", group = "Calibration")
public final class MecanumDriveGamepadTargetTestOpMode extends LinearOpMode {
    private static final double POWER_LIMIT = 0.3;
    private static final double TARGET_STEP_CM = 10.0;
    private static final double TARGET_STEP_DEG = 15.0;
    private static final double PID_STEP = 0.001;
    private static final long PID_REPEAT_MS = 150;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        Localizer localizer = robot.localizer;
        MecanumDrive drive = new MecanumDrive(
                hardwareMap, "leftfront", "rightfront", "leftback", "rightback", localizer);
        drive.setPowerLimits(POWER_LIMIT, POWER_LIMIT);

        double targetX = 0.0;
        double targetY = 0.0;
        double targetHeading = 0.0;
        double positionKp = MecanumDrive.DEFAULT_POS_KP;
        double positionKi = MecanumDrive.DEFAULT_POS_KI;
        double positionKd = MecanumDrive.DEFAULT_POS_KD;
        double headingKp = MecanumDrive.DEFAULT_HEAD_KP;
        double headingKi = MecanumDrive.DEFAULT_HEAD_KI;
        double headingKd = MecanumDrive.DEFAULT_HEAD_KD;
        boolean previousA = false;
        boolean previousB = false;
        boolean previousY = false;
        long lastPidTuneMs = 0;

        telemetry.addLine("D-pad: target X/Y; LB/RB: heading; A start; B stop; Y reset");
        telemetry.addLine("D-pad tune: Up/Down pos Kp; Left/Right head Kp");
        telemetry.addLine("LB/RB + LT/RT: pos Kd +/-; head Kd +/-");
        telemetry.addData("power", "%.0f%%", POWER_LIMIT * 100.0);
        telemetry.update();
        waitForStart();

        try {
            while (opModeIsActive()) {
                boolean a = gamepad1.a;
                boolean b = gamepad1.b;
                boolean x = gamepad1.x;
                boolean y = gamepad1.y;

                boolean tuning = gamepad1.start;
                if (!tuning) {
                    if (gamepad1.dpad_right) targetX += TARGET_STEP_CM;
                    if (gamepad1.dpad_left) targetX -= TARGET_STEP_CM;
                    if (gamepad1.dpad_up) targetY += TARGET_STEP_CM;
                    if (gamepad1.dpad_down) targetY -= TARGET_STEP_CM;
                    if (gamepad1.left_bumper) targetHeading -= TARGET_STEP_DEG;
                    if (gamepad1.right_bumper) targetHeading += TARGET_STEP_DEG;
                }

                long now = System.currentTimeMillis();
                if (tuning && now - lastPidTuneMs >= PID_REPEAT_MS) {
                    double step = gamepad1.left_trigger > 0.5 || gamepad1.right_trigger > 0.5
                            ? PID_STEP * 0.1 : PID_STEP;
                    if (gamepad1.dpad_up) positionKp += step;
                    if (gamepad1.dpad_down) positionKp = Math.max(0.0, positionKp - step);
                    if (gamepad1.dpad_right) headingKp += step;
                    if (gamepad1.dpad_left) headingKp = Math.max(0.0, headingKp - step);
                    if (gamepad1.left_bumper) positionKd += step;
                    if (gamepad1.right_bumper) positionKd = Math.max(0.0, positionKd - step);
                    if (gamepad1.left_trigger > 0.5) headingKd += step;
                    if (gamepad1.right_trigger > 0.5) headingKd = Math.max(0.0, headingKd - step);
                    drive.setPositionGains(positionKp, positionKi, positionKd);
                    drive.setHeadingGains(headingKp, headingKi, headingKd);
                    lastPidTuneMs = now;
                }

                if (y && !previousY) {
                    localizer.resetPoseAndHeading();
                    targetX = targetY = targetHeading = 0.0;
                }
                if (b && !previousB) drive.stop();
                if (a && !previousA) drive.goToPosition(targetX, targetY, targetHeading);

                previousA = a;
                previousB = b;
                previousY = y;

                localizer.update();
                drive.update();

                telemetry.addData("target", "X %.1f  Y %.1f  H %.1f", targetX, targetY, targetHeading);
                telemetry.addData("pose", "X %.1f  Y %.1f  H %.1f",
                        localizer.getX(), localizer.getY(), localizer.getHeadingDeg());
                telemetry.addData("state", drive.getState());
                telemetry.addData("PID", "pos %.4f / %.4f / %.4f | head %.4f / %.4f / %.4f",
                        positionKp, positionKi, positionKd, headingKp, headingKi, headingKd);
                telemetry.addData("tuning", tuning ? "START held: edit Kp/Kd" : "START released: edit target");
                telemetry.addData("power", "FL %.2f FR %.2f BL %.2f BR %.2f",
                        drive.getLastFlPower(), drive.getLastFrPower(),
                        drive.getLastBlPower(), drive.getLastBrPower());
                telemetry.addLine("A start | B stop | Y reset | hold START for PID tune");
                telemetry.update();
                sleep(50);
            }
        } finally {
            drive.stop();
        }
    }
}
