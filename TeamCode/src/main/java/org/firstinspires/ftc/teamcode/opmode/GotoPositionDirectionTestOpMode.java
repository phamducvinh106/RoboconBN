package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Localizer;
import org.firstinspires.ftc.teamcode.core.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.RobotHardware;

@TeleOp(name = "Goto Position Direction Test", group = "Calibration")
public final class GotoPositionDirectionTestOpMode extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        Localizer localizer = robot.localizer;
        MecanumDrive drive = new MecanumDrive(
                hardwareMap, "leftfront", "rightfront", "leftback", "rightback", localizer);
        drive.setPowerLimits(0.30, 0.30);

        telemetry.addLine("A: forward +50cm | B: right +50cm | X: rotate +90deg | Y: reset pose");
        telemetry.update();
        waitForStart();

        boolean pa = false, pb = false, px = false, py = false;
        try {
            while (opModeIsActive()) {
                boolean a = gamepad1.a, b = gamepad1.b, x = gamepad1.x, y = gamepad1.y;
                if (a && !pa) {
                    localizer.resetPose();
                    drive.goToPosition(0, 50, 0);
                }
                if (b && !pb) {
                    localizer.resetPose();
                    drive.goToPosition(50, 0, 0);
                }
                if (x && !px) {
                    localizer.resetPoseAndHeading();
                    drive.goToPosition(0, 0, 90);
                }
                if (y && !py) localizer.resetPoseAndHeading();
                pa = a;
                pb = b;
                px = x;
                py = y;

                localizer.update();
                drive.update();

                telemetry.addData("state", drive.getState());
                telemetry.addData("pose", "X %.1f Y %.1f H %.1f",
                        localizer.getX(), localizer.getY(), localizer.getHeadingDeg());
                telemetry.addData("field err", "X %.1f Y %.1f H %.1f",
                        drive.getLastFieldErrorX(), drive.getLastFieldErrorY(),
                        drive.getLastHeadingErrorDeg());
                telemetry.addData("robot cmd", "fwd %.2f str %.2f rot %.2f",
                        drive.getLastRobotForward(), drive.getLastRobotStrafe(),
                        drive.getLastRobotRotate());
                telemetry.addData("motor", "FL %.2f FR %.2f BL %.2f BR %.2f",
                        drive.getLastFlPower(), drive.getLastFrPower(),
                        drive.getLastBlPower(), drive.getLastBrPower());
                telemetry.addData("at target", drive.atTarget());
                telemetry.update();
                sleep(20);
            }
        } finally {
            drive.stop();
        }
    }
}
