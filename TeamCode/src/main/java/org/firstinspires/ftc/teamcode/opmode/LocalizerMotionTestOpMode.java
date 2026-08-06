package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Localizer;
import org.firstinspires.ftc.teamcode.core.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.RobotHardware;

@TeleOp(name = "Localizer Motion Test", group = "Calibration")
public final class LocalizerMotionTestOpMode extends LinearOpMode {
    private static final double POWER = 0.18;
    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        Localizer localizer = robot.localizer;
        MecanumDrive drive = new MecanumDrive(
                hardwareMap,
                "leftfront", "rightfront", "leftback", "rightback",
                localizer
        );

        telemetry.addLine("LOCALIZER MOTION TEST");
        telemetry.addLine("A: forward | B: backward | X: strafe left");
        telemetry.addLine("Y: strafe right | D-pad left/right: rotate");
        telemetry.addLine("Release button to stop. Test on clear floor.");
        telemetry.update();
        waitForStart();

        try {
            while (opModeIsActive()) {
                localizer.update();
                double forward = 0.0;
                double strafe = 0.0;
                double rotate = 0.0;

                if (gamepad1.a) forward = -POWER;
                if (gamepad1.b) forward = POWER;
                if (gamepad1.x) strafe = POWER;
                if (gamepad1.y) strafe = -POWER;
                if (gamepad1.dpad_left) rotate = POWER;
                if (gamepad1.dpad_right) rotate = -POWER;

                drive.driveRobotCentric(forward, strafe, rotate);
                telemetry.addData("command", "forward %.2f / strafe %.2f / rotate %.2f",
                        forward, strafe, rotate);
                telemetry.addData("pose", "X %.2f cm / Y %.2f cm / heading %.2f deg",
                        localizer.getX(), localizer.getY(), localizer.getHeadingDeg());
                telemetry.addData("delta", "forward %.3f / left %.3f cm",
                        localizer.getLastForwardLocalCm(), localizer.getLastLeftLocalCm());
                telemetry.addData("encoder", "parallel %.3f / perpendicular %.3f cm",
                        localizer.getAccumParallelCm(), localizer.getAccumPerpendicularCm());
                telemetry.addData("IMU", "yaw %.2f deg", localizer.getHeadingDeg());
                telemetry.update();
                sleep(20);
            }
        } finally {
            drive.stop();
        }
    }
}
