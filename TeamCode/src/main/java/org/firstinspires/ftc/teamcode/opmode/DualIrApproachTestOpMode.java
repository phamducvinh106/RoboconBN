package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.RobotHardware;

@TeleOp(name = "Dual IR Slow Approach Test", group = "Test")
public final class DualIrApproachTestOpMode extends LinearOpMode {
    private static final double APPROACH_POWER = 0.08;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        MecanumDrive drive = new MecanumDrive(
                hardwareMap, "leftfront", "rightfront", "leftback", "rightback", robot.localizer);
        boolean approach = true;
        boolean lastA = false;

        telemetry.addLine("Auto approach: starts when both IR are inactive");
        telemetry.addLine("A: pause/resume | stops when both IR sensors activate");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            boolean leftActive = !robot.leftIR.getState();
            boolean rightActive = !robot.rightIR.getState();
            boolean bothActive = leftActive && rightActive;
            if (gamepad1.a && !lastA) approach = !approach;
            lastA = gamepad1.a;
            if (!leftActive && !rightActive && !bothActive) approach = true;
            if (bothActive) approach = false;
            if (approach) drive.robotCentric(APPROACH_POWER, 0.0, 0.0);
            else drive.stop();

            telemetry.addData("leftIR", "%s (raw=%s)", leftActive ? "ACTIVE" : "inactive", robot.leftIR.getState());
            telemetry.addData("rightIR", "%s (raw=%s)", rightActive ? "ACTIVE" : "inactive", robot.rightIR.getState());
            telemetry.addData("both active", bothActive);
            telemetry.addData("approach", approach);
            telemetry.addData("power", approach && !bothActive ? APPROACH_POWER : 0.0);
            telemetry.update();
            idle();
        }
        drive.stop();
        robot.stopActuators();
    }
}
