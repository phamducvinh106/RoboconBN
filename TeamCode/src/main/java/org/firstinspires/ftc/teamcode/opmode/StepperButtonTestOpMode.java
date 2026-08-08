package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.core.LiftingSequenceConfig;
import org.firstinspires.ftc.teamcode.core.RobotConfigAssets;
import org.firstinspires.ftc.teamcode.core.RobotHardware;

@TeleOp(name = "Stepper Five Position Test", group = "Test")
public final class StepperButtonTestOpMode extends LinearOpMode {
    private static final LiftingSequenceConfig.ElevatorTarget[] POSITIONS = {
            LiftingSequenceConfig.ElevatorTarget.HOME,
            LiftingSequenceConfig.ElevatorTarget.READY1,
            LiftingSequenceConfig.ElevatorTarget.LIFT1,
            LiftingSequenceConfig.ElevatorTarget.READY2,
            LiftingSequenceConfig.ElevatorTarget.LIFT2
    };

    @Override
    public void runOpMode() throws InterruptedException {
        LiftingSequenceConfig config;
        try {
            config = RobotConfigAssets.load(hardwareMap.appContext.getAssets());
        } catch (Exception error) {
            telemetry.addData("SAFE_STOP", "invalid config: %s", error.getMessage());
            telemetry.update();
            return;
        }
        RobotHardware robot = new RobotHardware(hardwareMap, config);
        DigitalChannel step = robot.step;
        DigitalChannel dir = robot.dir;
        int selected = 0;
        boolean lastUp = false;
        boolean lastDown = false;
        boolean homed = false;

        telemetry.addLine("A: HOME  B: READY1  X: LIFT1  Y: READY2  D-pad UP: LIFT2");
        telemetry.addLine("D-pad DOWN: HOME");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            if (gamepad1.a) selected = 0;
            if (gamepad1.b) selected = 1;
            if (gamepad1.x) selected = 2;
            if (gamepad1.y) selected = 3;
            if (up && !lastUp) selected = 4;
            if (down && !lastDown) selected = 0;
            lastUp = up;
            lastDown = down;

            LiftingSequenceConfig.ElevatorTarget target = POSITIONS[selected];
            boolean reached;
            if (target == LiftingSequenceConfig.ElevatorTarget.HOME) {
                reached = robot.homeElevator();
                homed = reached;
            } else {
                reached = robot.stepElevatorToward(config.elevatorSteps(target), System.nanoTime());
            }

            telemetry.addData("target", "%d/%s", selected, target.name());
            telemetry.addData("reached", reached);
            telemetry.addData("endstop", robot.elevatorHomed());
            telemetry.addData("step", step.getState());
            telemetry.addData("dir", dir.getState());
            telemetry.addData("home complete", homed);
            telemetry.update();
            idle();
        }
        robot.stopActuators();
    }
}
