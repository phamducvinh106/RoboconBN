package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.core.LiftingSequenceConfig;
import org.firstinspires.ftc.teamcode.core.LiftingSequenceStateMachine;
import org.firstinspires.ftc.teamcode.core.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.RobotHardware;

@Autonomous(name = "Lifting Sequence", group = "Autonomous")
public final class LiftingSequenceOpMode extends LinearOpMode {
    @Override public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, "leftfront", "rightfront", "leftback", "rightback", robot.localizer);
        LiftingSequenceStateMachine machine = new LiftingSequenceStateMachine(
                System::nanoTime,
                new LiftingSequenceStateMachine.Actuators() {
                    public void stop() { robot.stopActuators(); drive.stop(); }
                    public boolean home() { return robot.homeElevator(); }
                    public boolean elevatorAt(LiftingSequenceConfig.ElevatorTarget target) { return robot.stepElevatorToward(target.steps, System.nanoTime()); }
                    public void setFork(LiftingSequenceConfig.ForkPose pose) {
                        robot.servoLeft.setPosition(pose == LiftingSequenceConfig.ForkPose.HOLD ? LiftingSequenceConfig.HOLD_LEFT : LiftingSequenceConfig.PLACE_LEFT);
                        robot.servoRight.setPosition(pose == LiftingSequenceConfig.ForkPose.HOLD ? LiftingSequenceConfig.HOLD_RIGHT : LiftingSequenceConfig.PLACE_RIGHT);
                    }
                    public void drive(double forward, double strafe) { drive.driveRobotCentric(forward, strafe, 0); }
                    public void stopDrive() { drive.stop(); }
                });
        waitForStart();
        try {
            while (opModeIsActive() && !isStopRequested() && machine.getState() != LiftingSequenceStateMachine.State.SAFE_STOP) {
                robot.localizer.update();
                drive.update();
                machine.tick();
                telemetry.addData("state", machine.getState());
                telemetry.addData("shelf/level", "%d/%d", machine.getShelf(), machine.getLevel());
                telemetry.addData("cycles", machine.getCompletedCycles());
                telemetry.addData("elapsedMs", machine.elapsedNs() / 1_000_000L);
                telemetry.addData("pose", "%.1f, %.1f, %.1f", robot.localizer.getX(), robot.localizer.getY(), robot.localizer.getHeadingDeg());
                telemetry.addData("retries/failure", "%d/%s", machine.getRetries(), machine.getFailure());
                telemetry.update();
                idle();
            }
        } finally { drive.stop(); robot.stopActuators(); }
    }
}
