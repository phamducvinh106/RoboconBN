package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.core.EncoderLocalizerManager;
import org.firstinspires.ftc.teamcode.core.EndstopManager;
import org.firstinspires.ftc.teamcode.core.ForkServoManager;
import org.firstinspires.ftc.teamcode.core.HardwareContracts;
import org.firstinspires.ftc.teamcode.core.IrSensorManager;
import org.firstinspires.ftc.teamcode.core.LiftingSequenceConfig;
import org.firstinspires.ftc.teamcode.core.LiftingSequenceConfigLoader;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import org.firstinspires.ftc.teamcode.core.ReleaseBackoutSensorManager;
import org.firstinspires.ftc.teamcode.core.RobotHardware;
import org.firstinspires.ftc.teamcode.core.StepperElevatorManager;

@TeleOp(name = "Lifting Hardware Communication Test", group = "Test")
public final class LiftingHardwareTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LiftingSequenceConfig config;
        try (InputStream input = hardwareMap.appContext.getAssets().open("phase2-lifting-config.json")) {
            java.io.ByteArrayOutputStream output = new java.io.ByteArrayOutputStream(); byte[] buffer = new byte[1024]; int count; while ((count = input.read(buffer)) != -1) output.write(buffer, 0, count); config = LiftingSequenceConfigLoader.load(new String(output.toByteArray(), StandardCharsets.UTF_8));
        } catch (Exception error) {
            telemetry.addData("SAFE_STOP", "invalid config: %s", error.getMessage());
            telemetry.update();
            return;
        }
        RobotHardware robot = new RobotHardware(hardwareMap);
        DigitalChannel step = robot.step;
        DigitalChannel dir = robot.dir;
        EndstopManager endstop = new EndstopManager(() -> !robot.endstop1.getState());
        StepperElevatorManager elevator = new StepperElevatorManager(
                channel(step), channel(dir), endstop,
                config.stepHighNs, config.stepLowNs, config.lift2Steps);
        ForkServoManager fork = new ForkServoManager(
                servo(robot.servoLeft), servo(robot.servoRight),
                config.placeLeft, config.placeRight, config.holdLeft, config.holdRight);
        IrSensorManager ir = new IrSensorManager(
                () -> !robot.leftIR.getState(), () -> !robot.rightIR.getState());
        EncoderLocalizerManager pose = new EncoderLocalizerManager(() -> {
            robot.localizer.update();
            return new HardwareContracts.PoseReading(robot.localizer.getX(), robot.localizer.getY(),
                    robot.localizer.getHeadingDeg(), System.nanoTime());
        });
        ReleaseBackoutSensorManager release = new ReleaseBackoutSensorManager(
                () -> !robot.endstop1.getState(), poseSource(pose));
        telemetry.addLine("LOW POWER HARDWARE COMMUNICATION TEST");
        telemetry.addLine("A=home  B=READY1  X=PLACE  Y=HOLD  Dpad=IR/pose");
        telemetry.addLine("ORB camera tested via OrbTarget1TestOpMode");
        telemetry.update();
        waitForStart();
        boolean lastA = false, lastB = false, lastX = false, lastY = false;
        try {
            while (opModeIsActive() && !isStopRequested()) {
                long now = System.nanoTime();
                if (gamepad1.a && !lastA) elevator.moveToward(0, now);
                if (gamepad1.b && !lastB) elevator.moveToward(LiftingSequenceConfig.ElevatorTarget.READY1.steps, now);
                if (gamepad1.x && !lastX) fork.setPose(ForkServoManager.Pose.PLACE);
                if (gamepad1.y && !lastY) fork.setPose(ForkServoManager.Pose.HOLD);
                lastA = gamepad1.a; lastB = gamepad1.b; lastX = gamepad1.x; lastY = gamepad1.y;
                if (gamepad1.dpad_up) elevator.moveToward(LiftingSequenceConfig.ElevatorTarget.LIFT1.steps, now);
                if (gamepad1.dpad_down) elevator.stop();
                HardwareContracts.PoseReading reading = pose.reading();
                telemetry.addData("devices", "step dir endstop1 servoLeft servoRight leftIR rightIR webcam1 webcam2");
                telemetry.addData("step/dir", "%s / %s (pulsing=%s)", step.getState(), dir.getState(), elevator.pulsing());
                telemetry.addData("endstop", "active=%s polarity=active-low", endstop.active());
                telemetry.addData("elevator", "position=%d known=%s", elevator.position(), elevator.positionKnown());
                telemetry.addData("fork", "%s PLACE=(%.2f,%.2f) HOLD=(%.2f,%.2f)", fork.pose(), LiftingSequenceConfig.PLACE_LEFT, LiftingSequenceConfig.PLACE_RIGHT, LiftingSequenceConfig.HOLD_LEFT, LiftingSequenceConfig.HOLD_RIGHT);
                telemetry.addData("IR", "left=%s right=%s both=%s polarity=active-low", ir.leftActive(), ir.rightActive(), ir.bothActive());
                telemetry.addData("pose", "valid=%s x=%.2f y=%.2f heading=%.2f", reading.valid, reading.xCm, reading.yCm, reading.headingDeg);
                telemetry.addData("release/backout", "released=%s x=%.2f y=%.2f", release.released(), release.reading().xCm, release.reading().yCm);
                telemetry.addData("config", "version=%d fingerprint=%s timing=%d/%d ns", config.version, config.fingerprint, config.stepHighNs, config.stepLowNs);
                telemetry.update();
                idle();
            }
        } finally {
            elevator.stop();
            robot.stopActuators();
        }
    }

    private static HardwareContracts.BinaryChannel channel(DigitalChannel channel) {
        return new HardwareContracts.BinaryChannel() { public boolean high() { return channel.getState(); } public void setHigh(boolean high) { channel.setState(high); } };
    }
    private static HardwareContracts.ServoChannel servo(Servo servo) { return servo::setPosition; }
    private static HardwareContracts.PoseSource poseSource(EncoderLocalizerManager pose) { return pose::reading; }
}
