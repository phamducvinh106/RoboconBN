package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.teamcode.core.CameraAdapterManager;
import org.firstinspires.ftc.teamcode.core.CameraChannel;
import org.firstinspires.ftc.teamcode.core.CameraFrameContract;
import org.firstinspires.ftc.teamcode.core.EncoderLocalizerManager;
import org.firstinspires.ftc.teamcode.core.EndstopManager;
import org.firstinspires.ftc.teamcode.core.ForkServoManager;
import org.firstinspires.ftc.teamcode.core.HardwareContracts;
import org.firstinspires.ftc.teamcode.core.IrSensorManager;
import org.firstinspires.ftc.teamcode.core.LiftingSequenceConfig;
import org.firstinspires.ftc.teamcode.core.Pi5UartCameraTransport;
import org.firstinspires.ftc.teamcode.core.ReleaseBackoutSensorManager;
import org.firstinspires.ftc.teamcode.core.RobotHardware;
import org.firstinspires.ftc.teamcode.core.StepperElevatorManager;

@TeleOp(name = "Lifting Hardware Communication Test", group = "Test")
public final class LiftingHardwareTestOpMode extends LinearOpMode {
    private static final long CAMERA_MAX_AGE_NS = 250_000_000L;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        DigitalChannel step = robot.step;
        DigitalChannel dir = robot.dir;
        EndstopManager endstop = new EndstopManager(() -> !robot.endstop1.getState());
        StepperElevatorManager elevator = new StepperElevatorManager(
                channel(step), channel(dir), endstop,
                LiftingSequenceConfig.STEP_HIGH_NS, LiftingSequenceConfig.STEP_LOW_NS,
                LiftingSequenceConfig.ElevatorTarget.LIFT2.steps);
        ForkServoManager fork = new ForkServoManager(
                servo(robot.servoLeft), servo(robot.servoRight),
                LiftingSequenceConfig.PLACE_LEFT, LiftingSequenceConfig.PLACE_RIGHT,
                LiftingSequenceConfig.HOLD_LEFT, LiftingSequenceConfig.HOLD_RIGHT);
        IrSensorManager ir = new IrSensorManager(
                () -> !robot.leftIR.getState(), () -> !robot.rightIR.getState());
        EncoderLocalizerManager pose = new EncoderLocalizerManager(() -> {
            robot.localizer.update();
            return new HardwareContracts.PoseReading(robot.localizer.getX(), robot.localizer.getY(),
                    robot.localizer.getHeadingDeg(), System.nanoTime());
        });
        ReleaseBackoutSensorManager release = new ReleaseBackoutSensorManager(
                () -> !robot.endstop1.getState(), poseSource(pose));
        CameraAdapterManager cameras = new CameraAdapterManager(
                new Pi5UartCameraTransport(System::nanoTime), CAMERA_MAX_AGE_NS);
        hardwareMap.get(WebcamName.class, "webcam2");

        telemetry.addLine("LOW POWER HARDWARE COMMUNICATION TEST");
        telemetry.addLine("A=home  B=READY1  X=PLACE  Y=HOLD  Dpad=IR/pose  LB/RB=cameras");
        telemetry.addLine("Pi5 UART parser and OpenCV deferred; no state machine or placement");
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
                CameraFrameContract left = cameras.reading(CameraChannel.WEBCAM1);
                CameraFrameContract right = cameras.reading(CameraChannel.WEBCAM2);
                HardwareContracts.PoseReading reading = pose.reading();
                telemetry.addData("devices", "step dir endstop1 servoLeft servoRight leftIR rightIR webcam1 webcam2");
                telemetry.addData("step/dir", "%s / %s (pulsing=%s)", step.getState(), dir.getState(), elevator.pulsing());
                telemetry.addData("endstop", "active=%s polarity=active-low", endstop.active());
                telemetry.addData("elevator", "position=%d known=%s", elevator.position(), elevator.positionKnown());
                telemetry.addData("fork", "%s PLACE=(%.2f,%.2f) HOLD=(%.2f,%.2f)", fork.pose(), LiftingSequenceConfig.PLACE_LEFT, LiftingSequenceConfig.PLACE_RIGHT, LiftingSequenceConfig.HOLD_LEFT, LiftingSequenceConfig.HOLD_RIGHT);
                telemetry.addData("IR", "left=%s right=%s both=%s polarity=active-low", ir.leftActive(), ir.rightActive(), ir.bothActive());
                telemetry.addData("pose", "valid=%s x=%.2f y=%.2f heading=%.2f", reading.valid, reading.xCm, reading.yCm, reading.headingDeg);
                telemetry.addData("release/backout", "released=%s x=%.2f y=%.2f", release.released(), release.reading().xCm, release.reading().yCm);
                telemetry.addData("webcam1", "valid=%s fresh=%s ageNs=%d", left.valid, cameras.movementAuthorized(CameraChannel.WEBCAM1, now), now - left.timestampNs);
                telemetry.addData("webcam2", "valid=%s fresh=%s ageNs=%d", right.valid, cameras.movementAuthorized(CameraChannel.WEBCAM2, now), now - right.timestampNs);
                telemetry.addData("config", "version=external JSON pending; timing=%d/%d ns", LiftingSequenceConfig.STEP_HIGH_NS, LiftingSequenceConfig.STEP_LOW_NS);
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
