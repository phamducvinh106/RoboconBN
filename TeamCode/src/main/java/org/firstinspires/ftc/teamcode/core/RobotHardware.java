package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public final class RobotHardware {
    public enum ElevatorState {
        HOME(0), READY1(844), LIFT1(1781), READY2(4688), LIFT2(5625);

        final int steps;

        ElevatorState(int steps) {
            this.steps = steps;
        }
    }

    private static final long STEP_PULSE_NS = 1_000_000L;
    private static final long ELEVATOR_TIMEOUT_NS = 8_000_000_000L;
    private int elevatorStepPosition;
    private boolean elevatorPositionKnown;

    public final DcMotorEx leftfront;
    public final DcMotorEx leftback;
    public final DcMotorEx rightfront;
    public final DcMotorEx rightback;
    public final Servo servoLeft;
    public final Servo servoRight;
    public final DigitalChannel leftIR;
    public final DigitalChannel rightIR;
    public final DigitalChannel step;
    public final DigitalChannel dir;
    public final DigitalChannel endstop1;
    public final WebcamName webcam1;
    public final IMU imu;
    public final Localizer localizer;

    public RobotHardware(HardwareMap hardwareMap) {
        leftfront = hardwareMap.get(DcMotorEx.class, "leftfront");
        leftback = hardwareMap.get(DcMotorEx.class, "leftback");
        rightfront = hardwareMap.get(DcMotorEx.class, "rightfront");
        rightback = hardwareMap.get(DcMotorEx.class, "rightback");
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");
        leftIR = input(hardwareMap, "leftIR");
        rightIR = input(hardwareMap, "rightIR");
        step = output(hardwareMap, "step");
        dir = output(hardwareMap, "dir");
        endstop1 = input(hardwareMap, "endstop1");
        webcam1 = hardwareMap.get(WebcamName.class, "webcam1");
        imu = hardwareMap.get(IMU.class, "imu");

        stopActuators();
        // Contract: Localizer(parallelPod, perpendicularPod) means leftfront=forward, rightfront=strafe.
        localizer = new Localizer(
                leftfront,
                rightfront,
                imu,
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
    }

    public boolean cargoReady() {
        return !leftIR.getState() && !rightIR.getState();
    }

    public boolean elevatorHomed() {
        return !endstop1.getState();
    }

    public boolean homeElevator() {
        dir.setState(false);
        long deadline = System.nanoTime() + ELEVATOR_TIMEOUT_NS;
        while (!elevatorHomed() && System.nanoTime() < deadline) {
            pulseStep();
        }
        step.setState(false);
        elevatorStepPosition = 0;
        elevatorPositionKnown = elevatorHomed();
        return elevatorPositionKnown;
    }

    public boolean moveElevatorToReady1() {
        return moveElevatorTo(ElevatorState.READY1);
    }

    public boolean moveElevatorToLift1() {
        return moveElevatorTo(ElevatorState.LIFT1);
    }

    public boolean moveElevatorToReady2() {
        return moveElevatorTo(ElevatorState.READY2);
    }

    public boolean moveElevatorToLift2() {
        return moveElevatorTo(ElevatorState.LIFT2);
    }

    public boolean moveElevatorToHome() {
        return homeElevator();
    }

    private boolean moveElevatorTo(ElevatorState target) {
        if (!elevatorPositionKnown && !elevatorHomed() && !homeElevator()) {
            return false;
        }
        if (target == ElevatorState.HOME) {
            return homeElevator();
        }

        int delta = target.steps - elevatorStepPosition;
        dir.setState(delta >= 0);
        long deadline = System.nanoTime() + ELEVATOR_TIMEOUT_NS;
        while (elevatorStepPosition != target.steps && System.nanoTime() < deadline) {
            if (delta < 0 && elevatorHomed()) {
                elevatorStepPosition = 0;
                break;
            }
            pulseStep();
            elevatorStepPosition += delta >= 0 ? 1 : -1;
        }
        step.setState(false);
        boolean reached = elevatorStepPosition == target.steps;
        elevatorPositionKnown = reached;
        return reached;
    }

    private void pulseStep() {
        step.setState(true);
        long deadline = System.nanoTime() + STEP_PULSE_NS;
        while (System.nanoTime() < deadline) {
            Thread.yield();
        }
        step.setState(false);
        long lowDeadline = System.nanoTime() + STEP_PULSE_NS;
        while (System.nanoTime() < lowDeadline) {
            Thread.yield();
        }
    }

    public void stopActuators() {
        leftfront.setPower(0);
        leftback.setPower(0);
        rightfront.setPower(0);
        rightback.setPower(0);
        step.setState(false);
        dir.setState(false);
    }

    private static DigitalChannel input(HardwareMap map, String name) {
        DigitalChannel channel = map.get(DigitalChannel.class, name);
        channel.setMode(DigitalChannel.Mode.INPUT);
        return channel;
    }

    private static DigitalChannel output(HardwareMap map, String name) {
        DigitalChannel channel = map.get(DigitalChannel.class, name);
        channel.setMode(DigitalChannel.Mode.OUTPUT);
        channel.setState(false);
        return channel;
    }
}
