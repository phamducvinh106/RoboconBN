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
        ElevatorState(int steps) { this.steps = steps; }
    }

    private long pulseLowUntilNs;
    private long pulseHighUntilNs;
    private boolean pulseActive;
    private boolean commandUp;
    private static final long STEP_PULSE_NS = LiftingSequenceConfig.STEP_HIGH_NS;
    private static final long STEP_LOW_NS = LiftingSequenceConfig.STEP_LOW_NS;
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
        return stepElevatorToward(0, System.nanoTime());
    }

    public boolean stepElevatorToward(int targetSteps, long nowNs) {
        if (targetSteps < 0 || targetSteps > LiftingSequenceConfig.ElevatorTarget.LIFT2.steps) {
            throw new IllegalArgumentException("elevator target out of bounds");
        }
        if (targetSteps == 0 && elevatorHomed()) {
            step.setState(false);
            elevatorStepPosition = 0;
            elevatorPositionKnown = true;
            return true;
        }
        int delta = targetSteps - elevatorStepPosition;
        if (delta == 0) { step.setState(false); return true; }
        boolean up = delta > 0;
        if (pulseActive && up != commandUp) return false;
        if (nowNs < pulseLowUntilNs) return false;
        if (targetSteps == 0 && elevatorHomed()) {
            step.setState(false);
            elevatorStepPosition = 0;
            elevatorPositionKnown = true;
            pulseActive = false;
            return true;
        }
        if (!pulseActive) {
            step.setState(false);
            dir.setState(up);
            commandUp = up;
            step.setState(true);
            pulseActive = true;
            pulseHighUntilNs = nowNs + STEP_PULSE_NS;
            return false;
        }
        if (nowNs < pulseHighUntilNs) return false;
        step.setState(false);
        pulseActive = false;
        pulseLowUntilNs = nowNs + STEP_LOW_NS;
        if (!up && elevatorHomed()) {
            elevatorStepPosition = 0;
            elevatorPositionKnown = true;
            return true;
        }
        elevatorStepPosition += up ? 1 : -1;
        elevatorPositionKnown = elevatorStepPosition >= 0 && elevatorStepPosition <= LiftingSequenceConfig.ElevatorTarget.LIFT2.steps;
        return elevatorStepPosition == targetSteps;
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
        if (!elevatorPositionKnown && !elevatorHomed()) return false;
        return stepElevatorToward(target.steps, System.nanoTime());
    }

    public void stopActuators() {
        leftfront.setPower(0);
        leftback.setPower(0);
        rightfront.setPower(0);
        rightback.setPower(0);
        step.setState(false);
        dir.setState(false);
        servoLeft.setPosition(LiftingSequenceConfig.PLACE_LEFT);
        servoRight.setPosition(LiftingSequenceConfig.PLACE_RIGHT);
        pulseActive = false;
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
