package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public final class RobotHardware {
    private final LiftingSequenceConfig config;
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
    public final IMU imu;
    public final Localizer localizer;

    public RobotHardware(HardwareMap hardwareMap) {
        this(hardwareMap, null);
    }

    public RobotHardware(HardwareMap hardwareMap, LiftingSequenceConfig config) {
        this.config = config;
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
        imu = hardwareMap.get(IMU.class, "imu");
        stopActuators();
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

    /** Blocks until target is reached or homing completes. */
    public boolean stepElevatorToward(int targetSteps, long ignoredNowNs) {
        int maxSteps = config == null ? 5625 : config.lift2Steps;
        if (targetSteps < 0 || targetSteps > maxSteps) {
            throw new IllegalArgumentException("elevator target out of bounds");
        }
        if (targetSteps == 0 && elevatorHomed()) {
            step.setState(false);
            elevatorStepPosition = 0;
            elevatorPositionKnown = true;
            return true;
        }
        int delta = targetSteps - elevatorStepPosition;
        if (delta == 0) {
            step.setState(false);
            return true;
        }
        boolean up = delta > 0;
        int remaining = Math.abs(delta);
        long highNs = stepHighNs();
        long lowNs = stepLowNs();
        step.setState(false);
        dir.setState(up);
        for (int i = 0; i < remaining; i++) {
            if (!up && elevatorHomed()) {
                step.setState(false);
                elevatorStepPosition = 0;
                elevatorPositionKnown = true;
                return targetSteps == 0;
            }
            step.setState(true);
            busyWaitNs(highNs);
            step.setState(false);
            busyWaitNs(lowNs);
            if (!up && elevatorHomed()) {
                elevatorStepPosition = 0;
                elevatorPositionKnown = true;
                return targetSteps == 0;
            }
            elevatorStepPosition += up ? 1 : -1;
        }
        elevatorPositionKnown = elevatorStepPosition >= 0 && elevatorStepPosition <= maxSteps;
        return elevatorStepPosition == targetSteps;
    }

    public void stopActuators() {
        leftfront.setPower(0);
        leftback.setPower(0);
        rightfront.setPower(0);
        rightback.setPower(0);
        step.setState(false);
        dir.setState(false);
        servoLeft.setPosition(config == null ? 0.25 : config.placeLeft);
        servoRight.setPosition(config == null ? 0.75 : config.placeRight);
    }

    private long stepHighNs() { return config == null ? 1L : config.stepHighNs; }
    private long stepLowNs() { return config == null ? 1L : config.stepLowNs; }

    private static void busyWaitNs(long durationNs) {
        if (durationNs <= 1L) return;
        long deadline = System.nanoTime() + durationNs;
        while (System.nanoTime() < deadline) { }
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
