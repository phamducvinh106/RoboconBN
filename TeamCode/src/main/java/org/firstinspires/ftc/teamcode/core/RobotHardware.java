package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

public final class RobotHardware {
    private static final int HOMING_POLL_STRIDE = 32;
    private static final int ELEVATOR_STEPS_PER_TICK = 64;

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
        disableLynxBulkCache(hardwareMap);
        leftfront = hardwareMap.get(DcMotorEx.class, "leftfront");
        leftback = hardwareMap.get(DcMotorEx.class, "leftback");
        rightfront = hardwareMap.get(DcMotorEx.class, "rightfront");
        rightback = hardwareMap.get(DcMotorEx.class, "rightback");
        servoLeft = hardwareMap.get(Servo.class, "servoLeft");
        servoRight = hardwareMap.get(Servo.class, "servoRight");
        servoLeft.setDirection(Servo.Direction.REVERSE);
        servoRight.setDirection(Servo.Direction.FORWARD);
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
        if (targetSteps == 0) {
            if (!elevatorHomed()) {
                pulseDownHoming(Math.min(maxSteps, ELEVATOR_STEPS_PER_TICK));
            }
            if (elevatorHomed()) {
                finishStepLow();
                elevatorStepPosition = 0;
                elevatorPositionKnown = true;
                return true;
            }
            elevatorPositionKnown = false;
            return false;
        }
        if (!elevatorPositionKnown) {
            return false;
        }
        int delta = targetSteps - elevatorStepPosition;
        if (delta == 0) {
            finishStepLow();
            return true;
        }
        if (delta > 0) {
            pulseUp(Math.min(delta, ELEVATOR_STEPS_PER_TICK));
        } else {
            pulseDownHoming(Math.min(-delta, ELEVATOR_STEPS_PER_TICK));
        }
        elevatorPositionKnown = elevatorStepPosition >= 0 && elevatorStepPosition <= maxSteps;
        return elevatorStepPosition == targetSteps;
    }

    public void stopActuators() {
        leftfront.setPower(0);
        leftback.setPower(0);
        rightfront.setPower(0);
        rightback.setPower(0);
        finishStepLow();
        dir.setState(false);
        servoLeft.setPosition(config == null ? 0.75 : config.placeLeft);
        servoRight.setPosition(config == null ? 0.75 : config.placeRight);
    }

    private void setElevatorDir(boolean up) {
        boolean high = config != null && config.stepperDirInverted ? !up : up;
        dir.setState(high);
    }

    private void pulseUp(int count) {
        setElevatorDir(true);
        finishStepLow();
        for (int i = 0; i < count; i++) {
            step.setState(true);
            step.setState(false);
            elevatorStepPosition++;
        }
    }

    private void pulseDownHoming(int maxSteps) {
        setElevatorDir(false);
        finishStepLow();
        int moved = 0;
        while (!elevatorHomed() && moved < maxSteps) {
            int burst = Math.min(HOMING_POLL_STRIDE, maxSteps - moved);
            for (int i = 0; i < burst; i++) {
                step.setState(true);
                step.setState(false);
                moved++;
            }
        }
        finishStepLow();
        if (elevatorHomed()) {
            elevatorStepPosition = 0;
            elevatorPositionKnown = true;
        } else {
            elevatorStepPosition = Math.max(0, elevatorStepPosition - moved);
        }
    }

    private void finishStepLow() {
        step.setState(false);
    }

    private static void disableLynxBulkCache(HardwareMap hardwareMap) {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
        }
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
