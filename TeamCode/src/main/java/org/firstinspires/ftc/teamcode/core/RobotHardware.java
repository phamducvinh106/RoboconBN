package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public final class RobotHardware {
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
