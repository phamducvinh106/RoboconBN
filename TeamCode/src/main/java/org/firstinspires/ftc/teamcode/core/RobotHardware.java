package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.odometry.Localizer;
import org.firstinspires.ftc.teamcode.core.pen.RobotConfig;

import java.util.List;

/** Map phần cứng robot vẽ: mecanum, odometry, IMU, servo bút. */
public final class RobotHardware {

    public final DcMotorEx leftfront;
    public final DcMotorEx leftback;
    public final DcMotorEx rightfront;
    public final DcMotorEx rightback;
    public final Servo penServo;
    public final IMU imu;
    public final Localizer localizer;
    public final RobotConfig config;

    public RobotHardware(HardwareMap hardwareMap) {
        this(hardwareMap, RobotConfig.defaults());
    }

    public RobotHardware(HardwareMap hardwareMap, RobotConfig config) {
        if (config == null) throw new IllegalArgumentException("missing config");
        this.config = config;
        disableLynxBulkCache(hardwareMap);

        leftfront = hardwareMap.get(DcMotorEx.class, "leftfront");
        leftback = hardwareMap.get(DcMotorEx.class, "leftback");
        rightfront = hardwareMap.get(DcMotorEx.class, "rightfront");
        rightback = hardwareMap.get(DcMotorEx.class, "rightback");
        penServo = hardwareMap.get(Servo.class, config.pen.deviceName);
        imu = hardwareMap.get(IMU.class, "imu");

        stopMotors();

        localizer = new Localizer(
                leftfront,
                rightfront,
                imu,
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        );
    }

    public void stopMotors() {
        leftfront.setPower(0);
        leftback.setPower(0);
        rightfront.setPower(0);
        rightback.setPower(0);
    }

    private static void disableLynxBulkCache(HardwareMap hardwareMap) {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule hub : hubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.OFF);
        }
    }
}
