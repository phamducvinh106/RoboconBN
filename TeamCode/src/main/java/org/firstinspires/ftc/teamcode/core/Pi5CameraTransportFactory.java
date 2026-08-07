package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.HardwareMap;

public final class Pi5CameraTransportFactory {
    private Pi5CameraTransportFactory() {}

    public static HardwareContracts.CameraTransport create(
            HardwareMap hardwareMap,
            LiftingSequenceConfig config,
            HardwareContracts.Clock clock) {
        if (hardwareMap == null || config == null || clock == null) {
            throw new IllegalArgumentException("missing pi5 camera dependencies");
        }
        FtcPi5I2cBurstReader reader = FtcPi5I2cBurstReader.fromHardwareMap(hardwareMap, config.pi5I2cAddress);
        return new Pi5I2cCameraTransport(
                clock,
                reader,
                config.sensorStaleNs,
                config.cameraFrameWidth,
                config.blockTypesByCode
        );
    }
}
