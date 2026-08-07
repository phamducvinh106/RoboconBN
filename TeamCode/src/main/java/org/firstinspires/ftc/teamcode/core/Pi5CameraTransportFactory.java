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
        Pi5UartLineReader reader = FtcPi5UartLineReaderFactory.fromHardwareMap(hardwareMap, config);
        return new Pi5UartCameraTransport(
                clock,
                reader,
                config.sensorStaleNs,
                config.cameraFrameWidth,
                config.blockTypesByCode
        );
    }
}
