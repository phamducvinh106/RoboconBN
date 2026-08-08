package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.HardwareMap;

public final class Pi5CameraTransportFactory {
    private Pi5CameraTransportFactory() {}

    public static HardwareContracts.CameraTransport create(
            HardwareMap hardwareMap,
            LiftingSequenceConfig config,
            HardwareContracts.Clock clock) {
        return createTransport(hardwareMap, config, clock);
    }

    public static PiCdcCameraTransport createTransport(
            HardwareMap hardwareMap,
            LiftingSequenceConfig config,
            HardwareContracts.Clock clock) {
        if (hardwareMap == null || config == null || clock == null) {
            throw new IllegalArgumentException("missing pi5 camera dependencies");
        }
        PiCdcCameraTransport transport = new PiCdcCameraTransport(
                hardwareMap.appContext,
                config.sensorStaleNs,
                config.cameraFrameWidth,
                config.cameraFrameHeight
        );
        if (!transport.start()) {
            throw new IllegalStateException("Pi USB CDC ACM device unavailable: "
                    + transport.receiver().getLastError());
        }
        return transport;
    }
}
