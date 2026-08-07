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
        PiCdcCameraTransport transport = new PiCdcCameraTransport(
                hardwareMap.appContext,
                config.sensorStaleNs / 1_000_000L
        );
        if (!transport.start()) {
            throw new IllegalStateException("Pi USB CDC ACM device unavailable");
        }
        return transport;
    }

    public static Pi5UartCameraTransport createWithHubPolling(
            HardwareMap hardwareMap,
            LiftingSequenceConfig config,
            HardwareContracts.Clock clock,
            Runnable idleCallback) {
        Pi5UartCameraTransport transport = (Pi5UartCameraTransport) create(hardwareMap, config, clock);
        transport.enableHubPolling(idleCallback, FtcPi5SoftUartLineReader.recommendedHubSamples(config.pi5UartBaud));
        return transport;
    }
}
