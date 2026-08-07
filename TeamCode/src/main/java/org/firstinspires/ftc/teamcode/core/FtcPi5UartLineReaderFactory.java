package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public final class FtcPi5UartLineReaderFactory {
    private FtcPi5UartLineReaderFactory() {}

    public static Pi5UartLineReader fromHardwareMap(HardwareMap hardwareMap, LiftingSequenceConfig config) {
        if (hardwareMap == null || config == null) {
            throw new IllegalArgumentException("missing uart dependencies");
        }
        DigitalChannel rx = hardwareMap.get(DigitalChannel.class, config.pi5UartDeviceName);
        return new FtcPi5SoftUartLineReader(rx, config.pi5UartBaud);
    }
}
