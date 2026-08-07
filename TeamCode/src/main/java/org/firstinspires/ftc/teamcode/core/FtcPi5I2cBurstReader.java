package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

public final class FtcPi5I2cBurstReader implements Pi5I2cBurstReader {
    private final I2cDeviceSynch device;

    public FtcPi5I2cBurstReader(I2cDeviceSynch device) {
        if (device == null) throw new IllegalArgumentException("device");
        this.device = device;
        device.engage();
    }

    public static FtcPi5I2cBurstReader fromHardwareMap(HardwareMap hardwareMap, int address7bit) {
        I2cDeviceSynch device = hardwareMap.get(I2cDeviceSynch.class, "pi5Camera");
        device.setI2cAddress(I2cAddr.create7bit(address7bit));
        return new FtcPi5I2cBurstReader(device);
    }

    @Override
    public byte[] readBurst(int register, int length) throws InterruptedException {
        return device.read(register, length);
    }
}
