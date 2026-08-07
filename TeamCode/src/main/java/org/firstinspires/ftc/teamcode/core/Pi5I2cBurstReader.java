package org.firstinspires.ftc.teamcode.core;

public interface Pi5I2cBurstReader {
    byte[] readBurst(int register, int length) throws InterruptedException;
}
