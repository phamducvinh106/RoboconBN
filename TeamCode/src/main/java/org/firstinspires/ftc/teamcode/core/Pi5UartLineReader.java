package org.firstinspires.ftc.teamcode.core;

public interface Pi5UartLineReader {
    /** Drain newly received bytes from the transport (non-blocking). */
    byte[] pollBytes();
}
