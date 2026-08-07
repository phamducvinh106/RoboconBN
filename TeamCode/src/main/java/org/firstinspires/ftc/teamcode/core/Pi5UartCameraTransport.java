package org.firstinspires.ftc.teamcode.core;

/** D-03/D-04/D-05/D-06: UART/parser and vision remain deferred. */
public final class Pi5UartCameraTransport implements HardwareContracts.CameraTransport {
    private final HardwareContracts.Clock clock;
    public Pi5UartCameraTransport(HardwareContracts.Clock clock) { if(clock==null) throw new IllegalArgumentException("clock"); this.clock=clock; }
    public CameraFrameContract read(CameraChannel channel) { return CameraFrameContract.invalid(channel, clock.nowNs()); }
}
