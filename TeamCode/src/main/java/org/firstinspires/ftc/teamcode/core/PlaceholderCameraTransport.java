package org.firstinspires.ftc.teamcode.core;

/** Fail-closed camera transport until Hub ORB pipeline is wired into lifting. */
public final class PlaceholderCameraTransport implements HardwareContracts.CameraTransport {
    @Override
    public CameraFrameContract read(CameraChannel channel) {
        return CameraFrameContract.invalid(channel, System.nanoTime());
    }
}
