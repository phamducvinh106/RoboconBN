package org.firstinspires.ftc.teamcode.core;

import android.content.Context;

/** Camera transport backed by Pi USB CDC JSON packets. */
public final class PiCdcCameraTransport implements HardwareContracts.CameraTransport {
    private final PiBlockReceiver receiver;
    private final long maxAgeMs;

    public PiCdcCameraTransport(Context context, long maxAgeMs) {
        if (context == null || maxAgeMs < 0) throw new IllegalArgumentException("invalid CDC transport");
        receiver = new PiBlockReceiver(context);
        this.maxAgeMs = maxAgeMs;
    }

    public boolean start() { return receiver.start(); }

    @Override
    public CameraFrameContract read(CameraChannel channel) {
        PiBlockReceiver.BlockDetection detection = receiver.getLatest(channel == CameraChannel.WEBCAM1 ? "left" : "right");
        if (detection == null || System.currentTimeMillis() - detection.timestampMs > maxAgeMs) {
            return CameraFrameContract.invalid(channel, System.nanoTime());
        }
        return new CameraFrameContract(
                channel,
                detection.timestampMs * 1_000_000L,
                detection.found,
                detection.x,
                (int) Math.round(detection.x * 640.0),
                (int) Math.round(detection.y * 480.0),
                detection.blockType,
                detection.className,
                detection.found,
                0,
                0
        );
    }

    public void close() { receiver.close(); }
}
