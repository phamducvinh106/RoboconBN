package org.firstinspires.ftc.teamcode.core;

import android.content.Context;

/** Camera transport backed by validated Pi USB CDC JSON packets. */
public final class PiCdcCameraTransport implements HardwareContracts.CameraTransport {
    private final PiBlockReceiver receiver;
    private final long maxAgeNs;
    private final int frameWidth;
    private final int frameHeight;

    public PiCdcCameraTransport(Context context, long maxAgeNs, int frameWidth, int frameHeight) {
        if (context == null || maxAgeNs < 0 || frameWidth <= 0 || frameHeight <= 0) {
            throw new IllegalArgumentException("invalid CDC transport");
        }
        receiver = new PiBlockReceiver(context);
        this.maxAgeNs = maxAgeNs;
        this.frameWidth = frameWidth;
        this.frameHeight = frameHeight;
    }

    public boolean start() { return receiver.start(); }

    public PiBlockReceiver receiver() { return receiver; }

    @Override
    public CameraFrameContract read(CameraChannel channel) {
        long nowNs = System.nanoTime();
        PiCdcPacket.Frame frame = receiver.getLatestFrame();
        if (frame == null || !frame.frameValid || !frame.valid) {
            return CameraFrameContract.invalid(channel, nowNs);
        }
        if (nowNs < frame.receivedNs || nowNs - frame.receivedNs > maxAgeNs) {
            return CameraFrameContract.invalid(channel, nowNs);
        }
        PiCdcPacket.ChannelDetection detection = channel == CameraChannel.WEBCAM1 ? frame.left : frame.right;
        if (detection == null || !detection.found) {
            return CameraFrameContract.invalid(channel, nowNs);
        }
        double dxPx = PiCdcPacket.dxPx(detection.x, frameWidth);
        return new CameraFrameContract(
                channel,
                frame.receivedNs,
                true,
                dxPx,
                PiCdcPacket.centerPx(detection.x, frameWidth),
                PiCdcPacket.centerPx(detection.y, frameHeight),
                detection.blockType,
                detection.className,
                true,
                0,
                frame.heartbeat
        );
    }

    @Override
    public void close() { receiver.close(); }
}
