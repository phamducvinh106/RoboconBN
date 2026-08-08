package org.firstinspires.ftc.teamcode.core;

public final class CameraFrameContract {
    public final CameraChannel channel;
    public final long timestampNs;
    public final boolean valid;
    public final double dxPx;
    public final int centerX;
    public final int centerY;
    public final int blockCode;
    public final String blockType;
    public final boolean channelFound;
    public final int rawPayload;
    public final int heartbeat;

    public CameraFrameContract(
            CameraChannel channel,
            long timestampNs,
            boolean valid,
            double dxPx,
            int centerX,
            int centerY,
            int blockCode,
            String blockType,
            boolean channelFound,
            int rawPayload,
            int heartbeat) {
        if (channel == null) throw new IllegalArgumentException("channel");
        this.channel = channel;
        this.timestampNs = timestampNs;
        this.channelFound = channelFound;
        this.centerX = centerX;
        this.centerY = centerY;
        this.blockCode = blockCode;
        this.blockType = blockType;
        this.rawPayload = rawPayload;
        this.heartbeat = heartbeat;
        boolean dxOk = channel == CameraChannel.WEBCAM2 || Double.isFinite(dxPx);
        this.valid = valid
                && channelFound
                && blockType != null
                && (blockCode < 0 || PiCdcPacket.isValidBlockCode(blockCode))
                && dxOk;
        this.dxPx = this.valid ? dxPx : Double.NaN;
    }

    public boolean fresh(long nowNs, long maxAgeNs) {
        return valid && maxAgeNs >= 0 && timestampNs >= 0 && nowNs >= timestampNs && nowNs - timestampNs <= maxAgeNs;
    }

    public boolean authorizesMovement(long nowNs, long maxAgeNs) {
        return fresh(nowNs, maxAgeNs);
    }

    public static CameraFrameContract invalid(CameraChannel channel, long nowNs) {
        return new CameraFrameContract(channel, nowNs, false, Double.NaN, -1, -1, -1, null, false, 0, -1);
    }
}
