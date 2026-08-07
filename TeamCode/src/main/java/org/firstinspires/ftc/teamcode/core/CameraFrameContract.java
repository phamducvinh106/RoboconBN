package org.firstinspires.ftc.teamcode.core;

public final class CameraFrameContract {
    public final CameraChannel channel; public final long timestampNs; public final boolean valid; public final double dxPx;
    public CameraFrameContract(CameraChannel channel, long timestampNs, boolean valid, double dxPx) {
        if (channel==null) throw new IllegalArgumentException("channel");
        this.channel=channel; this.timestampNs=timestampNs; this.valid=valid&&Double.isFinite(dxPx); this.dxPx=dxPx;
    }
    public boolean fresh(long nowNs, long maxAgeNs) { return valid&&maxAgeNs>=0&&timestampNs>=0&&nowNs>=timestampNs&&nowNs-timestampNs<=maxAgeNs; }
    public boolean authorizesMovement(long nowNs,long maxAgeNs) { return fresh(nowNs,maxAgeNs); }
    public static CameraFrameContract invalid(CameraChannel channel,long nowNs) { return new CameraFrameContract(channel,nowNs,false,Double.NaN); }
}
