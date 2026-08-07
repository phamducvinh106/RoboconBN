package org.firstinspires.ftc.teamcode.core;

public final class HardwareContracts {
    private HardwareContracts() {}
    public interface Clock { long nowNs(); }
    public interface BinaryChannel { boolean high(); void setHigh(boolean high); }
    public interface ServoChannel { void setPosition(double position); }
    public interface BooleanSensor { boolean active(); }
    public interface PoseSource { PoseReading read(); }
    public interface CameraTransport { CameraFrameContract read(CameraChannel channel); }
    public static final class PoseReading {
        public final double xCm, yCm, headingDeg; public final long timestampNs; public final boolean valid;
        public PoseReading(double xCm, double yCm, double headingDeg, long timestampNs) {
            this.xCm=xCm; this.yCm=yCm; this.headingDeg=headingDeg; this.timestampNs=timestampNs;
            valid=Double.isFinite(xCm)&&Double.isFinite(yCm)&&Double.isFinite(headingDeg)&&Math.abs(headingDeg)<=360;
        }
    }
}
