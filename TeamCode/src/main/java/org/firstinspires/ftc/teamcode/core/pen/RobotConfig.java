package org.firstinspires.ftc.teamcode.core.pen;

/** Cấu hình robot vẽ — pen servo + thông số chuyển động. */
public final class RobotConfig {

    public static final int SCHEMA_VERSION = 1;

    public final PenConfig pen;
    public final MotionConfig motion;

    public RobotConfig(PenConfig pen, MotionConfig motion) {
        if (pen == null || motion == null) throw new IllegalArgumentException("missing config section");
        this.pen = pen;
        this.motion = motion;
    }

    public static RobotConfig defaults() {
        return new RobotConfig(
                new PenConfig("penServo", 0.05, 0.45),
                new MotionConfig(0.20, 3.0, 1.0, 0.35, 2));
    }

    public static final class PenConfig {
        public final String deviceName;
        public final double upPosition;
        public final double downPosition;

        public PenConfig(String deviceName, double upPosition, double downPosition) {
            if (deviceName == null || deviceName.isEmpty()) {
                throw new IllegalArgumentException("deviceName");
            }
            this.deviceName = deviceName;
            this.upPosition = upPosition;
            this.downPosition = downPosition;
        }
    }

    public static final class MotionConfig {
        public final double drivePower;
        /** Ngưỡng vị trí trong JSON (mm); chuyển sang cm khi áp dụng cho MecanumDrive. */
        public final double positionToleranceMm;
        public final double headingToleranceDeg;
        public final double headingHoldPower;
        public final int settleCycles;

        public MotionConfig(double drivePower, double positionToleranceMm,
                            double headingToleranceDeg, double headingHoldPower,
                            int settleCycles) {
            this.drivePower = drivePower;
            this.positionToleranceMm = positionToleranceMm;
            this.headingToleranceDeg = headingToleranceDeg;
            this.headingHoldPower = headingHoldPower;
            this.settleCycles = settleCycles;
        }

        public double positionToleranceCm() {
            return positionToleranceMm / 10.0;
        }
    }
}
