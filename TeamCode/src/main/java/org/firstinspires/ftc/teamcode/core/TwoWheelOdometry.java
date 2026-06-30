package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public final class TwoWheelOdometry {

    // ============================================================
    //  >>>  TUNE  <<<  Các thông số cần chỉnh cho từng robot
    // ============================================================

    // --- Đầu đo encoder ---
    private static final double WHEEL_DIAMETER_CM    = 4.8;      // [m] đường kính bánh xe odometry
    private static final double TICKS_PER_REV        = 2000.0;   // [m] số tick trên 1 vòng quay
    private static final double PARALLEL_ENCODER_SIGN     = -1.0; // [m] chiều encoder song song (+1 / -1)
    private static final double PERPENDICULAR_ENCODER_SIGN = -1.0; // [m] chiều encoder vuông góc (+1 / -1)
    private static final double HEADING_SIGN               =  1.0; // [m] chiều IMU heading (+1 / -1)

    // --- Vị trí đặt dead-wheel so với tâm robot ---
    private static final double PARALLEL_Y_OFFSET_CM      =  5.0; // [cm] offset Y của encoder song song
    private static final double PERPENDICULAR_X_OFFSET_CM = -25.0;// [cm] offset X của encoder vuông góc

    // --- Ngưỡng lọc (deadband) ---
    private static final double DELTA_HEADING_EPSILON     = 1e-5; // [rad] bỏ qua nếu delta heading nhỏ hơn
    private static final int    DELTA_TICK_DEADBAND       = 2;    // [tick] bỏ qua nếu delta tick của cả 2 encoder < giá trị này
    private static final double ACCUM_HEADING_MIN_RAD     = 0.2;  // [rad] heading tích lũy tối thiểu để tính suggested offset

    // ============================================================
    //  Hết vùng TUNE — không sửa bên dưới nếu không cần thiết
    // ============================================================

    private static final double CM_PER_TICK =
            (Math.PI * WHEEL_DIAMETER_CM) / TICKS_PER_REV;

    private final DcMotorEx parallelPod;
    private final DcMotorEx perpendicularPod;
    private final IMU imu;

    private int lastParallelTicks;
    private int lastPerpendicularTicks;
    private double lastHeadingRad;

    private double xCm = 0.0;
    private double yCm = 0.0;

    private double lastDeltaHeadingRad = 0.0;
    private double lastParallelDeltaCm = 0.0;
    private double lastPerpendicularDeltaCm = 0.0;
    private double lastForwardLocalCm = 0.0;
    private double lastLeftLocalCm = 0.0;

    private double accumHeadingRad = 0.0;
    private double accumParallelCm = 0.0;
    private double accumPerpendicularCm = 0.0;

    public TwoWheelOdometry(
            HardwareMap hardwareMap,
            String parallelPodName,
            String perpendicularPodName,
            String imuName,
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection,
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection
    ) {
        parallelPod = hardwareMap.get(
                DcMotorEx.class,
                parallelPodName
        );

        perpendicularPod = hardwareMap.get(
                DcMotorEx.class,
                perpendicularPodName
        );

        imu = hardwareMap.get(IMU.class, imuName);

        imu.resetDeviceConfigurationForOpMode();

        RevHubOrientationOnRobot orientation =
                new RevHubOrientationOnRobot(
                        logoDirection,
                        usbDirection
                );

        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();

        initializePreviousValues();
    }

    public TwoWheelOdometry(
            DcMotorEx parallelPod,
            DcMotorEx perpendicularPod,
            IMU imu,
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection,
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection
    ) {
        this.parallelPod = parallelPod;
        this.perpendicularPod = perpendicularPod;
        this.imu = imu;

        imu.resetDeviceConfigurationForOpMode();

        RevHubOrientationOnRobot orientation =
                new RevHubOrientationOnRobot(
                        logoDirection,
                        usbDirection
                );

        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();

        initializePreviousValues();
    }

    private void initializePreviousValues() {
        lastParallelTicks =
                parallelPod.getCurrentPosition();

        lastPerpendicularTicks =
                perpendicularPod.getCurrentPosition();

        lastHeadingRad = getHeadingRad();
    }

    public void update() {
        int currentParallelTicks =
                parallelPod.getCurrentPosition();

        int currentPerpendicularTicks =
                perpendicularPod.getCurrentPosition();

        int deltaParallelTicks =
                currentParallelTicks - lastParallelTicks;

        int deltaPerpendicularTicks =
                currentPerpendicularTicks
                        - lastPerpendicularTicks;

        lastParallelTicks = currentParallelTicks;
        lastPerpendicularTicks = currentPerpendicularTicks;

        double currentHeadingRad = getHeadingRad();

        double deltaHeadingRad = angleWrap(
                currentHeadingRad - lastHeadingRad
        );

        double avgHeadingRad =
                lastHeadingRad + deltaHeadingRad / 2.0;

        lastHeadingRad = currentHeadingRad;

        if (Math.abs(deltaHeadingRad) < DELTA_HEADING_EPSILON
                && Math.abs(deltaParallelTicks) < DELTA_TICK_DEADBAND
                && Math.abs(deltaPerpendicularTicks) < DELTA_TICK_DEADBAND) {
            return;
        }

        double parallelDeltaCm =
                PARALLEL_ENCODER_SIGN
                        * deltaParallelTicks
                        * CM_PER_TICK;

        double perpendicularDeltaCm =
                PERPENDICULAR_ENCODER_SIGN
                        * deltaPerpendicularTicks
                        * CM_PER_TICK;

        double forwardLocalCm =
                parallelDeltaCm
                        + PARALLEL_Y_OFFSET_CM
                        * deltaHeadingRad;

        double leftLocalCm =
                perpendicularDeltaCm
                        - PERPENDICULAR_X_OFFSET_CM
                        * deltaHeadingRad;

        // Chuyển từ frame robot sang frame global.
        //
        // Quy ước frame robot:
        //   +forward = phía trước, +left = bên trái.
        // Quy ước frame global:
        //   +X = right (phải), +Y = forward (tới).
        //
        // Rotation matrix chuẩn R(θ) = [[cos, -sin], [sin, cos]]
        // chuyển local (forward, left) → global (X=right, Y=forward).
        //
        // Tuy nhiên các dấu dưới đây KHÔNG khớp rotation chuẩn vì
        // encoder signs (PARALLEL_ENCODER_SIGN, PERPENDICULAR_ENCODER_SIGN)
        // đã được calibrate thực nghiệm với camera. KHÔNG tự ý đổi dấu
        // nếu chưa re-calibrate toàn bộ.
        double cos = Math.cos(avgHeadingRad);
        double sin = Math.sin(avgHeadingRad);

        double xGlobalCm =
                -forwardLocalCm * sin
                        - leftLocalCm * cos;

        double yGlobalCm =
                forwardLocalCm * cos
                        - leftLocalCm * sin;

        xCm += xGlobalCm;
        yCm += yGlobalCm;

        lastDeltaHeadingRad = deltaHeadingRad;
        lastParallelDeltaCm = parallelDeltaCm;
        lastPerpendicularDeltaCm = perpendicularDeltaCm;
        lastForwardLocalCm = forwardLocalCm;
        lastLeftLocalCm = leftLocalCm;

        accumHeadingRad += deltaHeadingRad;
        accumParallelCm += parallelDeltaCm;
        accumPerpendicularCm += perpendicularDeltaCm;
    }

    public double getX() {
        return xCm;
    }

    public double getY() {
        return yCm;
    }

    public double getHeadingDeg() {
        return Math.toDegrees(lastHeadingRad);
    }

    public double getLastDeltaHeadingRad() {
        return lastDeltaHeadingRad;
    }

    public double getLastParallelDeltaCm() {
        return lastParallelDeltaCm;
    }

    public double getLastPerpendicularDeltaCm() {
        return lastPerpendicularDeltaCm;
    }

    public double getLastForwardLocalCm() {
        return lastForwardLocalCm;
    }

    public double getLastLeftLocalCm() {
        return lastLeftLocalCm;
    }

    public double getAccumHeadingRad() {
        return accumHeadingRad;
    }

    public double getAccumParallelCm() {
        return accumParallelCm;
    }

    public double getAccumPerpendicularCm() {
        return accumPerpendicularCm;
    }

    public double getSuggestedParallelYOffsetCm() {
        if (Math.abs(accumHeadingRad) < ACCUM_HEADING_MIN_RAD) {
            return Double.NaN;
        }

        return -accumParallelCm / accumHeadingRad / PARALLEL_ENCODER_SIGN;
    }

    public double getSuggestedPerpendicularXOffsetCm() {
        if (Math.abs(accumHeadingRad) < ACCUM_HEADING_MIN_RAD) {
            return Double.NaN;
        }

        return accumPerpendicularCm / accumHeadingRad / PERPENDICULAR_ENCODER_SIGN;
    }

    public void resetPose() {
        reset();
    }

    public void resetPoseAndHeading() {
        imu.resetYaw();
        reset();
    }

    public void reset() {
        xCm = 0.0;
        yCm = 0.0;

        lastParallelTicks =
                parallelPod.getCurrentPosition();

        lastPerpendicularTicks =
                perpendicularPod.getCurrentPosition();

        lastHeadingRad = getHeadingRad();

        lastDeltaHeadingRad = 0.0;
        lastParallelDeltaCm = 0.0;
        lastPerpendicularDeltaCm = 0.0;
        lastForwardLocalCm = 0.0;
        lastLeftLocalCm = 0.0;

        accumHeadingRad = 0.0;
        accumParallelCm = 0.0;
        accumPerpendicularCm = 0.0;
    }

    private double getHeadingRad() {
        double raw = imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.RADIANS);

        if (Double.isNaN(raw)) {
            return 0.0;
        }

        return HEADING_SIGN * raw;
    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }

        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }
}
