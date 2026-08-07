package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Mecanum drive — robot-centric mixing, field-centric manual drive, PID goToPosition.
 *
 * Conventions: field +X right, +Y forward; heading decreases counter-clockwise.
 * Robot frame: +forward, +strafe left, +rotate CCW.
 *
 * Call {@code localizer.update()} before {@link #update()}.
 */
public final class MecanumDrive {

    public static final double DEFAULT_POS_KP = 0.031;
    public static final double DEFAULT_POS_KI = 0.012;
    public static final double DEFAULT_POS_KD = 0.05;
    public static final double DEFAULT_HEAD_KP = 0.031;
    public static final double DEFAULT_HEAD_KI = 0.006;
    public static final double DEFAULT_HEAD_KD = 0.05;

    public static final double DEFAULT_TOLERANCE_CM = 1.5;
    public static final double DEFAULT_TOLERANCE_DEG = 2.5;
    public static final double MAX_TRANSLATIONAL_POWER = 0.75;
    public static final double MAX_ROTATIONAL_POWER = 0.45;
    public static final double DEFAULT_SLEW_PER_LOOP = 0.08;

    public enum DriveState { IDLE, MOVING, HOLDING }

    public interface OdometryProvider {
        double getX();
        double getY();
        double getHeadingDeg();
    }

    private final DcMotorEx fl, fr, bl, br;
    private final OdometryProvider odometry;
    private final PidController xPid, yPid, hPid;

    private DriveState state = DriveState.IDLE;
    private double targetXCm, targetYCm, targetHeadingDeg;
    private double toleranceCm = DEFAULT_TOLERANCE_CM;
    private double toleranceDeg = DEFAULT_TOLERANCE_DEG;
    private double positionPowerLimit = MAX_TRANSLATIONAL_POWER;
    private double headingPowerLimit = MAX_ROTATIONAL_POWER;
    private double slewPerLoop = DEFAULT_SLEW_PER_LOOP;

    private double cmdForward, cmdStrafe, cmdRotate;
    private double lastFlPower, lastFrPower, lastBlPower, lastBrPower;
    private double lastFieldErrorX, lastFieldErrorY, lastHeadingErrorDeg;

    public MecanumDrive(HardwareMap hwMap, String flName, String frName,
                        String blName, String brName, Localizer odometry) {
        this(hwMap, flName, frName, blName, brName, wrap(odometry));
    }

    public MecanumDrive(HardwareMap hwMap, String flName, String frName,
                        String blName, String brName, OdometryProvider odometry) {
        fl = hwMap.get(DcMotorEx.class, flName);
        fr = hwMap.get(DcMotorEx.class, frName);
        bl = hwMap.get(DcMotorEx.class, blName);
        br = hwMap.get(DcMotorEx.class, brName);
        fl.setDirection(DcMotorSimple.Direction.FORWARD);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        this.odometry = odometry;
        xPid = newPid(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD, MAX_TRANSLATIONAL_POWER);
        yPid = newPid(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD, MAX_TRANSLATIONAL_POWER);
        hPid = newPid(DEFAULT_HEAD_KP, DEFAULT_HEAD_KI, DEFAULT_HEAD_KD, MAX_ROTATIONAL_POWER);
    }

    /** No motors — offline simulation only. */
    public static MecanumDrive forTest(OdometryProvider odometry) {
        return new MecanumDrive(odometry);
    }

    private MecanumDrive(OdometryProvider odometry) {
        fl = fr = bl = br = null;
        this.odometry = odometry;
        xPid = newPid(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD, MAX_TRANSLATIONAL_POWER);
        yPid = newPid(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD, MAX_TRANSLATIONAL_POWER);
        hPid = newPid(DEFAULT_HEAD_KP, DEFAULT_HEAD_KI, DEFAULT_HEAD_KD, MAX_ROTATIONAL_POWER);
    }

    public void update() {
        switch (state) {
            case MOVING:
            case HOLDING:
                updatePositionControl();
                break;
            default:
                stopMotors();
                break;
        }
    }

    public void goToPosition(double xCm, double yCm, double headingDeg) {
        requireFinite(xCm, "xCm");
        requireFinite(yCm, "yCm");
        requireFinite(headingDeg, "headingDeg");
        targetXCm = xCm;
        targetYCm = yCm;
        targetHeadingDeg = headingDeg;
        xPid.reset();
        yPid.reset();
        hPid.reset();
        cmdForward = cmdStrafe = cmdRotate = 0.0;
        state = DriveState.MOVING;
    }

    public void holdPosition() {
        goToPosition(odometry.getX(), odometry.getY(), odometry.getHeadingDeg());
    }

    public void driveRobotCentric(double forward, double strafe, double rotate) {
        state = DriveState.IDLE;
        cmdForward = forward;
        cmdStrafe = strafe;
        cmdRotate = rotate;
        applyRobotFrame(forward, strafe, rotate);
    }

    public void driveFieldCentric(double fieldForward, double fieldStrafe, double rotate) {
        state = DriveState.IDLE;
        double[] robot = fieldToRobot(fieldForward, fieldStrafe, odometry.getHeadingDeg());
        cmdForward = robot[0];
        cmdStrafe = robot[1];
        cmdRotate = rotate;
        applyRobotFrame(robot[0], robot[1], rotate);
    }

    public void setRawPowers(double flPower, double frPower, double blPower, double brPower) {
        state = DriveState.IDLE;
        cmdForward = cmdStrafe = cmdRotate = 0.0;
        setMotorPowers(flPower, frPower, blPower, brPower);
    }

    public void stop() {
        state = DriveState.IDLE;
        cmdForward = cmdStrafe = cmdRotate = 0.0;
        stopMotors();
    }

    public DriveState getState() { return state; }
    public boolean atTarget() { return state == DriveState.HOLDING; }

    public boolean atTarget(double tolCm, double tolDeg) {
        if (state != DriveState.MOVING && state != DriveState.HOLDING) return false;
        return Math.abs(odometry.getX() - targetXCm) <= tolCm
                && Math.abs(odometry.getY() - targetYCm) <= tolCm
                && Math.abs(wrapHeadingError(targetHeadingDeg - odometry.getHeadingDeg())) <= tolDeg;
    }

    public double getRemainingError() {
        double dx = odometry.getX() - targetXCm;
        double dy = odometry.getY() - targetYCm;
        return Math.hypot(dx, dy);
    }

    public void setTolerance(double toleranceCm, double toleranceDeg) {
        requirePositiveFinite(toleranceCm, "toleranceCm");
        requirePositiveFinite(toleranceDeg, "toleranceDeg");
        this.toleranceCm = toleranceCm;
        this.toleranceDeg = toleranceDeg;
    }

    public void setPowerLimits(double positionLimit, double headingLimit) {
        positionPowerLimit = clamp01(positionLimit);
        headingPowerLimit = clamp01(headingLimit);
        xPid.setOutputLimits(-positionPowerLimit, positionPowerLimit);
        yPid.setOutputLimits(-positionPowerLimit, positionPowerLimit);
        hPid.setOutputLimits(-headingPowerLimit, headingPowerLimit);
    }

    public void setSlewPerLoop(double slew) {
        if (!Double.isFinite(slew) || slew <= 0) throw new IllegalArgumentException("slew");
        slewPerLoop = slew;
    }

    public void setPositionGains(double kp, double ki, double kd) {
        xPid.setGains(kp, ki, kd);
        yPid.setGains(kp, ki, kd);
    }

    public void setHeadingGains(double kp, double ki, double kd) {
        hPid.setGains(kp, ki, kd);
    }

    public double getLastFlPower() { return lastFlPower; }
    public double getLastFrPower() { return lastFrPower; }
    public double getLastBlPower() { return lastBlPower; }
    public double getLastBrPower() { return lastBrPower; }
    public double getLastRobotForward() { return cmdForward; }
    public double getLastRobotStrafe() { return cmdStrafe; }
    public double getLastRobotRotate() { return cmdRotate; }
    public double getLastFieldErrorX() { return lastFieldErrorX; }
    public double getLastFieldErrorY() { return lastFieldErrorY; }
    public double getLastHeadingErrorDeg() { return lastHeadingErrorDeg; }

    // --- pure math (offline checks) ---

    public static double[] mixMecanum(double forward, double strafe, double rotate) {
        return normalizePowers(
                forward + strafe + rotate,
                forward - strafe - rotate,
                forward - strafe + rotate,
                forward + strafe - rotate);
    }

    /** field +Y forward, +X right → robot forward/strafe-left. */
    public static double[] fieldToRobot(double fieldForward, double fieldStrafe, double headingDeg) {
        double rad = Math.toRadians(headingDeg);
        double cos = Math.cos(rad);
        double sin = Math.sin(rad);
        return new double[]{
                fieldForward * cos - fieldStrafe * sin,
                fieldForward * sin + fieldStrafe * cos
        };
    }

    public static double wrapHeadingError(double errorDeg) {
        double e = errorDeg % 360.0;
        if (e > 180.0) e -= 360.0;
        if (e < -180.0) e += 360.0;
        return e;
    }

    public static double[] normalizePowers(double fl, double fr, double bl, double br) {
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }
        return new double[]{fl, fr, bl, br};
    }

    private void updatePositionControl() {
        double currentX = odometry.getX();
        double currentY = odometry.getY();
        double currentHeading = odometry.getHeadingDeg();

        lastFieldErrorX = targetXCm - currentX;
        lastFieldErrorY = targetYCm - currentY;
        lastHeadingErrorDeg = wrapHeadingError(targetHeadingDeg - currentHeading);

        double fieldStrafePower = xPid.calculate(targetXCm, currentX);
        double fieldForwardPower = yPid.calculate(targetYCm, currentY);
        double[] robot = fieldToRobot(fieldForwardPower, fieldStrafePower, currentHeading);

        double rotatePower = -hPid.calculate(
                currentHeading + lastHeadingErrorDeg,
                currentHeading);
        if (Math.abs(lastHeadingErrorDeg) <= toleranceDeg) {
            rotatePower = 0.0;
            hPid.reset();
        }
        if (state == DriveState.HOLDING) {
            rotatePower = 0.0;
        }
        rotatePower = Math.max(-headingPowerLimit, Math.min(headingPowerLimit, rotatePower));

        // Wrapped heading error controls shortest turn; PID measurement stays continuous.
        // This avoids a false derivative spike when heading crosses +/-180 degrees.
        

        cmdForward = slew(cmdForward, robot[0], slewPerLoop);
        cmdStrafe = slew(cmdStrafe, robot[1], slewPerLoop);
        cmdRotate = slew(cmdRotate, rotatePower, slewPerLoop);

        applyRobotFrame(cmdForward, cmdStrafe, cmdRotate);

        boolean atPose = xPid.atSetpoint(toleranceCm)
                && yPid.atSetpoint(toleranceCm)
                && Math.abs(lastHeadingErrorDeg) <= toleranceDeg;

        if (atPose) {
            state = DriveState.HOLDING;
        } else if (state == DriveState.HOLDING) {
            state = DriveState.MOVING;
        }
    }

    private void applyRobotFrame(double forward, double strafe, double rotate) {
        double[] p = mixMecanum(forward, strafe, rotate);
        setMotorPowers(p[0], p[1], p[2], p[3]);
    }

    private void setMotorPowers(double flPower, double frPower, double blPower, double brPower) {
        lastFlPower = flPower;
        lastFrPower = frPower;
        lastBlPower = blPower;
        lastBrPower = brPower;
        if (fl != null) {
            fl.setPower(flPower);
            fr.setPower(frPower);
            bl.setPower(blPower);
            br.setPower(brPower);
        }
    }

    private void stopMotors() {
        setMotorPowers(0, 0, 0, 0);
    }

    private static double slew(double current, double target, double maxDelta) {
        double d = target - current;
        if (d > maxDelta) return current + maxDelta;
        if (d < -maxDelta) return current - maxDelta;
        return target;
    }

    private static PidController newPid(double kp, double ki, double kd, double limit) {
        PidController pid = new PidController(kp, ki, kd);
        pid.setOutputLimits(-limit, limit);
        return pid;
    }

    private static OdometryProvider wrap(Localizer localizer) {
        return new OdometryProvider() {
            @Override public double getX() { return localizer.getX(); }
            @Override public double getY() { return localizer.getY(); }
            @Override public double getHeadingDeg() { return localizer.getHeadingDeg(); }
        };
    }

    private static void requireFinite(double v, String name) {
        if (!Double.isFinite(v)) throw new IllegalArgumentException(name);
    }

    private static void requirePositiveFinite(double v, String name) {
        if (!Double.isFinite(v) || v <= 0) throw new IllegalArgumentException(name);
    }

    private static double clamp01(double v) {
        return Math.max(0.0, Math.min(1.0, v));
    }
}
