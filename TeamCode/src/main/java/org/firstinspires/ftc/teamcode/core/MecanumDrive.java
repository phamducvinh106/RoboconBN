package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Mecanum 4-wheel drive.
 *
 * Tich hợp PID + TwoWheelOdometry để di chuyên chính xác đến toạ dô
 * (x, y, heading). Hồ trợ chay theo script JSON, có thê tích hợp
 * vào state machine bên ngoài thông qua getState().
 *
 * Cách dùng tối thiêu:
 * <pre>{@code
 *   TwoWheelOdometry odo = new TwoWheelOdometry(hwMap, "ypod", "xpod", "imu", ...);
 *   MecanumDrive drive = new MecanumDrive(hwMap, "fl", "fr", "bl", "br", odo);
 *
 *   while (opModeIsActive()) {
 *       odo.update();
 *       drive.update();
 *       if (drive.atTarget()) { ... }
 *   }
 * }</pre>
 */
public final class MecanumDrive {

    // ============================================================
    //  >>>  TUNE  <<<  PID gains & limits
    // ============================================================

    public static final double DEFAULT_POS_KP = 0.05;
    public static final double DEFAULT_POS_KI = 0.012;
    public static final double DEFAULT_POS_KD = 0.025;

    public static final double DEFAULT_HEAD_KP = 0.04;
    public static final double DEFAULT_HEAD_KI = 0.006;
    public static final double DEFAULT_HEAD_KD = 0.012;

    public static final double DEFAULT_TOLERANCE_CM  = 1.5;
    public static final double DEFAULT_TOLERANCE_DEG = 2.5;

    /** Công suất tối đa cho phần tịnh tiến. */
    public static final double MAX_TRANSLATIONAL_POWER = 0.75;
    /** Công suất tối đa cho phần xoay. */
    public static final double MAX_ROTATIONAL_POWER    = 0.45;

    // ============================================================
    //  State machine
    // ============================================================

    public enum DriveState {
        /** Không làm gì, motor dừng. */
        IDLE,
        /** Đang PID đến target. */
        MOVING,
        /** Đã dạt target, dang giữ vị trí. */
        HOLDING,
        /** Dang chay script JSON (tất cả các bước liên tục). */
        SCRIPT_RUNNING,
        /** Script dã hoàn thành. */
        SCRIPT_DONE,
        /** Dang chay một bước script duy nhất (chờ gọi executeNextStep). */
        SCRIPT_STEP_RUNNING,
        /** Bước script hiện tại dã hoàn thành, dợi lệnh tiếp theo. */
        SCRIPT_STEP_DONE
    }

    // ============================================================
    //  Internals
    // ============================================================

    /**
     * Odometry source — tách interface de test duọc.
     * TwoWheelOdometry dã có sẵn getX() / getY() / getHeadingDeg() nhung
     * không implements interface này. Constructor sē wrap tự dộng.
     */
    public interface OdometryProvider {
        double getX();
        double getY();
        double getHeadingDeg();
    }

    private final DcMotorEx fl, fr, bl, br;
    private final OdometryProvider odometry;

    private final PidController xPid;
    private final PidController yPid;
    private final PidController hPid;

    private DriveState state = DriveState.IDLE;

    private double targetXCm;
    private double targetYCm;
    private double targetHeadingDeg;

    private double toleranceCm  = DEFAULT_TOLERANCE_CM;
    private double toleranceDeg = DEFAULT_TOLERANCE_DEG;

    // ---- Script ----
    private final List<ScriptStep> scriptSteps = new ArrayList<>();
    private int scriptIndex;
    private long scriptStepStartMs;

    // ---- Debug (test inspection) ----
    double lastFlPower, lastFrPower, lastBlPower, lastBrPower;
    double lastRobotForward, lastRobotStrafe, lastRobotRotate;

    // ============================================================
    //  Constructors
    // ============================================================

    /**
     * Production constructor — khơi tao motor tù HardwareMap.
     *
     * @param hwMap    HardwareMap cúa OpMode
     * @param flName   tên motor front-left
     * @param frName   tên motor front-right
     * @param blName   tên motor back-left
     * @param brName   tên motor back-right
     * @param odometry TwoWheelOdometry dã khơi tao (cùng OpMode)
     */
    public MecanumDrive(
            HardwareMap hwMap,
            String flName,
            String frName,
            String blName,
            String brName,
            TwoWheelOdometry odometry
    ) {
        this(hwMap, flName, frName, blName, brName,
                new OdometryProvider() {
                    @Override
                    public double getX() {
                        return odometry.getX();
                    }

                    @Override
                    public double getY() {
                        return odometry.getY();
                    }

                    @Override
                    public double getHeadingDeg() {
                        return odometry.getHeadingDeg();
                    }
                });
    }

    /**
     * Constructor với OdometryProvider — de test offline.
     */
    public MecanumDrive(
            HardwareMap hwMap,
            String flName,
            String frName,
            String blName,
            String brName,
            OdometryProvider odometry
    ) {
        this.fl = hwMap.get(DcMotorEx.class, flName);
        this.fr = hwMap.get(DcMotorEx.class, frName);
        this.bl = hwMap.get(DcMotorEx.class, blName);
        this.br = hwMap.get(DcMotorEx.class, brName);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        this.odometry = odometry;

        xPid = new PidController(
                DEFAULT_POS_KP,
                DEFAULT_POS_KI,
                DEFAULT_POS_KD
        );

        yPid = new PidController(
                DEFAULT_POS_KP,
                DEFAULT_POS_KI,
                DEFAULT_POS_KD
        );

        hPid = new PidController(
                DEFAULT_HEAD_KP,
                DEFAULT_HEAD_KI,
                DEFAULT_HEAD_KD
        );

        xPid.setOutputLimits(
                -MAX_TRANSLATIONAL_POWER,
                MAX_TRANSLATIONAL_POWER
        );

        yPid.setOutputLimits(
                -MAX_TRANSLATIONAL_POWER,
                MAX_TRANSLATIONAL_POWER
        );

        hPid.setOutputLimits(
                -MAX_ROTATIONAL_POWER,
                MAX_ROTATIONAL_POWER
        );
    }

    /**
     * Test constructor — không can FTC hardware.
     * Chi dùng trong offline test.
     */
    MecanumDrive(OdometryProvider odometry) {
        this.fl = null;
        this.fr = null;
        this.bl = null;
        this.br = null;

        this.odometry = odometry;

        xPid = new PidController(
                DEFAULT_POS_KP,
                DEFAULT_POS_KI,
                DEFAULT_POS_KD
        );

        yPid = new PidController(
                DEFAULT_POS_KP,
                DEFAULT_POS_KI,
                DEFAULT_POS_KD
        );

        hPid = new PidController(
                DEFAULT_HEAD_KP,
                DEFAULT_HEAD_KI,
                DEFAULT_HEAD_KD
        );

        xPid.setOutputLimits(
                -MAX_TRANSLATIONAL_POWER,
                MAX_TRANSLATIONAL_POWER
        );

        yPid.setOutputLimits(
                -MAX_TRANSLATIONAL_POWER,
                MAX_TRANSLATIONAL_POWER
        );

        hPid.setOutputLimits(
                -MAX_ROTATIONAL_POWER,
                MAX_ROTATIONAL_POWER
        );
    }

    // ============================================================
    //  update() — gọi mồi vòng lặp
    // ============================================================

    /**
     * Gọi mồi vòng lặp. Tính PID, cap nhật motor, tiến state machine.
     * Phai gọi {@code odometry.update()} TRƯỚC khi goi hàm này.
     */
    public void update() {
        long nowMs = currentTimeMs();

        switch (state) {
            case MOVING:
            case HOLDING:
                updatePositionControl(nowMs);
                break;

            case SCRIPT_RUNNING:
                updateScript(nowMs);
                break;

            case SCRIPT_STEP_RUNNING:
                updateScriptStep(nowMs);
                break;

            default:
                stopMotors();
                break;
        }
    }

    /**
     * PID diêu khiên vị trí.
     */
    private void updatePositionControl(long nowMs) {
        double currentX   = odometry.getX();
        double currentY   = odometry.getY();
        double currentDeg = odometry.getHeadingDeg();

        // PID raw outputs trong frame robot
        double robotStrafe  = xPid.calculate(targetXCm,  currentX);
        double robotForward = yPid.calculate(targetYCm,  currentY);

        double rotate = hPid.calculate(
                targetHeadingDeg,
                currentDeg
        );

        /*
         * Transform tu frame robot sang field-centric
         * de robot giữ trajectory dúng ngay cả khi dang xoay.
         */
        double headingRad = Math.toRadians(currentDeg);
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);

        double fieldForward =
                robotForward * cos + robotStrafe * sin;

        double fieldStrafe =
                -robotForward * sin + robotStrafe * cos;

        lastRobotForward = fieldForward;
        lastRobotStrafe  = fieldStrafe;
        lastRobotRotate  = rotate;

        applyMecanumPowers(
                fieldForward,
                fieldStrafe,
                rotate
        );

        // Chuyên trạng thái
        if (xPid.atSetpoint(toleranceCm)
                && yPid.atSetpoint(toleranceCm)
                && hPid.atSetpoint(toleranceDeg)) {
            state = DriveState.HOLDING;
        } else if (state == DriveState.HOLDING) {
            // Ra khói target → quay lai MOVING
            state = DriveState.MOVING;
        }
    }

    /**
     * Tiến một bước script.
     */
    private void updateScript(long nowMs) {
        if (scriptIndex >= scriptSteps.size()) {
            state = DriveState.SCRIPT_DONE;
            stopMotors();
            return;
        }

        double currentX   = odometry.getX();
        double currentY   = odometry.getY();
        double currentDeg = odometry.getHeadingDeg();

        ScriptStep step = scriptSteps.get(scriptIndex);

        double robotStrafe  = xPid.calculate(step.x, currentX);
        double robotForward = yPid.calculate(step.y, currentY);
        double rotate       = hPid.calculate(step.h, currentDeg);

        double headingRad = Math.toRadians(currentDeg);
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);

        double fieldForward =
                robotForward * cos + robotStrafe * sin;

        double fieldStrafe =
                -robotForward * sin + robotStrafe * cos;

        lastRobotForward = fieldForward;
        lastRobotStrafe  = fieldStrafe;
        lastRobotRotate  = rotate;

        applyMecanumPowers(fieldForward, fieldStrafe, rotate);

        boolean atTarget = xPid.atSetpoint(toleranceCm)
                && yPid.atSetpoint(toleranceCm)
                && hPid.atSetpoint(toleranceDeg);

        long elapsed = nowMs - scriptStepStartMs;

        if (atTarget || elapsed >= step.timeoutMs) {
            scriptIndex++;
            scriptStepStartMs = nowMs;

            xPid.reset();
            yPid.reset();
            hPid.reset();
        }
    }

    // ============================================================
    //  Mecanum kinematics (inverse)
    // ============================================================

    /**
     * Ap dung inverse kinematics cho mecanum.
     *
     * fl = forward + strafe + rotate
     * fr = forward - strafe - rotate
     * bl = forward - strafe + rotate
     * br = forward + strafe - rotate
     *
     * Dầu ra duọc chuẩn hoá nếu vượt [-1, 1] dê giữ tỉ lệ.
     */
    private void applyMecanumPowers(
            double forward,
            double strafe,
            double rotate
    ) {
        double flPower = forward + strafe + rotate;
        double frPower = forward - strafe - rotate;
        double blPower = forward - strafe + rotate;
        double brPower = forward + strafe - rotate;

        // Normalize
        double maxAbs = Math.max(
                Math.max(Math.abs(flPower), Math.abs(frPower)),
                Math.max(Math.abs(blPower), Math.abs(brPower))
        );

        if (maxAbs > 1.0) {
            flPower /= maxAbs;
            frPower /= maxAbs;
            blPower /= maxAbs;
            brPower /= maxAbs;
        }

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

    /**
     * Overload dành cho raw motor powers — bỏ qua kinematics,
     * gán trực tiếp fl/fr/bl/br.
     */
    private void applyMecanumPowers(
            double flPower,
            double frPower,
            double blPower,
            double brPower
    ) {
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

    // ============================================================
    //  Điêu khiên trục tiếp (velocity mode)
    // ============================================================

    /**
     * Đat công suất raw cho 4 motor.
     */
    public void setRawPowers(
            double flPower,
            double frPower,
            double blPower,
            double brPower
    ) {
        state = DriveState.IDLE;
        applyMecanumPowers(flPower, frPower, blPower, brPower);
    }

    /**
     * Điêu khiên field-centric.
     *
     * @param fieldForward  tiến/lùi so với sân (-1..1)
     * @param fieldStrafe   trái/phải so với sân (-1..1)
     * @param rotate        xoay (-1..1)
     */
    public void driveFieldCentric(
            double fieldForward,
            double fieldStrafe,
            double rotate
    ) {
        state = DriveState.IDLE;

        double headingRad =
                Math.toRadians(odometry.getHeadingDeg());

        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);

        // Robot powers from field-centric input
        double robotForward =
                fieldForward * cos - fieldStrafe * sin;

        double robotStrafe =
                fieldForward * sin + fieldStrafe * cos;

        lastRobotForward = robotForward;
        lastRobotStrafe  = robotStrafe;
        lastRobotRotate  = rotate;

        applyMecanumPowers(robotForward, robotStrafe, rotate);
    }

    // ============================================================
    //  PID diêu khiên vị trí
    // ============================================================

    /**
     * Băt dầu di chuyên PID dên (x, y, headingDeg).
     * Gọi một lân dê băt dầu, sau dó gọi update() mồi vòng lap.
     */
    public void goToPosition(
            double xCm,
            double yCm,
            double headingDeg
    ) {
        targetXCm        = xCm;
        targetYCm        = yCm;
        targetHeadingDeg = headingDeg;

        xPid.reset();
        yPid.reset();
        hPid.reset();

        state = DriveState.MOVING;
    }

    /**
     * Giữ vị trí hiện tai.
     */
    public void holdPosition() {
        targetXCm        = odometry.getX();
        targetYCm        = odometry.getY();
        targetHeadingDeg = odometry.getHeadingDeg();

        xPid.reset();
        yPid.reset();
        hPid.reset();

        state = DriveState.MOVING;
    }

    // ============================================================
    //  Script JSON
    // ============================================================

    /**
     * Nap script JSON.
     *
     * Format:
     * <pre>
     * [
     *   {"x": 50, "y": 0,  "h": 90, "timeout": 3000},
     *   {"x": 50, "y": 50, "h": 0,  "timeout": 3000}
     * ]
     * </pre>
     *
     * x, y: cm.  h: degrees.  timeout: ms.
     */
    public void loadScript(String json) {
        scriptSteps.clear();
        scriptIndex = 0;

        if (json == null || json.trim().isEmpty()) {
            return;
        }

        /*
         * Parser don gian, không phu thuộc org.json
         * de offline test chay duọc trên PC không cần Android SDK.
         */
        String trimmed = json.trim();

        if (!trimmed.startsWith("[")) {
            return;
        }

        // Split by "},{" to get individual objects
        String inner = trimmed.substring(1, trimmed.length() - 1).trim();

        if (inner.isEmpty()) {
            return;
        }

        // Find object boundaries
        int depth = 0;
        int start  = -1;

        for (int i = 0; i < inner.length(); i++) {
            char c = inner.charAt(i);

            if (c == '{') {
                if (depth == 0) start = i;
                depth++;
            } else if (c == '}') {
                depth--;
                if (depth == 0 && start >= 0) {
                    parseStep(inner.substring(start, i + 1));
                }
            }
        }
    }

    private void parseStep(String object) {
        double x = 0, y = 0, h = 0;
        long timeout = 5000;

        String inner = object.substring(1, object.length() - 1);

        // Split fields by comma outside quotes
        List<String> fields = splitJsonFields(inner);

        for (String field : fields) {
            int colon = field.indexOf(':');

            if (colon < 0) continue;

            String key = field.substring(0, colon)
                    .replace("\"", "").trim();

            String value = field.substring(colon + 1)
                    .replace("\"", "").trim();

            try {
                switch (key) {
                    case "x":       x       = Double.parseDouble(value); break;
                    case "y":       y       = Double.parseDouble(value); break;
                    case "h":       h       = Double.parseDouble(value); break;
                    case "timeout": timeout = Long.parseLong(value);     break;
                }
            } catch (NumberFormatException ignored) {
            }
        }

        scriptSteps.add(new ScriptStep(x, y, h, timeout));
    }

    private List<String> splitJsonFields(String inner) {
        List<String> result = new ArrayList<>();

        int start = 0;
        int braceDepth = 0;
        boolean inString = false;

        for (int i = 0; i < inner.length(); i++) {
            char c = inner.charAt(i);

            if (c == '"' && (i == 0 || inner.charAt(i - 1) != '\\')) {
                inString = !inString;
            }

            if (!inString) {
                if (c == '{') braceDepth++;
                if (c == '}') braceDepth--;
            }

            if (!inString && braceDepth == 0 && c == ',') {
                result.add(inner.substring(start, i).trim());
                start = i + 1;
            }
        }

        if (start < inner.length()) {
            result.add(inner.substring(start).trim());
        }

        return result;
    }

    /** Bắt dầu chay script. */
    public void startScript() {
        if (scriptSteps.isEmpty()) {
            state = DriveState.SCRIPT_DONE;
            return;
        }

        scriptIndex        = 0;
        scriptStepStartMs  = currentTimeMs();

        xPid.reset();
        yPid.reset();
        hPid.reset();

        state = DriveState.SCRIPT_RUNNING;
    }

    /** Dừng script giữa chừng. */
    public void abortScript() {
        scriptSteps.clear();
        scriptIndex = 0;
        stop();
    }

    /**
     * Chạy bước tiếp theo trong script. Mỗi lần gọi một bước.
     *
     * Gọi update() mỗi vòng lặp sau khi gọi hàm này. Khi bước hoàn
     * thành (dạt target hoặc timeout), state chuyển về
     * SCRIPT_STEP_DONE. Gọi executeNextStep() thêm lần nữa dể
     * chạy bước kế tiếp.
     *
     * @return true nếu có bước dể chạy, false nếu dã hết script
     */
    public boolean executeNextStep() {
        if (scriptSteps.isEmpty()
                || scriptIndex >= scriptSteps.size()) {
            state = DriveState.SCRIPT_DONE;
            return false;
        }

        scriptStepStartMs  = currentTimeMs();

        xPid.reset();
        yPid.reset();
        hPid.reset();

        state = DriveState.SCRIPT_STEP_RUNNING;
        return true;
    }

    /**
     * Chạy một bước script cho đến khi hoàn thành (blocking style).
     *
     * Cảnh báo: hàm này chạy vòng lặp nội bộ (không có telemetry).
     * Chi nên dùng trong Autonomous nếu OpMode không cần telemetry.
     *
     * @return true nếu bước hoàn thành, false nếu hết script
     */
    public boolean executeNextStepBlocking() {
        if (!executeNextStep()) {
            return false;
        }

        while (state == DriveState.SCRIPT_STEP_RUNNING) {
            update();
        }

        return state != DriveState.SCRIPT_DONE;
    }

    /**
     * Tiến một bước script riêng lẻ — không tự dộng chuyển
     * sang bước tiếp theo.
     */
    private void updateScriptStep(long nowMs) {
        if (scriptIndex >= scriptSteps.size()) {
            state = DriveState.SCRIPT_DONE;
            stopMotors();
            return;
        }

        double currentX   = odometry.getX();
        double currentY   = odometry.getY();
        double currentDeg = odometry.getHeadingDeg();

        ScriptStep step = scriptSteps.get(scriptIndex);

        double robotStrafe  = xPid.calculate(step.x, currentX);
        double robotForward = yPid.calculate(step.y, currentY);
        double rotate       = hPid.calculate(step.h, currentDeg);

        double headingRad = Math.toRadians(currentDeg);
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);

        double fieldForward =
                robotForward * cos + robotStrafe * sin;

        double fieldStrafe =
                -robotForward * sin + robotStrafe * cos;

        lastRobotForward = fieldForward;
        lastRobotStrafe  = fieldStrafe;
        lastRobotRotate  = rotate;

        applyMecanumPowers(fieldForward, fieldStrafe, rotate);

        boolean atTarget = xPid.atSetpoint(toleranceCm)
                && yPid.atSetpoint(toleranceCm)
                && hPid.atSetpoint(toleranceDeg);

        long elapsed = nowMs - scriptStepStartMs;

        if (atTarget || elapsed >= step.timeoutMs) {
            // Đánh dấu bước hiện tại hoàn thành, dợi lệnh tiếp theo
            scriptIndex++;

            xPid.reset();
            yPid.reset();
            hPid.reset();

            if (scriptIndex >= scriptSteps.size()) {
                state = DriveState.SCRIPT_DONE;
                stopMotors();
            } else {
                state = DriveState.SCRIPT_STEP_DONE;
                stopMotors();
            }
        }
    }

    /** Bước script hiện tại (0-based). */
    public int getScriptStepIndex() {
        return scriptIndex;
    }

    /** Tổng số bước trong script. */
    public int getScriptStepCount() {
        return scriptSteps.size();
    }

    // ============================================================
    //  Control
    // ============================================================

    /** Dùng mọi motor, trở về IDLE. */
    public void stop() {
        state = DriveState.IDLE;
        stopMotors();
    }

    private void stopMotors() {
        lastFlPower = 0;
        lastFrPower = 0;
        lastBlPower = 0;
        lastBrPower = 0;

        if (fl != null) {
            fl.setPower(0);
            fr.setPower(0);
            bl.setPower(0);
            br.setPower(0);
        }
    }

    // ============================================================
    //  Queries
    // ============================================================

    public DriveState getState() {
        return state;
    }

    /** Dã dến target (POSITION mode) hoặc script current step? */
    public boolean atTarget() {
        return state == DriveState.HOLDING;
    }

    public boolean atTarget(double toleranceCm, double toleranceDeg) {
        if (state != DriveState.MOVING && state != DriveState.HOLDING) {
            return false;
        }

        double dx = Math.abs(odometry.getX() - targetXCm);
        double dy = Math.abs(odometry.getY() - targetYCm);
        double dh = Math.abs(
                angleDiff(
                        odometry.getHeadingDeg(),
                        targetHeadingDeg
                )
        );

        return dx <= toleranceCm
                && dy <= toleranceCm
                && dh <= toleranceDeg;
    }

    /** Lôi toạ dô còn lai (Euclidean). */
    public double getRemainingError() {
        if (state != DriveState.MOVING
                && state != DriveState.HOLDING
                && state != DriveState.SCRIPT_RUNNING) {
            return 0.0;
        }

        double dx = odometry.getX() - targetXCm;
        double dy = odometry.getY() - targetYCm;

        return Math.sqrt(dx * dx + dy * dy);
    }

    // ============================================================
    //  Tuning
    // ============================================================

    public void setTolerance(double toleranceCm, double toleranceDeg) {
        this.toleranceCm  = toleranceCm;
        this.toleranceDeg = toleranceDeg;
    }

    public void setPositionGains(double kp, double ki, double kd) {
        xPid.setGains(kp, ki, kd);
        yPid.setGains(kp, ki, kd);
    }

    public void setHeadingGains(double kp, double ki, double kd) {
        hPid.setGains(kp, ki, kd);
    }

    public PidController getXPid() { return xPid; }
    public PidController getYPid() { return yPid; }
    public PidController getHPid() { return hPid; }

    public OdometryProvider getOdometryProvider() {
        return odometry;
    }

    // ============================================================
    //  Helpers
    // ============================================================

    private static double angleDiff(double a, double b) {
        double d = Math.abs(a - b) % 360.0;
        return d > 180.0 ? 360.0 - d : d;
    }

    private static long currentTimeMs() {
        return System.currentTimeMillis();
    }

    // ============================================================
    //  Inner types
    // ============================================================

    private static final class ScriptStep {

        final double x;
        final double y;
        final double h;
        final long   timeoutMs;

        ScriptStep(double x, double y, double h, long timeoutMs) {
            this.x = x;
            this.y = y;
            this.h = h;
            this.timeoutMs = timeoutMs;
        }
    }
}
