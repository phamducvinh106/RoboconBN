package org.firstinspires.ftc.teamcode.core;

/**
 * PID controller — dùng chung cho mọi tác vụ cần PID.
 *
 * Đặc điểm:
 *   - Derivative-on-measurement (tránh derivative kick khi setpoint thay đổi)
 *   - Anti-windup tích phân (clamp riêng I-term, hoặc back‑calculation)
 *   - Output clamping
 *   - Reset thông minh (reset I, D, hoặc toàn bộ)
 *   - Hỗ trợ FTC telemetry
 *
 * Cách dùng tối thiểu:
 * <pre>{@code
 *   PidController pid = new PidController(kp, ki, kd);
 *   pid.setOutputLimits(-1.0, 1.0);
 *
 *   while (true) {
 *       double power = pid.calculate(targetPosition, currentPosition);
 *       motor.setPower(power);
 *       if (atTarget) pid.reset();
 *   }
 * }</pre>
 */
public final class PidController {

    // ============================================================
    //  >>>  TUNE  <<<
    // ============================================================
    private double kp;
    private double ki;
    private double kd;

    // --- Output limits (mặc định ±1.0 cho motor power) ---
    private double outputMin = -1.0;
    private double outputMax =  1.0;

    // --- I-term clamp riêng (anti-windup) ---
    private double iClamp = 0.3;

    // ============================================================
    //  Nội bộ
    // ============================================================

    private double setpoint;
    private double lastMeasurement;
    private double integral;
    private double lastError;
    private double lastOutput;

    private boolean firstRun = true;

    // ============================================================
    //  Constructors
    // ============================================================

    public PidController(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public PidController(double kp, double ki, double kd,
                         double outputMin, double outputMax) {
        this(kp, ki, kd);
        setOutputLimits(outputMin, outputMax);
    }

    // ============================================================
    //  calculate() — gọi mỗi vòng lặp
    // ============================================================

    /**
     * Tính toán output PID.
     *
     * @param measurement  giá trị đo hiện tại (encoder, gyro, ...)
     * @return output đã clamp
     */
    public double calculate(double measurement) {
        return calculate(setpoint, measurement);
    }

    /**
     * Tính toán output PID, đồng thời cập nhật setpoint.
     *
     * @param setpoint    giá trị đích
     * @param measurement giá trị đo hiện tại
     * @return output đã clamp
     */
    public double calculate(double setpoint, double measurement) {
        this.setpoint = setpoint;

        double error = setpoint - measurement;

        // Derivative on measurement (không phải on error)
        double derivative;
        if (firstRun) {
            derivative = 0.0;
            firstRun = false;
        } else {
            derivative = measurement - lastMeasurement;
        }

        // Tích phân chỉ khi output chưa bão hoà (anti-windup đơn giản)
        if (lastOutput > outputMax && error > 0) {
            // đang bão hoà dương, error dương (tiếp tục đẩy) → không tích luỹ
        } else if (lastOutput < outputMin && error < 0) {
            // đang bão hoà âm, error âm (tiếp tục đẩy) → không tích luỹ
        } else {
            integral += error;
        }

        // Clamp I-term riêng
        if (integral >  iClamp) integral =  iClamp;
        if (integral < -iClamp) integral = -iClamp;

        double output = kp * error
                      + ki * integral
                      - kd * derivative;   // trừ vì derivative = delta measurement

        // Clamp output
        if (output > outputMax) output = outputMax;
        if (output < outputMin) output = outputMin;

        lastMeasurement = measurement;
        lastError = error;
        lastOutput = output;

        return output;
    }

    // ============================================================
    //  setpoint
    // ============================================================

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getSetpoint() {
        return setpoint;
    }

    // ============================================================
    //  Tuning — thay đổi gains khi đang chạy
    // ============================================================

    public void setKp(double kp) {
        this.kp = kp;
    }

    public void setKi(double ki) {
        this.ki = ki;
    }

    public void setKd(double kd) {
        this.kd = kd;
    }

    public void setGains(double kp, double ki, double kd) {
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

    public double getKp() { return kp; }
    public double getKi() { return ki; }
    public double getKd() { return kd; }

    // ============================================================
    //  Output limits
    // ============================================================

    public void setOutputLimits(double min, double max) {
        if (min >= max) {
            throw new IllegalArgumentException(
                    "outputMin (" + min + ") >= outputMax (" + max + ")"
            );
        }

        this.outputMin = min;
        this.outputMax = max;

        // Clamp I-term limit về khoảng output hợp lý
        this.iClamp = Math.min(iClamp, (max - min) * 0.5);
    }

    public double getOutputMin() { return outputMin; }
    public double getOutputMax() { return outputMax; }

    // ============================================================
    //  I-term clamp
    // ============================================================

    /**
     * Đặt giới hạn tích phân riêng (thường  nhỏ hơn outputMax).
     * Mặc định = 0.3.
     */
    public void setIClamp(double clamp) {
        this.iClamp = Math.abs(clamp);
    }

    public double getIClamp() {
        return iClamp;
    }

    // ============================================================
    //  Reset
    // ============================================================

    /**
     * Reset toàn bộ trạng thái (I=0, D=0).
     * Gọi khi bắt đầu tác vụ PID mới.
     */
    public void reset() {
        integral = 0.0;
        lastOutput = 0.0;
        firstRun = true;
    }

    /**
     * Reset I-term (chỉ xoá tích phân, giữ D state).
     * Gọi khi vượt qua ngưỡng để tránh I tích luỹ quá mức.
     */
    public void resetIntegral() {
        integral = 0.0;
    }

    // ============================================================
    //  Getters nội bộ (dùng cho telemetry / debug)
    // ============================================================

    public double getIntegral()  { return integral; }
    public double getLastError() { return lastError; }
    public double getLastOutput(){ return lastOutput; }

    /**
     * True nếu error đang trong khoảng chấp nhận được.
     */
    public boolean atSetpoint(double tolerance) {
        return Math.abs(lastError) <= tolerance;
    }
}
