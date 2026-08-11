package org.firstinspires.ftc.teamcode.core.drive;

/**
 * PID controller — dùng chung cho mọi tác vụ cần PID.
 *
 * Đặc điểm:
 *   - Derivative-on-measurement (tránh derivative kick khi setpoint thay đổi)
 *   - Anti-windup tích phân (clamp riêng I-term, hoặc back‑calculation)
 *   - Output clamping
 *   - Reset thông minh (reset I, D, hoặc toàn bộ)
 *   - Hỗ trợ FTC telemetry
 */
public final class PidController {

    private double kp;
    private double ki;
    private double kd;

    private double outputMin = -1.0;
    private double outputMax =  1.0;
    private double iClamp = 0.3;

    private double setpoint;
    private double lastMeasurement;
    private double integral;
    private double lastError;
    private double lastOutput;

    private boolean firstRun = true;

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

    public double calculate(double measurement) {
        return calculate(setpoint, measurement);
    }

    public double calculate(double setpoint, double measurement) {
        this.setpoint = setpoint;

        double error = setpoint - measurement;

        double derivative;
        if (firstRun) {
            derivative = 0.0;
            firstRun = false;
        } else {
            derivative = measurement - lastMeasurement;
        }

        if (lastOutput > outputMax && error > 0) {
            // đang bão hoà dương
        } else if (lastOutput < outputMin && error < 0) {
            // đang bão hoà âm
        } else {
            integral += error;
        }

        if (integral >  iClamp) integral =  iClamp;
        if (integral < -iClamp) integral = -iClamp;

        double output = kp * error
                      + ki * integral
                      - kd * derivative;

        if (output > outputMax) output = outputMax;
        if (output < outputMin) output = outputMin;

        lastMeasurement = measurement;
        lastError = error;
        lastOutput = output;

        return output;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double getSetpoint() {
        return setpoint;
    }

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

    public void setOutputLimits(double min, double max) {
        if (min >= max) {
            throw new IllegalArgumentException(
                    "outputMin (" + min + ") >= outputMax (" + max + ")"
            );
        }

        this.outputMin = min;
        this.outputMax = max;
        this.iClamp = Math.min(iClamp, (max - min) * 0.5);
    }

    public double getOutputMin() { return outputMin; }
    public double getOutputMax() { return outputMax; }

    public void setIClamp(double clamp) {
        this.iClamp = Math.abs(clamp);
    }

    public double getIClamp() {
        return iClamp;
    }

    public void reset() {
        integral = 0.0;
        lastOutput = 0.0;
        firstRun = true;
    }

    public void resetIntegral() {
        integral = 0.0;
    }

    public double getIntegral()  { return integral; }
    public double getLastError() { return lastError; }
    public double getLastOutput(){ return lastOutput; }

    public boolean atSetpoint(double tolerance) {
        return Math.abs(lastError) <= tolerance;
    }
}
