package org.firstinspires.ftc.teamcode.core.pen;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Điều khiển servo bút — calibrate {@code upPosition} / {@code downPosition} trên robot thật.
 *
 * Thứ tự an toàn khi dừng: luôn nhấc bút ({@link #penUp()}) trước khi tắt drive.
 */
public final class PenServoManager {

    private final Servo servo;
    private final RobotConfig.PenConfig config;
    private boolean penIsDown;

    public PenServoManager(Servo servo, RobotConfig.PenConfig config) {
        if (servo == null || config == null) throw new IllegalArgumentException("missing pen hardware");
        this.servo = servo;
        this.config = config;
        penIsDown = false;
    }

    /** Nhấc bút — vị trí servo thấp hơn (mặc định 0.05, tune trên sân). */
    public void penUp() {
        servo.setPosition(config.upPosition);
        penIsDown = false;
    }

    /** Hạ bút — vị trí servo cao hơn (mặc định 0.45, tune trên sân). */
    public void penDown() {
        servo.setPosition(config.downPosition);
        penIsDown = true;
    }

    /** Dừng khẩn — nhấc bút để không kéo vết khi abort. */
    public void stop() {
        penUp();
    }

    public boolean isPenDown() {
        return penIsDown;
    }
}
