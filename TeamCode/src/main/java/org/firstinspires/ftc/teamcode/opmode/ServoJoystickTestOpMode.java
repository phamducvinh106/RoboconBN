package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@TeleOp(name = "Servo Joystick Test", group = "Test")
public final class ServoJoystickTestOpMode extends LinearOpMode {
    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;
    private static final double DEADBAND = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        ServoImplEx servoLeft = hardwareMap.get(ServoImplEx.class, "servoLeft");
        ServoImplEx servoRight = hardwareMap.get(ServoImplEx.class, "servoRight");
        servoLeft.setDirection(Servo.Direction.REVERSE);
        servoRight.setDirection(Servo.Direction.FORWARD);
        PwmControl.PwmRange pwmRange = new PwmControl.PwmRange(500, 2500);
        servoLeft.setPwmRange(pwmRange);
        servoRight.setPwmRange(pwmRange);
        double leftPosition = 0.5;
        double rightPosition = 0.5;
        servoLeft.setPosition(leftPosition);
        servoRight.setPosition(rightPosition);

        telemetry.addLine("SERVO JOYSTICK TEST");
        telemetry.addLine("Left stick Y: servoLeft | Right stick Y: servoRight");
        telemetry.addLine("Stick up increases position; stick down decreases position");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            double leftInput = -gamepad1.left_stick_y;
            double rightInput = -gamepad1.right_stick_y;
            if (Math.abs(leftInput) > DEADBAND) {
                leftPosition = clip(leftPosition + leftInput * 0.02);
            }
            if (Math.abs(rightInput) > DEADBAND) {
                rightPosition = clip(rightPosition + rightInput * 0.02);
            }

            servoLeft.setPosition(leftPosition);
            servoRight.setPosition(rightPosition);
            telemetry.addData("servoLeft", "%.3f", leftPosition);
            telemetry.addData("servoRight", "%.3f", rightPosition);
            telemetry.update();
            sleep(20);
        }
    }

    private static double clip(double position) {
        return Math.max(SERVO_MIN, Math.min(SERVO_MAX, position));
    }
}
