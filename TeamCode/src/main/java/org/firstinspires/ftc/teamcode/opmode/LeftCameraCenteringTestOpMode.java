package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Localizer;
import org.firstinspires.ftc.teamcode.core.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.RobotHardware;
import org.firstinspires.ftc.teamcode.core.ColorContourCamera;

@TeleOp(name = "Left Camera Centering Test", group = "Test")
public final class LeftCameraCenteringTestOpMode extends LinearOpMode {
    private static final String CAMERA_NAME = "webcam1";
    private static final double STRAFE_POWER = 0.16;
    private static final long MAX_RESULT_AGE_MS = 300;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        Localizer localizer = robot.localizer;
        MecanumDrive drive = new MecanumDrive(
                hardwareMap, "leftfront", "rightfront", "leftback", "rightback", localizer
        );
        ColorContourCamera camera = new ColorContourCamera(
                hardwareMap, CAMERA_NAME, true, ColorContourCamera.Mode.SINGLE_TARGET
        );
        camera.startAsync();

        try {
            telemetry.addLine("LEFT CAMERA SINGLE-TARGET CENTERING");
            telemetry.addLine("Only webcam1 controls strafe; no right camera/IR/pickup");
            telemetry.addLine("ColorContourCamera: BLOCK_1..BLOCK_4 | A: stop drive");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                ColorContourCamera.Result result = camera.getLatestResult();
                boolean fresh = result.valid
                        && System.currentTimeMillis() - result.timestampMs <= MAX_RESULT_AGE_MS;
                double strafe = 0.0;
                if (fresh && result.fastCentering && Math.abs(result.dxPx) > 4.0) {
                    strafe = result.dxPx > 0 ? STRAFE_POWER : -STRAFE_POWER;
                }
                if (gamepad1.a) strafe = 0.0;

                localizer.update();
                drive.driveRobotCentric(0.0, strafe, 0.0);
                telemetry.addData("class", "%s / stable=%d", result.label, result.stableFrames);
                telemetry.addData("camera", "%s / fresh=%s", camera.getCameraState(), fresh);
                telemetry.addData("result", "dx %.1f / dy %.1f / confidence %.3f",
                        result.dxPx, result.dyPx, result.confidence);
                telemetry.addData("stale age", "%d ms", System.currentTimeMillis() - result.timestampMs);
                telemetry.addData("command", "strafe %.2f", strafe);
                telemetry.addData("pose", "X %.1f / Y %.1f / H %.1f",
                        localizer.getX(), localizer.getY(), localizer.getHeadingDeg());
                telemetry.update();
                sleep(20);
            }
        } finally {
            drive.stop();
            camera.stop();
        }
    }
}
