package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.Localizer;
import org.firstinspires.ftc.teamcode.core.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.RobotHardware;
import org.firstinspires.ftc.teamcode.core.TemplateMatchCamera;

@TeleOp(name = "Left Camera Centering Test", group = "Test")
public final class LeftCameraCenteringTestOpMode extends LinearOpMode {
    private static final String CAMERA_NAME = "webcam1";
    private static final String[] TARGETS = {
            "target1.png", "target2.png", "target3.png", "target4.png"
    };
    private static final double STRAFE_POWER = 0.16;
    private static final long MAX_RESULT_AGE_MS = 300;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        Localizer localizer = robot.localizer;
        MecanumDrive drive = new MecanumDrive(
                hardwareMap, "leftfront", "rightfront", "leftback", "rightback", localizer
        );
        TemplateMatchCamera camera = new TemplateMatchCamera(
                hardwareMap, CAMERA_NAME, true, TARGETS[0],
                TemplateMatchCamera.CameraMode.SINGLE_TARGET,
                new TemplateMatchCamera.CameraConfig()
        );
        camera.startAsync();

        int targetIndex = 0;
        boolean[] previousDpad = new boolean[4];
        try {
            telemetry.addLine("LEFT CAMERA SINGLE-TARGET CENTERING");
            telemetry.addLine("Only webcam1 controls strafe; no right camera/IR/pickup");
            telemetry.addLine("D-pad: choose target 1..4 | A: stop drive");
            telemetry.update();
            waitForStart();

            while (opModeIsActive()) {
                boolean[] dpad = {
                        gamepad1.dpad_up, gamepad1.dpad_right,
                        gamepad1.dpad_down, gamepad1.dpad_left
                };
                for (int i = 0; i < TARGETS.length; i++) {
                    if (dpad[i] && !previousDpad[i]) {
                        targetIndex = i;
                        camera.setTarget(TARGETS[targetIndex]);
                    }
                    previousDpad[i] = dpad[i];
                }

                TemplateMatchCamera.CameraResult result = camera.getLatestResult();
                boolean fresh = result.isValid()
                        && result.staleAgeMs <= MAX_RESULT_AGE_MS;
                double strafe = 0.0;
                if (fresh && Math.abs(result.dxPx) > camera.getConfig().centerDeadbandPx) {
                    strafe = result.dxPx > 0 ? STRAFE_POWER : -STRAFE_POWER;
                }
                if (gamepad1.a) strafe = 0.0;

                localizer.update();
                drive.driveRobotCentric(0.0, strafe, 0.0);
                telemetry.addData("target", "%d: %s", targetIndex + 1, TARGETS[targetIndex]);
                telemetry.addData("camera", "%s / fresh=%s", camera.getCameraState(), fresh);
                telemetry.addData("result", "dx %.1f / dy %.1f / confidence %.3f",
                        result.dxPx, result.dyPx, result.confidence);
                telemetry.addData("stale age", "%d ms", result.staleAgeMs);
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
