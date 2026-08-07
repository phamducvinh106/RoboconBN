package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.core.ColorContourCamera;
import org.firstinspires.ftc.teamcode.core.Localizer;
import org.firstinspires.ftc.teamcode.core.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.RobotHardware;

@TeleOp(name = "Left Color Centering Test", group = "Test")
public final class LeftColorCenteringTestOpMode extends LinearOpMode {
    private static final double STRAFE_POWER = 0.16;
    private static final double CENTER_DEADBAND_PX = 4.0;

    @Override public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        Localizer localizer = robot.localizer;
        MecanumDrive drive = new MecanumDrive(hardwareMap, "leftfront", "rightfront", "leftback", "rightback", localizer);
        ColorContourCamera camera = new ColorContourCamera(hardwareMap, "webcam1", true, ColorContourCamera.Mode.SINGLE_TARGET);
        camera.startAsync();
        waitForStart();
        try {
            while (opModeIsActive()) {
                ColorContourCamera.Result r = camera.getLatestResult();
                double strafe = r.valid && r.fastCentering && Math.abs(r.dxPx) > CENTER_DEADBAND_PX
                        ? (r.dxPx > 0 ? STRAFE_POWER : -STRAFE_POWER) : 0;
                localizer.update();
                drive.driveRobotCentric(0, strafe, 0);
                telemetry.addData("camera", "%s (%d)", camera.getCameraState(), camera.getCameraErrorCode());
                telemetry.addData("mode", r.fastCentering ? "FAST_CENTERING" : "CLASSIFYING");
                telemetry.addData("class", "%s stable=%d/5", r.label, r.stableFrames);
                telemetry.addData("center", "dx %.1f / dy %.1f", r.dxPx, r.dyPx);
                telemetry.addData("confidence", "%.2f", r.confidence);
                telemetry.addData("strafe", "%.2f", strafe);
                telemetry.update();
                sleep(20);
            }
        } finally { drive.stop(); camera.stop(); }
    }
}
