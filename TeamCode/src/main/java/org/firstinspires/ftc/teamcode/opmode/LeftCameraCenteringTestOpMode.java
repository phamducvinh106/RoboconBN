package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.core.CameraAdapterManager;
import org.firstinspires.ftc.teamcode.core.CameraChannel;
import org.firstinspires.ftc.teamcode.core.CameraFrameContract;
import org.firstinspires.ftc.teamcode.core.LiftingSequenceConfig;
import org.firstinspires.ftc.teamcode.core.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.Pi5CameraTransportFactory;
import org.firstinspires.ftc.teamcode.core.PiCdcCameraTransport;
import org.firstinspires.ftc.teamcode.core.PiCdcPacket;
import org.firstinspires.ftc.teamcode.core.RobotConfigAssets;
import org.firstinspires.ftc.teamcode.core.RobotHardware;

@TeleOp(name = "Left Camera Centering Test", group = "Test")
public final class LeftCameraCenteringTestOpMode extends LinearOpMode {
    private static final double TARGET_X = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        LiftingSequenceConfig config;
        try {
            config = RobotConfigAssets.load(hardwareMap.appContext.getAssets());
        } catch (Exception error) {
            telemetry.addData("SAFE_STOP", "invalid config: %s", error.getMessage());
            telemetry.update();
            return;
        }
        if (config.cameraFrameWidth != PiCdcPacket.DEFAULT_FRAME_WIDTH
                || config.cameraFrameHeight != PiCdcPacket.DEFAULT_FRAME_HEIGHT) {
            telemetry.addData("SAFE_STOP", "expected %dx%d frame",
                    PiCdcPacket.DEFAULT_FRAME_WIDTH, PiCdcPacket.DEFAULT_FRAME_HEIGHT);
            telemetry.update();
            return;
        }

        RobotHardware robot = new RobotHardware(hardwareMap, config);
        MecanumDrive drive = new MecanumDrive(
                hardwareMap, "leftfront", "rightfront", "leftback", "rightback", robot.localizer);
        PiCdcCameraTransport cameraTransport = Pi5CameraTransportFactory.createTransport(
                hardwareMap, config, System::nanoTime);
        CameraAdapterManager cameras = new CameraAdapterManager(cameraTransport, config.sensorStaleNs);

        final int frameWidth = config.cameraFrameWidth;
        final int frameHeight = config.cameraFrameHeight;
        final double deadbandNorm = config.centerDeadbandPx / (double) frameWidth;

        telemetry.addLine("Left camera horizontal centering test");
        telemetry.addLine("Giữ A để căn ngang");
        telemetry.addLine("Release A / stale / invalid = stop");
        telemetry.update();
        waitForStart();

        try {
            while (opModeIsActive() && !isStopRequested()) {
                long now = System.nanoTime();
                robot.localizer.update();
                CameraFrameContract left = cameras.reading(CameraChannel.WEBCAM1);
                boolean fresh = left.fresh(now, config.sensorStaleNs);
                boolean active = gamepad1.a;

                double xNorm = Double.NaN;
                double yNorm = Double.NaN;
                double dxNorm = Double.NaN;
                String status = "STOPPED";
                String command = "none";

                if (!active) {
                    drive.stop();
                    status = "STOPPED";
                } else if (!left.valid || !fresh || !left.channelFound) {
                    drive.stop();
                    status = "STOPPED";
                } else {
                    xNorm = left.centerX / (double) frameWidth;
                    yNorm = left.centerY / (double) frameHeight;
                    dxNorm = xNorm - TARGET_X;
                    if (Math.abs(dxNorm) <= deadbandNorm) {
                        drive.stop();
                        status = "CENTERED";
                        command = "hold";
                    } else {
                        double speed = config.centerSpeed;
                        double strafe = dxNorm > 0.0 ? -speed : speed;
                        drive.driveRobotCentric(0.0, strafe, 0.0);
                        status = "MOVING";
                        command = strafe < 0.0 ? "strafe-right" : "strafe-left";
                    }
                }

                telemetry.addData("control", "Giữ A để căn ngang | active=%s", active);
                telemetry.addData("status", status);
                telemetry.addData("x", "%.3f", xNorm);
                telemetry.addData("y", "%.3f", yNorm);
                telemetry.addData("dx", "%.3f", dxNorm);
                telemetry.addData("targetX", "%.3f", TARGET_X);
                telemetry.addData("deadband", "%.3f", deadbandNorm);
                telemetry.addData("pixel", "centerX=%d centerY=%d (%dx%d)",
                        left.centerX, left.centerY, frameWidth, frameHeight);
                telemetry.addData("camera", "valid=%s fresh=%s found=%s type=%s hb=%d",
                        left.valid, fresh, left.channelFound, left.blockType, left.heartbeat);
                telemetry.addData("command", command);
                telemetry.update();
                idle();
            }
        } finally {
            cameraTransport.close();
            drive.stop();
            robot.stopActuators();
        }
    }
}
