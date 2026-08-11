package org.firstinspires.ftc.teamcode.opmode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.odometry.Localizer;
import org.firstinspires.ftc.teamcode.core.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.RobotHardware;

@TeleOp(name = "Pod Sign Offset Test", group = "Calibration")
public final class PodSignOffsetTestOpMode extends LinearOpMode {
    private static final double ROTATE_POWER = 0.16;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        Localizer localizer = robot.localizer;
        MecanumDrive drive = new MecanumDrive(
                hardwareMap, "leftfront", "rightfront", "leftback", "rightback", localizer
        );

        telemetry.addLine("POD SIGN / OFFSET TEST");
        telemetry.addLine("A: reset | B: clockwise 360 degrees");
        telemetry.addLine("leftfront encoder = parallel; rightfront encoder = perpendicular");
        telemetry.update();
        waitForStart();

        boolean previousA = false;
        boolean previousB = false;
        boolean rotating = false;
        long deadline = 0;
        try {
            while (opModeIsActive()) {
                boolean a = gamepad1.a;
                boolean b = gamepad1.b;
                if (a && !previousA) localizer.resetPoseAndHeading();
                if (b && !previousB) {
                    localizer.resetPoseAndHeading();
                    rotating = true;
                    deadline = System.nanoTime() + 15_000_000_000L;
                }
                previousA = a;
                previousB = b;

                localizer.update();
                if (rotating && Math.toDegrees(localizer.getAccumHeadingRad()) <= -360.0) {
                    rotating = false;
                }
                if (rotating && System.nanoTime() >= deadline) rotating = false;

                drive.setRawPowers(
                        rotating ? -ROTATE_POWER : 0,
                        rotating ? ROTATE_POWER : 0,
                        rotating ? -ROTATE_POWER : 0,
                        rotating ? ROTATE_POWER : 0
                );

                telemetry.addData("raw delta", "parallel %d / perpendicular %d",
                        localizer.getLastParallelDeltaTicks(), localizer.getLastPerpendicularDeltaTicks());
                telemetry.addData("heading", "%.2f deg", localizer.getHeadingDeg());
                telemetry.addData("accum cm", "parallel %.3f / perpendicular %.3f",
                        localizer.getAccumParallelCm(), localizer.getAccumPerpendicularCm());
                telemetry.addData("local cm", "forward %.3f / left %.3f",
                        localizer.getLastForwardLocalCm(), localizer.getLastLeftLocalCm());
                telemetry.addData("suggested offset cm", "Y %.4f / X %.4f",
                        localizer.getSuggestedParallelYOffsetCm(),
                        localizer.getSuggestedPerpendicularXOffsetCm());
                telemetry.addData("pose cm", "X %.3f / Y %.3f", localizer.getX(), localizer.getY());
                telemetry.update();
                sleep(20);
            }
        } finally {
            drive.stop();
        }
    }
}
