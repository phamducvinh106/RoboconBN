package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.RobotConfigAssets;
import org.firstinspires.ftc.teamcode.core.RobotHardware;
import org.firstinspires.ftc.teamcode.core.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.odometry.Localizer;
import org.firstinspires.ftc.teamcode.core.pen.DrawPathConfig;
import org.firstinspires.ftc.teamcode.core.pen.DrawPathFollower;
import org.firstinspires.ftc.teamcode.core.pen.PenServoManager;
import org.firstinspires.ftc.teamcode.core.pen.RobotConfig;

/**
 * Autonomous vẽ tranh theo draw-path.json.
 *
 * Checklist trước START:
 * - Robot Config FTC: 4 motor mecanum, 2 encoder pod, IMU, servo penServo
 * - Đặt robot đúng startPose (pose trong JSON)
 * - Calibrate upPosition/downPosition trong robot-config.json
 * - Kiểm tra bút không chạm giấy khi penUp
 */
@Autonomous(name = "Draw Art", group = "Autonomous")
public final class DrawArtOpMode extends LinearOpMode {

    @Override
    public void runOpMode() {
        RobotConfig robotConfig;
        DrawPathConfig drawPath;
        try {
            robotConfig = RobotConfigAssets.loadRobotConfig(hardwareMap.appContext.getAssets());
            drawPath = RobotConfigAssets.loadDrawPath(hardwareMap.appContext.getAssets());
        } catch (Exception e) {
            telemetry.addLine("LOI DOC CONFIG:");
            telemetry.addLine(e.getMessage());
            telemetry.update();
            return;
        }

        RobotHardware robot = new RobotHardware(hardwareMap, robotConfig);
        Localizer localizer = robot.localizer;
        MecanumDrive drive = new MecanumDrive(
                hardwareMap,
                "leftfront", "rightfront", "leftback", "rightback",
                localizer
        );
        drive.setHeadingHoldMode(MecanumDrive.HeadingHoldMode.ALWAYS);
        drive.setTolerance(
                robotConfig.motion.positionToleranceCm(),
                robotConfig.motion.headingToleranceDeg);
        drive.setPowerLimits(
                robotConfig.motion.drivePower,
                robotConfig.motion.headingHoldPower);

        PenServoManager pen = new PenServoManager(robot.penServo, robotConfig.pen);
        pen.penUp();

        localizer.setPose(
                drawPath.startPose.xCm(),
                drawPath.startPose.yCm(),
                drawPath.startPose.headingDeg);

        DrawPathFollower follower = new DrawPathFollower(
                drive, pen, drawPath, robotConfig.motion.settleCycles);

        telemetry.addLine("ROBOT VE TRANH");
        telemetry.addData("Path", drawPath.name);
        telemetry.addData("Waypoints", follower.getWaypointCount());
        telemetry.addData("Start (cm)", "X=%.1f Y=%.1f H=%.1f",
                drawPath.startPose.xCm(),
                drawPath.startPose.yCm(),
                drawPath.startPose.headingDeg);
        telemetry.addData("Drawing heading", "%.1f deg", drawPath.drawingHeadingDeg);
        telemetry.addLine("Dat robot dung startPose. BAM START de ve.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        try {
            while (opModeIsActive() && !follower.isDone()) {
                localizer.update();
                follower.tick();
                drive.update();

                DrawPathConfig.Waypoint current = follower.getCurrentWaypoint();
                telemetry.addData("Waypoint", "%d / %d",
                        Math.min(follower.getWaypointIndex() + 1, follower.getWaypointCount()),
                        follower.getWaypointCount());
                if (current != null) {
                    telemetry.addData("Target (cm)", "X=%.1f Y=%.1f", current.xCm(), current.yCm());
                    telemetry.addData("Pen", current.penDown ? "HA BUT" : "NHAC BUT");
                }
                telemetry.addData("Pose (cm)", "X=%.2f Y=%.2f", localizer.getX(), localizer.getY());
                telemetry.addData("Heading", "%.2f deg", localizer.getHeadingDeg());
                telemetry.addData("Heading error", "%.2f deg", drive.getLastHeadingErrorDeg());
                telemetry.addData("Follower", follower.getState());
                telemetry.update();
            }
        } finally {
            pen.penUp();
            drive.stop();
            robot.stopMotors();
        }
    }
}
