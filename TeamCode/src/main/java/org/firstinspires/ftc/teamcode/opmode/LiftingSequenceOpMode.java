package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.core.*;

/**
 * Shared autonomous gameplay loop. Subclasses fix alliance (BLUE or RED factory order).
 */
public abstract class LiftingSequenceOpMode extends LinearOpMode {

    protected abstract Alliance alliance();

    @Override
    public void runOpMode() throws InterruptedException {
        Alliance alliance = alliance();
        FieldBlueConfig field;
        LiftingSequenceConfig config;
        try {
            String fieldJson = RobotConfigAssets.readAsset(hardwareMap.appContext.getAssets(),
                    RobotConfigAssets.FIELD_BLUE_PATH);
            field = FieldBlueConfigLoader.load(fieldJson);
            config = RobotConfigAssets.load(hardwareMap.appContext.getAssets(), alliance);
        } catch (Exception e) {
            telemetry.addData("config", "SAFE_STOP: %s", e.getMessage());
            telemetry.update();
            return;
        }

        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("alliance", alliance.name());
            telemetry.addData("field", field.calibrated ? "calibrated" : "NOT CALIBRATED");
            if (!field.calibrated) {
                telemetry.addLine("Edit field-blue.json with measured poses, then set calibrated=true");
            }
            telemetry.update();
            idle();
        }
        if (!opModeIsActive()) return;
        if (!field.calibrated) {
            telemetry.addLine("SAFE_STOP: field not calibrated");
            telemetry.update();
            return;
        }

        RobotHardware robot = new RobotHardware(hardwareMap, config);
        MecanumDrive drive = new MecanumDrive(
                hardwareMap, "leftfront", "rightfront", "leftback", "rightback", robot.localizer);
        drive.setTolerance(config.positionToleranceCm, config.headingToleranceDeg);
        final PiCdcCameraTransport cameraTransport = Pi5CameraTransportFactory.createTransport(
                hardwareMap, config, System::nanoTime);
        final CameraAdapterManager cameras = new CameraAdapterManager(cameraTransport, config.sensorStaleNs);
        final Pi5GameplayCameraResult camera = new Pi5GameplayCameraResult(cameras, config);
        final PoseNavigation navigation = new PoseNavigation(drive, config);
        final BackOutTracker backOut = new BackOutTracker();

        LiftingSequenceStateMachine machine = new LiftingSequenceStateMachine(System::nanoTime,
                new LiftingSequenceStateMachine.Actuators() {
                    public void stop() {
                        robot.stopActuators();
                        drive.stop();
                    }

                    public boolean home() {
                        return robot.homeElevator();
                    }

                    public boolean elevatorAt(LiftingSequenceConfig.ElevatorTarget target) {
                        return robot.stepElevatorToward(config.elevatorSteps(target), System.nanoTime());
                    }

                    public void setFork(LiftingSequenceConfig.ForkPose pose) {
                        robot.servoLeft.setPosition(
                                pose == LiftingSequenceConfig.ForkPose.HOLD ? config.holdLeft : config.placeLeft);
                        robot.servoRight.setPosition(
                                pose == LiftingSequenceConfig.ForkPose.HOLD ? config.holdRight : config.placeRight);
                    }

                    public void drive(double forward, double strafe) {
                        navigation.clear();
                        drive.driveRobotCentric(forward, strafe, 0);
                    }

                    public void stopDrive() {
                        drive.stop();
                    }

                    public void markBackOutAnchor() {
                        backOut.mark(robot.localizer.getX(), robot.localizer.getY());
                    }

                    public double backOutDistanceCm() {
                        return backOut.distanceCm(robot.localizer.getX(), robot.localizer.getY());
                    }

                    public LiftingSequenceStateMachine.PoseReading pose() {
                        return new LiftingSequenceStateMachine.PoseReading(
                                robot.localizer.getX(),
                                robot.localizer.getY(),
                                robot.localizer.getHeadingDeg(),
                                System.nanoTime());
                    }

                    public boolean arrival(LiftingSequenceConfig.Pose target, long nowNs) {
                        return navigation.arrival(target);
                    }
                }, camera, config);

        try {
            while (opModeIsActive() && !isStopRequested()
                    && machine.getState() != LiftingSequenceStateMachine.State.SAFE_STOP) {
                long now = System.nanoTime();
                robot.localizer.update();
                drive.update();
                camera.update(now);
                machine.setIrState(!robot.leftIR.getState(), !robot.rightIR.getState());
                machine.tick();
                telemetry.addData("alliance", alliance.name());
                telemetry.addData("state", machine.getState());
                telemetry.addData("shelf/level", "%d/%d", machine.getShelf(), machine.getLevel());
                telemetry.addData("cycles", machine.getCompletedCycles());
                telemetry.addData("pose", "%.1f, %.1f, %.1f",
                        robot.localizer.getX(), robot.localizer.getY(), robot.localizer.getHeadingDeg());
                telemetry.addData("route", navigation.describe());
                telemetry.addData("blocks", "%s / %s", machine.getLeftType(), machine.getRightType());
                telemetry.addData("camera", "leftValid=%s leftFresh=%s rightValid=%s rightFresh=%s",
                        camera.leftValid(), camera.leftFresh(now),
                        camera.rightValid(), camera.rightFresh(now));
                telemetry.addData("config", "schema %d fingerprint %s", config.version, config.fingerprint);
                telemetry.addData("failure", machine.getFailure());
                telemetry.update();
                idle();
            }
        } finally {
            cameraTransport.close();
            drive.stop();
            robot.stopActuators();
        }
    }

    private static final class PoseNavigation {
        private final MecanumDrive drive;
        private final LiftingSequenceConfig config;
        private LiftingSequenceConfig.Pose activeTarget = null;

        PoseNavigation(MecanumDrive drive, LiftingSequenceConfig config) {
            this.drive = drive;
            this.config = config;
        }

        void clear() {
            activeTarget = null;
        }

        boolean arrival(LiftingSequenceConfig.Pose target) {
            if (target == null) return false;
            if (!samePose(activeTarget, target)) {
                drive.goToPosition(target.x, target.y, target.heading);
                activeTarget = target;
            }
            return drive.atTarget(config.positionToleranceCm, config.headingToleranceDeg);
        }

        String describe() {
            if (activeTarget == null) return "none";
            return String.format("%.1f, %.1f, %.1f", activeTarget.x, activeTarget.y, activeTarget.heading);
        }

        private static boolean samePose(LiftingSequenceConfig.Pose a, LiftingSequenceConfig.Pose b) {
            if (a == null || b == null) return a == b;
            return Double.compare(a.x, b.x) == 0
                    && Double.compare(a.y, b.y) == 0
                    && Double.compare(a.heading, b.heading) == 0;
        }
    }

    private static final class BackOutTracker {
        private boolean marked = false;
        private double anchorX;
        private double anchorY;

        void mark(double x, double y) {
            anchorX = x;
            anchorY = y;
            marked = true;
        }

        double distanceCm(double x, double y) {
            if (!marked) return 0.0;
            return Math.hypot(x - anchorX, y - anchorY);
        }
    }
}
