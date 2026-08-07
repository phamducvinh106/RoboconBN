package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.core.*;

@Autonomous(name = "Lifting Sequence", group = "Autonomous")
public final class LiftingSequenceOpMode extends LinearOpMode {
    @Override public void runOpMode() throws InterruptedException {
        LiftingSequenceConfig config;
        try (java.io.InputStream input = hardwareMap.appContext.getAssets().open("phase2-lifting-config.json")) {
            java.io.ByteArrayOutputStream bytes = new java.io.ByteArrayOutputStream();
            byte[] buffer = new byte[1024];
            int count;
            while ((count = input.read(buffer)) != -1) bytes.write(buffer, 0, count);
            config = LiftingSequenceConfigLoader.load(new String(bytes.toByteArray(), java.nio.charset.StandardCharsets.UTF_8));
        } catch (java.io.IOException | IllegalArgumentException e) {
            telemetry.addData("config", "SAFE_STOP: %s", e.getMessage());
            telemetry.update();
            return;
        }
        RobotHardware robot = new RobotHardware(hardwareMap);
        MecanumDrive drive = new MecanumDrive(hardwareMap, "leftfront", "rightfront", "leftback", "rightback", robot.localizer);
        final CameraAdapterManager cameras = new CameraAdapterManager(
                Pi5CameraTransportFactory.createWithHubPolling(hardwareMap, config, System::nanoTime, this::idle),
                config.sensorStaleNs);
        LiftingSequenceStateMachine machine = new LiftingSequenceStateMachine(System::nanoTime, new LiftingSequenceStateMachine.Actuators() {
            public void stop() { robot.stopActuators(); drive.stop(); }
            public boolean home() { return robot.homeElevator(); }
            public boolean elevatorAt(LiftingSequenceConfig.ElevatorTarget t) { return robot.stepElevatorToward(t.steps, System.nanoTime()); }
            public void setFork(LiftingSequenceConfig.ForkPose p) { robot.servoLeft.setPosition(p == LiftingSequenceConfig.ForkPose.HOLD ? LiftingSequenceConfig.HOLD_LEFT : LiftingSequenceConfig.PLACE_LEFT); robot.servoRight.setPosition(p == LiftingSequenceConfig.ForkPose.HOLD ? LiftingSequenceConfig.HOLD_RIGHT : LiftingSequenceConfig.PLACE_RIGHT); }
            public void drive(double f,double s) { drive.driveRobotCentric(f,s,0); }
            public void stopDrive() { drive.stop(); }
            public boolean released() { return robot.cargoReady(); }
            public double backOutDistanceCm() { return Math.abs(robot.localizer.getX()); }
            public LiftingSequenceStateMachine.PoseReading pose() { return new LiftingSequenceStateMachine.PoseReading(robot.localizer.getX(),robot.localizer.getY(),robot.localizer.getHeadingDeg(),System.nanoTime()); }
            public boolean arrival(LiftingSequenceConfig.Pose target,long nowNs) { LiftingSequenceStateMachine.PoseReading p=pose(); return p.valid&&Math.abs(p.x-target.x)<=LiftingSequenceConfig.POSITION_TOLERANCE_CM&&Math.abs(p.y-target.y)<=LiftingSequenceConfig.POSITION_TOLERANCE_CM&&Math.abs(p.heading-target.heading)<=LiftingSequenceConfig.HEADING_TOLERANCE_DEG; }
        }, new LiftingSequenceStateMachine.CameraResult() { public boolean fresh(long n){return cameras.movementAuthorized(CameraChannel.WEBCAM1,n);} public boolean valid(){return cameras.reading(CameraChannel.WEBCAM1).valid;} public boolean stableLeftTarget(){return valid();} public double leftDxPx(){return cameras.reading(CameraChannel.WEBCAM1).dxPx;} }, config);
        waitForStart();
        try { while (opModeIsActive() && !isStopRequested() && machine.getState()!=LiftingSequenceStateMachine.State.SAFE_STOP) { robot.localizer.update(); drive.update(); machine.setIrState(!robot.leftIR.getState(),!robot.rightIR.getState()); machine.tick(); telemetry.addData("state",machine.getState()); telemetry.addData("shelf/level","%d/%d",machine.getShelf(),machine.getLevel()); telemetry.addData("cycles",machine.getCompletedCycles()); telemetry.addData("pose","%.1f, %.1f, %.1f",robot.localizer.getX(),robot.localizer.getY(),robot.localizer.getHeadingDeg()); telemetry.addData("config","schema %d fingerprint %s",config.version,config.fingerprint); telemetry.addData("failure",machine.getFailure()); telemetry.update(); idle(); } } finally { drive.stop(); robot.stopActuators(); }
    }
}
