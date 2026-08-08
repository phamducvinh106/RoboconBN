package org.firstinspires.ftc.teamcode.test;

import java.nio.file.Files;
import java.nio.file.Paths;
import org.firstinspires.ftc.teamcode.core.Alliance;
import org.firstinspires.ftc.teamcode.core.FieldBlueConfig;
import org.firstinspires.ftc.teamcode.core.FieldBlueConfigLoader;
import org.firstinspires.ftc.teamcode.core.LiftingSequenceConfig;
import org.firstinspires.ftc.teamcode.core.LiftingSequenceConfigLoader;
import org.firstinspires.ftc.teamcode.core.LiftingSequenceStateMachine;

public final class LiftingSequenceTest {
    static int passed;
    static void check(String name, boolean ok) {
        if (!ok) throw new AssertionError(name);
        passed++;
    }

    static final class C implements LiftingSequenceStateMachine.Clock {
        long n;
        public long nowNs() {
            return n;
        }
    }

    static class H implements LiftingSequenceStateMachine.Actuators {
        boolean stop;
        int homes;
        java.util.List<String> events = new java.util.ArrayList<>();
        double anchorX = 0;
        double anchorY = 0;
        double poseX = 0;
        double poseY = 0;
        int goToCalls = 0;
        LiftingSequenceConfig.Pose lastTarget = null;

        public void stop() {
            stop = true;
        }

        public boolean home() {
            events.add("HOME");
            return ++homes > 1;
        }

        public boolean elevatorAt(LiftingSequenceConfig.ElevatorTarget target) {
            events.add("ELEVATOR:" + target);
            return true;
        }

        public void setFork(LiftingSequenceConfig.ForkPose pose) {
            events.add(pose.name());
        }

        public void drive(double forward, double strafe) {}

        public void stopDrive() {}

        public void markBackOutAnchor() {
            events.add("ANCHOR");
            anchorX = poseX;
            anchorY = poseY;
        }

        public double backOutDistanceCm() {
            events.add("BACKOUT");
            return Math.hypot(poseX - anchorX, poseY - anchorY);
        }

        public LiftingSequenceStateMachine.PoseReading pose() {
            return new LiftingSequenceStateMachine.PoseReading(poseX, poseY, 0, 0);
        }

        public boolean arrival(LiftingSequenceConfig.Pose target, long nowNs) {
            if (lastTarget == null
                    || lastTarget.x != target.x
                    || lastTarget.y != target.y
                    || lastTarget.heading != target.heading) {
                goToCalls++;
                lastTarget = target;
                return false;
            }
            return true;
        }
    }

    static final class Cam implements LiftingSequenceStateMachine.CameraResult {
        boolean leftValid = true;
        boolean rightValid = true;
        boolean leftFresh = true;
        boolean rightFresh = true;
        String leftType = "01";
        String rightType = "02";
        double dxPx = 0;
        int stableTicks;

        void tickStable() {
            if (leftValid && leftFresh && Math.abs(dxPx) <= 8) stableTicks++;
            else stableTicks = 0;
        }

        public boolean leftFresh(long nowNs) {
            return leftFresh;
        }

        public boolean rightFresh(long nowNs) {
            return rightFresh;
        }

        public boolean leftValid() {
            return leftValid;
        }

        public boolean rightValid() {
            return rightValid;
        }

        public boolean stableLeftTarget() {
            return stableTicks >= 3;
        }

        public double leftDxPx() {
            return dxPx;
        }

        public String leftBlockType() {
            return leftType;
        }

        public String rightBlockType() {
            return rightType;
        }
    }

    static LiftingSequenceConfig config() throws Exception {
        String robot = new String(
                Files.readAllBytes(Paths.get("TeamCode/src/main/assets/robot-config.json")),
                java.nio.charset.StandardCharsets.UTF_8);
        String field = new String(
                Files.readAllBytes(Paths.get("TeamCode/src/main/assets/field-blue.json")),
                java.nio.charset.StandardCharsets.UTF_8);
        return LiftingSequenceConfigLoader.load(robot)
                .withField(FieldBlueConfigLoader.load(field), Alliance.BLUE);
    }

    static void advanceToScan(LiftingSequenceStateMachine machine, C clock) {
        for (int i = 0; i < 30 && machine.getState() != LiftingSequenceStateMachine.State.SCAN_RIGHT; i++) {
            machine.tick();
        }
    }

    static void testSafety() {
        C clock = new C();
        H actuators = new H();
        LiftingSequenceStateMachine machine = new LiftingSequenceStateMachine(clock, actuators);
        machine.requestStop();
        machine.tick();
        check("safe stop", machine.getState() == LiftingSequenceStateMachine.State.SAFE_STOP && actuators.stop);
    }

    static void testInvalidCamera() throws Exception {
        C clock = new C();
        H actuators = new H();
        Cam cam = new Cam();
        LiftingSequenceStateMachine machine = new LiftingSequenceStateMachine(clock, actuators, cam, config());
        advanceToScan(machine, clock);
        check("reached scan", machine.getState() == LiftingSequenceStateMachine.State.SCAN_RIGHT);
        cam.leftValid = false;
        machine.tick();
        check("invalid camera holds", machine.getState() == LiftingSequenceStateMachine.State.SCAN_RIGHT);
    }

    static void testScanLatchesBlockTypes() throws Exception {
        C clock = new C();
        H actuators = new H();
        Cam cam = new Cam();
        cam.leftType = "03";
        cam.rightType = "04";
        LiftingSequenceStateMachine machine = new LiftingSequenceStateMachine(clock, actuators, cam, config());
        advanceToScan(machine, clock);
        cam.tickStable();
        machine.tick();
        check("scan advances", machine.getState() == LiftingSequenceStateMachine.State.CENTER_LEFT_SLOW);
        check("left type", "03".equals(machine.getLeftType()));
        check("right type", "04".equals(machine.getRightType()));
    }

    static void testCenterStable() throws Exception {
        C clock = new C();
        H actuators = new H();
        Cam cam = new Cam();
        LiftingSequenceStateMachine machine = new LiftingSequenceStateMachine(clock, actuators, cam, config());
        advanceToScan(machine, clock);
        cam.tickStable();
        machine.tick();
        check("in center", machine.getState() == LiftingSequenceStateMachine.State.CENTER_LEFT_SLOW);
        cam.tickStable();
        machine.tick();
        check("not stable yet", machine.getState() == LiftingSequenceStateMachine.State.CENTER_LEFT_SLOW);
        cam.tickStable();
        machine.tick();
        check("stable center", machine.getState() == LiftingSequenceStateMachine.State.APPROACH_IR_SLOW);
    }

    static void testSerial() throws Exception {
        C clock = new C();
        H actuators = new H();
        Cam cam = new Cam();
        LiftingSequenceStateMachine machine = new LiftingSequenceStateMachine(clock, actuators, cam, config());
        machine.setIrState(true, true);
        for (int i = 0; i < 140 && machine.getCompletedCycles() == 0; i++) {
            cam.tickStable();
            clock.n += LiftingSequenceConfig.IR_DEBOUNCE_NS;
            actuators.poseX += 5;
            actuators.poseY += 5;
            machine.tick();
        }
        check("first cycle", machine.getCompletedCycles() == 1);
        check("right relift", actuators.events.lastIndexOf("ELEVATOR:LIFT1")
                > actuators.events.indexOf("ANCHOR"));
    }

    static void testBackOutAnchor() throws Exception {
        C clock = new C();
        H actuators = new H();
        Cam cam = new Cam();
        LiftingSequenceStateMachine machine = new LiftingSequenceStateMachine(clock, actuators, cam, config());
        machine.setIrState(true, true);
        for (int i = 0; i < 120
                && machine.getState() != LiftingSequenceStateMachine.State.BACK_OUT_AFTER_LEFT_RELEASE_20CM; i++) {
            cam.tickStable();
            clock.n += LiftingSequenceConfig.IR_DEBOUNCE_NS;
            machine.tick();
        }
        check("backout state", machine.getState()
                == LiftingSequenceStateMachine.State.BACK_OUT_AFTER_LEFT_RELEASE_20CM);
        actuators.poseX = 25;
        actuators.poseY = 0;
        machine.tick();
        check("backout distance", actuators.backOutDistanceCm() >= 20);
    }

    static void testStateTimeout() throws Exception {
        C clock = new C();
        H actuators = new H() {
            public boolean arrival(LiftingSequenceConfig.Pose target, long nowNs) {
                return false;
            }
        };
        LiftingSequenceStateMachine machine = new LiftingSequenceStateMachine(clock, actuators, null, config());
        machine.tick();
        machine.tick();
        clock.n += config().stateTimeoutNs + 1;
        machine.tick();
        check("state timeout", machine.getState() == LiftingSequenceStateMachine.State.SAFE_STOP);
        check("timeout failure", machine.getFailure() == LiftingSequenceStateMachine.FailureCode.NO_PROGRESS);
    }

    static void testGoToOncePerTarget() throws Exception {
        H actuators = new H();
        LiftingSequenceConfig.Pose target = config().placeAtFactory;
        actuators.arrival(target, 0);
        int afterFirst = actuators.goToCalls;
        actuators.arrival(target, 0);
        check("single goTo per pose", actuators.goToCalls == afterFirst && afterFirst == 1);
    }

    public static void main(String[] args) throws Exception {
        LiftingSequenceConfig.validate();
        testSafety();
        testInvalidCamera();
        testScanLatchesBlockTypes();
        testCenterStable();
        testSerial();
        testBackOutAnchor();
        testStateTimeout();
        testGoToOncePerTarget();
        System.out.println("RESULT: " + passed + " passed, 0 failed");
    }
}
