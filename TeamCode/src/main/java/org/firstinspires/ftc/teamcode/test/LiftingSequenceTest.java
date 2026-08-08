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
        double poseHeading = 0;
        int goToCalls = 0;
        int strafeCommands = 0;
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

        public void drive(double forward, double strafe) {
            if (strafe != 0) strafeCommands++;
        }

        public void stopDrive() {}

        public void resetNavigation() {
            lastTarget = null;
        }

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
            return new LiftingSequenceStateMachine.PoseReading(poseX, poseY, poseHeading, 0);
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
        for (int i = 0; i < 40 && machine.getState() != LiftingSequenceStateMachine.State.SCAN_RIGHT; i++) {
            machine.tick();
        }
    }

    static void tickNavigationSettle(LiftingSequenceStateMachine machine, int settleCycles) {
        for (int i = 0; i < settleCycles; i++) {
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

    static void assertScanAdvances(String name, Cam cam, String left, String right) throws Exception {
        C clock = new C();
        H actuators = new H();
        LiftingSequenceStateMachine machine = new LiftingSequenceStateMachine(clock, actuators, cam, config());
        advanceToScan(machine, clock);
        machine.tick();
        check(name + " advances", machine.getState() == LiftingSequenceStateMachine.State.APPROACH_IR_SLOW);
        check(name + " left", left.equals(machine.getLeftType()));
        check(name + " right", right.equals(machine.getRightType()));
    }

    static void testClassificationUsesDefaultsWhenCameraMissing() throws Exception {
        Cam leftInvalid = new Cam();
        leftInvalid.leftValid = false;
        assertScanAdvances("left invalid", leftInvalid, "01", "02");

        Cam rightInvalid = new Cam();
        rightInvalid.rightValid = false;
        assertScanAdvances("right invalid", rightInvalid, "01", "02");

        Cam leftStale = new Cam();
        leftStale.leftFresh = false;
        assertScanAdvances("left stale", leftStale, "01", "02");

        Cam rightStale = new Cam();
        rightStale.rightFresh = false;
        assertScanAdvances("right stale", rightStale, "01", "02");

        Cam leftUnknown = new Cam();
        leftUnknown.leftType = "unknown";
        assertScanAdvances("left unknown", leftUnknown, "01", "02");

        Cam rightUnknown = new Cam();
        rightUnknown.rightType = "unknown";
        assertScanAdvances("right unknown", rightUnknown, "01", "02");

        Cam unknownFactory = new Cam();
        unknownFactory.leftType = "99";
        unknownFactory.rightType = "02";
        assertScanAdvances("unknown factory left", unknownFactory, "01", "02");
    }

    static void testScanAdvancesWithoutCamera() throws Exception {
        C clock = new C();
        H actuators = new H();
        Cam cam = new Cam();
        cam.leftFresh = false;
        cam.rightFresh = false;
        LiftingSequenceStateMachine machine = new LiftingSequenceStateMachine(clock, actuators, cam, config());
        advanceToScan(machine, clock);
        check("reached scan", machine.getState() == LiftingSequenceStateMachine.State.SCAN_RIGHT);
        machine.tick();
        check("advances with defaults", machine.getState() == LiftingSequenceStateMachine.State.APPROACH_IR_SLOW);
        check("default left", "01".equals(machine.getLeftType()));
        check("default right", "02".equals(machine.getRightType()));
    }

    static void testScanLatchesBlockTypes() throws Exception {
        C clock = new C();
        H actuators = new H();
        Cam cam = new Cam();
        cam.leftType = "03";
        cam.rightType = "04";
        LiftingSequenceStateMachine machine = new LiftingSequenceStateMachine(clock, actuators, cam, config());
        advanceToScan(machine, clock);
        machine.tick();
        check("scan advances directly", machine.getState() == LiftingSequenceStateMachine.State.APPROACH_IR_SLOW);
        check("left type", "03".equals(machine.getLeftType()));
        check("right type", "04".equals(machine.getRightType()));
        check("classification does not strafe", actuators.strafeCommands == 0);
    }

    static void testSerial() throws Exception {
        C clock = new C();
        H actuators = new H();
        Cam cam = new Cam();
        LiftingSequenceStateMachine machine = new LiftingSequenceStateMachine(clock, actuators, cam, config());
        machine.setIrState(true, true);
        for (int i = 0; i < 200 && machine.getCompletedCycles() == 0; i++) {
            clock.n += LiftingSequenceConfig.IR_DEBOUNCE_NS;
            actuators.poseX += 5;
            actuators.poseY += 5;
            machine.tick();
        }
        check("first cycle", machine.getCompletedCycles() == 1);
        check("right relift", actuators.events.lastIndexOf("ELEVATOR:LIFT1")
                > actuators.events.indexOf("ANCHOR"));
    }

    static void testShelfBackOutUsesCurrentYAndPlus20X() throws Exception {
        C clock = new C();
        H actuators = new H();
        Cam cam = new Cam();
        LiftingSequenceConfig cfg = config();
        LiftingSequenceStateMachine machine = new LiftingSequenceStateMachine(clock, actuators, cam, cfg);
        actuators.poseX = 40;
        actuators.poseY = 167.65;
        actuators.poseHeading = 90;
        machine.setIrState(true, true);
        for (int i = 0; i < 80
                && machine.getState() != LiftingSequenceStateMachine.State.BACK_OUT_FROM_SHELF; i++) {
            clock.n += LiftingSequenceConfig.IR_DEBOUNCE_NS;
            machine.tick();
        }
        check("reached shelf backout", machine.getState()
                == LiftingSequenceStateMachine.State.BACK_OUT_FROM_SHELF);
        machine.tick();
        check("shelf backout x +20", Math.abs(actuators.lastTarget.x - 60.0) < 1e-9);
        check("shelf backout keeps y", Math.abs(actuators.lastTarget.y - 167.65) < 1e-9);
        check("shelf backout keeps heading", Math.abs(actuators.lastTarget.heading - 90.0) < 1e-9);
        check("shelf backout ignores fixed retreat y", actuators.lastTarget.y != cfg.retreat.y);
    }

    static void testBackOutAnchor() throws Exception {
        C clock = new C();
        H actuators = new H();
        Cam cam = new Cam();
        LiftingSequenceConfig cfg = config();
        LiftingSequenceStateMachine machine = new LiftingSequenceStateMachine(clock, actuators, cam, cfg);
        machine.setIrState(true, true);
        for (int i = 0; i < 120
                && machine.getState() != LiftingSequenceStateMachine.State.BACK_OUT_AFTER_LEFT_RELEASE_20CM; i++) {
            clock.n += LiftingSequenceConfig.IR_DEBOUNCE_NS;
            machine.tick();
        }
        check("backout state", machine.getState()
                == LiftingSequenceStateMachine.State.BACK_OUT_AFTER_LEFT_RELEASE_20CM);
        int goToAtEntry = actuators.goToCalls;
        machine.tick();
        check("retreat issued immediately", actuators.goToCalls > goToAtEntry);
        check("retreat target", actuators.lastTarget == cfg.retreat);
        actuators.poseX = 25;
        actuators.poseY = 0;
        tickNavigationSettle(machine, cfg.settleCycles);
        check("backout distance", actuators.backOutDistanceCm() >= 20);
    }

    static void testBackOutIssuesRetreatBeforeDistance() throws Exception {
        C clock = new C();
        H actuators = new H();
        Cam cam = new Cam();
        LiftingSequenceConfig cfg = config();
        LiftingSequenceStateMachine machine = new LiftingSequenceStateMachine(clock, actuators, cam, cfg);
        machine.setIrState(true, true);
        for (int i = 0; i < 160
                && machine.getState() != LiftingSequenceStateMachine.State.BACK_OUT_AFTER_LEFT_RELEASE_20CM; i++) {
            clock.n += LiftingSequenceConfig.IR_DEBOUNCE_NS;
            machine.tick();
        }
        check("in backout state", machine.getState()
                == LiftingSequenceStateMachine.State.BACK_OUT_AFTER_LEFT_RELEASE_20CM);
        actuators.poseX = 0;
        actuators.poseY = 0;
        machine.tick();
        check("retreat command on tick 1", actuators.lastTarget == cfg.retreat);
        check("still waiting for distance", machine.getState()
                == LiftingSequenceStateMachine.State.BACK_OUT_AFTER_LEFT_RELEASE_20CM);
    }

    static void testArrivalRequiresSettleCycles() throws Exception {
        C clock = new C();
        H actuators = new H();
        LiftingSequenceConfig cfg = config();
        actuators.poseX = cfg.placeAtFactory.x;
        actuators.poseY = cfg.placeAtFactory.y;
        LiftingSequenceStateMachine machine = new LiftingSequenceStateMachine(clock, actuators, null, cfg);
        for (int i = 0; i < 12
                && machine.getState() != LiftingSequenceStateMachine.State.PLACE_AT_FACTORY; i++) {
            machine.tick();
        }
        check("at place", machine.getState() == LiftingSequenceStateMachine.State.PLACE_AT_FACTORY);
        machine.tick();
        check("needs settle 1", machine.getState() == LiftingSequenceStateMachine.State.PLACE_AT_FACTORY);
        machine.tick();
        check("needs settle 2", machine.getState() == LiftingSequenceStateMachine.State.PLACE_AT_FACTORY);
        machine.tick();
        check("needs settle 3", machine.getState() == LiftingSequenceStateMachine.State.PLACE_AT_FACTORY);
        machine.tick();
        check("settled advances", machine.getState() == LiftingSequenceStateMachine.State.MOVE_TO_SHELF);
    }

    static void testNavigationTimesOutSafeStop() throws Exception {
        C clock = new C();
        H actuators = new H() {
            public boolean arrival(LiftingSequenceConfig.Pose target, long nowNs) {
                return false;
            }
        };
        LiftingSequenceConfig cfg = config();
        LiftingSequenceStateMachine machine = new LiftingSequenceStateMachine(clock, actuators, null, cfg);
        for (int i = 0; i < 12
                && machine.getState() != LiftingSequenceStateMachine.State.PLACE_AT_FACTORY; i++) {
            machine.tick();
        }
        check("at navigation state", machine.getState()
                == LiftingSequenceStateMachine.State.PLACE_AT_FACTORY);
        for (int i = 0; i <= cfg.maxRetries; i++) {
            clock.n += 30_000_000_000L;
            machine.tick();
            if (machine.getState() == LiftingSequenceStateMachine.State.SAFE_STOP) break;
        }
        check("timeout safe stop", machine.getState() == LiftingSequenceStateMachine.State.SAFE_STOP);
        check("no progress failure", machine.getFailure()
                == LiftingSequenceStateMachine.FailureCode.NO_PROGRESS);
    }

    static void testNoProgressSafeStop() throws Exception {
        C clock = new C();
        H actuators = new H() {
            public boolean arrival(LiftingSequenceConfig.Pose target, long nowNs) {
                return false;
            }
        };
        LiftingSequenceConfig cfg = config();
        LiftingSequenceStateMachine machine = new LiftingSequenceStateMachine(clock, actuators, null, cfg);
        for (int i = 0; i < 12
                && machine.getState() != LiftingSequenceStateMachine.State.PLACE_AT_FACTORY; i++) {
            machine.tick();
        }
        for (int i = 0; i <= cfg.maxRetries + 1; i++) {
            clock.n += 3_000_000_000L;
            machine.tick();
            if (machine.getState() == LiftingSequenceStateMachine.State.SAFE_STOP) break;
        }
        check("stuck safe stop", machine.getState() == LiftingSequenceStateMachine.State.SAFE_STOP);
        check("no progress code", machine.getFailure()
                == LiftingSequenceStateMachine.FailureCode.NO_PROGRESS);
    }

    static void testGoToOncePerTarget() throws Exception {
        H actuators = new H();
        LiftingSequenceConfig.Pose target = config().placeAtFactory;
        actuators.arrival(target, 0);
        int afterFirst = actuators.goToCalls;
        actuators.arrival(target, 0);
        check("single goTo per pose", actuators.goToCalls == afterFirst && afterFirst == 1);
    }

    static void testStartAtPlaceAtFactorySkipsInitialDrive() throws Exception {
        C clock = new C();
        H actuators = new H();
        LiftingSequenceConfig cfg = config();
        actuators.poseX = cfg.placeAtFactory.x;
        actuators.poseY = cfg.placeAtFactory.y;
        LiftingSequenceStateMachine machine = new LiftingSequenceStateMachine(clock, actuators, null, cfg);
        for (int i = 0; i < 12
                && machine.getState() != LiftingSequenceStateMachine.State.PLACE_AT_FACTORY; i++) {
            machine.tick();
        }
        check("reached place state", machine.getState() == LiftingSequenceStateMachine.State.PLACE_AT_FACTORY);
        check("no strafe before place arrival", actuators.strafeCommands == 0);
        machine.tick();
        check("still no strafe on place goTo", actuators.strafeCommands == 0);
        tickNavigationSettle(machine, cfg.settleCycles);
        check("advances to shelf", machine.getState() == LiftingSequenceStateMachine.State.MOVE_TO_SHELF);
        machine.tick();
        check("next target is fac1", actuators.lastTarget == cfg.shelfFor(1));
    }

    static void testHomingBlocksDrive() throws Exception {
        H actuators = new H() {
            int homes;

            public boolean home() {
                return ++homes >= 3;
            }
        };
        LiftingSequenceStateMachine machine = new LiftingSequenceStateMachine(new C(), actuators, null, config());
        machine.tick();
        machine.tick();
        check("stays homing", machine.getState() == LiftingSequenceStateMachine.State.HOMING);
        check("no drive during homing", actuators.goToCalls == 0);
        machine.tick();
        check("still homing", machine.getState() == LiftingSequenceStateMachine.State.HOMING);
        machine.tick();
        check("left homing", machine.getState() != LiftingSequenceStateMachine.State.HOMING);
    }

    static void testScanNormalizesSingleDigitType() throws Exception {
        Cam cam = new Cam();
        cam.leftType = "2";
        cam.rightType = "4";
        assertScanAdvances("normalized", cam, "02", "04");
    }

    static void testApproachUsesIrOnly() throws Exception {
        C clock = new C();
        H actuators = new H();
        Cam cam = new Cam();
        LiftingSequenceStateMachine machine = new LiftingSequenceStateMachine(clock, actuators, cam, config());
        advanceToScan(machine, clock);
        machine.tick();
        check("entered approach", machine.getState() == LiftingSequenceStateMachine.State.APPROACH_IR_SLOW);
        cam.leftValid = false;
        cam.rightValid = false;
        machine.setIrState(false, false);
        machine.tick();
        check("still approaching", machine.getState() == LiftingSequenceStateMachine.State.APPROACH_IR_SLOW);
        machine.setIrState(true, true);
        machine.tick();
        check("ir confirms", machine.getState() == LiftingSequenceStateMachine.State.CONFIRM_IR);
    }

    public static void main(String[] args) throws Exception {
        LiftingSequenceConfig.validate();
        testSafety();
        testClassificationUsesDefaultsWhenCameraMissing();
        testScanAdvancesWithoutCamera();
        testScanLatchesBlockTypes();
        testScanNormalizesSingleDigitType();
        testSerial();
        testShelfBackOutUsesCurrentYAndPlus20X();
        testBackOutAnchor();
        testBackOutIssuesRetreatBeforeDistance();
        testArrivalRequiresSettleCycles();
        testNavigationTimesOutSafeStop();
        testNoProgressSafeStop();
        testGoToOncePerTarget();
        testStartAtPlaceAtFactorySkipsInitialDrive();
        testHomingBlocksDrive();
        testApproachUsesIrOnly();
        System.out.println("RESULT: " + passed + " passed, 0 failed");
    }
}
