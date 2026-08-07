package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.core.LiftingSequenceConfig;
import org.firstinspires.ftc.teamcode.core.LiftingSequenceStateMachine;

public final class LiftingSequenceTest {
    static int passed;
    static final class FakeClock implements LiftingSequenceStateMachine.Clock {
        long now;
        public long nowNs() { return now; }
    }
    static final class FakeActuators implements LiftingSequenceStateMachine.Actuators {
        boolean stopped;
        int homes;
        LiftingSequenceConfig.ForkPose pose;
        public void stop() { stopped = true; }
        public boolean home() { return ++homes > 1; }
        public void setFork(LiftingSequenceConfig.ForkPose value) { pose = value; }
    }
    static void check(String name, boolean value) {
        if (!value) throw new AssertionError(name);
        passed++;
    }
    static void testConfig() {
        LiftingSequenceConfig.validate();
        check("targets bounded", LiftingSequenceConfig.ElevatorTarget.LIFT2.steps == 5625);
        check("release tolerance", LiftingSequenceConfig.RELEASE_BACK_OUT_CM == 20.0);
    }
    static void testTransitionsAndStop() {
        FakeClock clock = new FakeClock();
        FakeActuators hardware = new FakeActuators();
        LiftingSequenceStateMachine machine = new LiftingSequenceStateMachine(clock, hardware);
        machine.tick();
        check("start to homing", machine.getState() == LiftingSequenceStateMachine.State.HOMING);
        machine.tick();
        check("homing waits", machine.getState() == LiftingSequenceStateMachine.State.HOMING);
        machine.tick();
        check("homing to place", machine.getState() == LiftingSequenceStateMachine.State.PLACE);
        machine.tick();
        check("place to hold", machine.getState() == LiftingSequenceStateMachine.State.HOLD);
        check("hold pose", hardware.pose == LiftingSequenceConfig.ForkPose.PLACE);
        machine.requestStop();
        machine.tick();
        check("safe stop", machine.getState() == LiftingSequenceStateMachine.State.SAFE_STOP);
        check("actuators stopped", hardware.stopped);
    }
    static void testInvalidCamera() {
        FakeClock clock = new FakeClock();
        FakeActuators hardware = new FakeActuators();
        LiftingSequenceStateMachine machine = new LiftingSequenceStateMachine(clock, hardware);
        machine.rejectCamera(true, 0, 0, 0);
        check("stale camera", machine.getFailure() == LiftingSequenceStateMachine.FailureCode.CAMERA_STALE);
        check("stale stop", hardware.stopped);
    }
    public static void main(String[] args) {
        testConfig();
        testTransitionsAndStop();
        testInvalidCamera();
        System.out.println("RESULT: " + passed + " passed, 0 failed");
    }
}
