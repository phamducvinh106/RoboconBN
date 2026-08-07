package org.firstinspires.ftc.teamcode.core;

public final class LiftingSequenceStateMachine {
    public enum State { START, HOMING, PLACE, HOLD, SAFE_STOP }
    public enum FailureCode { NONE, STOP_REQUESTED, TIMEOUT, CAMERA_STALE, INVALID_POSE }
    public interface Clock { long nowNs(); }
    public interface Actuators {
        void stop();
        boolean home();
        void setFork(LiftingSequenceConfig.ForkPose pose);
    }

    private final Clock clock;
    private final Actuators actuators;
    private State state = State.START;
    private FailureCode failure = FailureCode.NONE;
    private long stateStartedNs;
    private boolean stopRequested;
    private boolean active = true;

    public LiftingSequenceStateMachine(Clock clock, Actuators actuators) {
        if (clock == null || actuators == null) throw new NullPointerException();
        LiftingSequenceConfig.validate();
        this.clock = clock;
        this.actuators = actuators;
        stateStartedNs = clock.nowNs();
    }

    public void requestStop() { stopRequested = true; }
    public void setActive(boolean active) { this.active = active; }
    public State getState() { return state; }
    public FailureCode getFailure() { return failure; }
    public long elapsedNs() { return clock.nowNs() - stateStartedNs; }

    public void tick() {
        if (state == State.SAFE_STOP) return;
        if (stopRequested || !active) { safeStop(FailureCode.STOP_REQUESTED); return; }
        if (elapsedNs() > LiftingSequenceConfig.STATE_TIMEOUT_NS) { safeStop(FailureCode.TIMEOUT); return; }
        switch (state) {
            case START: transition(State.HOMING); break;
            case HOMING:
                if (actuators.home()) transition(State.PLACE);
                break;
            case PLACE: actuators.setFork(LiftingSequenceConfig.ForkPose.PLACE); transition(State.HOLD); break;
            case HOLD: actuators.setFork(LiftingSequenceConfig.ForkPose.HOLD); break;
            default: break;
        }
    }

    public void rejectCamera(boolean stale, double x, double y, double heading) {
        if (stale || !Double.isFinite(x) || !Double.isFinite(y) || !Double.isFinite(heading)) {
            safeStop(stale ? FailureCode.CAMERA_STALE : FailureCode.INVALID_POSE);
        }
    }

    public void safeStop(FailureCode reason) {
        if (state == State.SAFE_STOP) return;
        failure = reason;
        state = State.SAFE_STOP;
        actuators.stop();
    }

    private void transition(State next) {
        state = next;
        stateStartedNs = clock.nowNs();
    }
}
