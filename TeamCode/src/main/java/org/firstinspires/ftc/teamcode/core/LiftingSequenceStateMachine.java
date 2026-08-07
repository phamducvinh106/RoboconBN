package org.firstinspires.ftc.teamcode.core;

public final class LiftingSequenceStateMachine {
    public enum State { START, HOMING, SET_PLACE, PLACE_AT_FACTORY, MOVE_TO_SHELF, SELECT_LEVEL, READY1_PUSH, READY1, READY2,
        SCAN_RIGHT, SCAN_LEFT, CENTER_LEFT_SLOW, APPROACH_IR_SLOW, CONFIRM_IR, SAVE_SHELF_POSE,
        CALIBRATE_SHELF_COORDINATE, LIFT1, LIFT2, BACK_OUT_FROM_SHELF, HOLD, CYCLE_COMPLETE, SAFE_STOP }
    public enum FailureCode { NONE, STOP_REQUESTED, TIMEOUT, CAMERA_STALE, CAMERA_CLASSIFICATION_FAILED, CENTERING_TIMEOUT,
        IR_TIMEOUT, IR_PARTIAL, POSE_INVALID, ELEVATOR_TIMEOUT }
    public interface Clock { long nowNs(); }
    public interface Actuators { void stop(); boolean home(); boolean elevatorAt(LiftingSequenceConfig.ElevatorTarget target); void setFork(LiftingSequenceConfig.ForkPose pose); void drive(double forward, double strafe); void stopDrive(); }
    public interface CameraResult { boolean fresh(long nowNs); boolean classificationValid(); boolean stableLeftTarget(); double leftDxPx(); }
    public interface PoseProvider { void update(); double x(); double y(); double headingDeg(); }
    public static final class ShelfPose { public final int shelf, level; public final double x, y, heading; public final long timestampNs;
        ShelfPose(int shelf, int level, double x, double y, double heading, long timestampNs) { this.shelf=shelf; this.level=level; this.x=x; this.y=y; this.heading=heading; this.timestampNs=timestampNs; } }

    private final Clock clock; private final Actuators actuators; private final CameraResult camera; private final PoseProvider pose;
    private State state = State.START; private FailureCode failure = FailureCode.NONE; private long stateStartedNs;
    private boolean stopRequested, active = true, leftIr, rightIr; private long bothIrSinceNs; private int retries;
    private int shelf = 1, level = 1, completedCycles; private ShelfPose shelfPose;
    private static final double CENTER_DEADBAND_PX = 8.0;

    public LiftingSequenceStateMachine(Clock clock, Actuators actuators) { this(clock, actuators, null, null); }
    public LiftingSequenceStateMachine(Clock clock, Actuators actuators, CameraResult camera, PoseProvider pose) {
        if (clock == null || actuators == null) throw new NullPointerException(); LiftingSequenceConfig.validate();
        this.clock=clock; this.actuators=actuators; this.camera=camera; this.pose=pose; stateStartedNs=clock.nowNs();
    }
    public void requestStop() { stopRequested=true; } public void setActive(boolean value) { active=value; }
    public State getState() { return state; } public FailureCode getFailure() { return failure; } public long elapsedNs() { return clock.nowNs()-stateStartedNs; }
    public int getShelf() { return shelf; } public int getLevel() { return level; } public int getCompletedCycles() { return completedCycles; }
    public int getRetries() { return retries; } public ShelfPose getShelfPose() { return shelfPose; }
    public void setIrState(boolean left, boolean right) { leftIr=left; rightIr=right; }

    public void tick() {
        if (state == State.SAFE_STOP) return;
        long now=clock.nowNs();
        if (stopRequested || !active) { safeStop(FailureCode.STOP_REQUESTED); return; }
        if (elapsedNs() > LiftingSequenceConfig.STATE_TIMEOUT_NS) { safeStop(FailureCode.TIMEOUT); return; }
        switch (state) {
            case START: transition(State.HOMING); break;
            case HOMING: if (actuators.home()) transition(State.SET_PLACE); break;
            case SET_PLACE: actuators.setFork(LiftingSequenceConfig.ForkPose.PLACE); transition(State.PLACE_AT_FACTORY); break;
            case PLACE_AT_FACTORY: transition(State.MOVE_TO_SHELF); break;
            case MOVE_TO_SHELF: transition(State.SELECT_LEVEL); break;
            case SELECT_LEVEL: transition(level == 1 ? State.READY1 : State.READY2); break;
            case READY1: if (actuators.elevatorAt(LiftingSequenceConfig.ElevatorTarget.READY1)) transition(State.SCAN_RIGHT); break;
            case READY2: if (actuators.elevatorAt(LiftingSequenceConfig.ElevatorTarget.READY2)) transition(State.SCAN_RIGHT); break;
            case SCAN_RIGHT: if (cameraOk(now)) transition(State.SCAN_LEFT); else retry(FailureCode.CAMERA_CLASSIFICATION_FAILED); break;
            case SCAN_LEFT: if (cameraOk(now)) transition(State.CENTER_LEFT_SLOW); else retry(FailureCode.CAMERA_CLASSIFICATION_FAILED); break;
            case CENTER_LEFT_SLOW:
                if (!cameraOk(now)) { actuators.stopDrive(); retry(FailureCode.CAMERA_STALE); break; }
                if (Math.abs(camera.leftDxPx()) <= CENTER_DEADBAND_PX) { actuators.stopDrive(); transition(State.APPROACH_IR_SLOW); }
                else actuators.drive(0, camera.leftDxPx() > 0 ? -0.08 : 0.08);
                break;
            case APPROACH_IR_SLOW:
                if (leftIr && rightIr) { actuators.stopDrive(); transition(State.CONFIRM_IR); }
                else { if (leftIr || rightIr) retries=0; actuators.drive(0.08, 0); }
                break;
            case CONFIRM_IR:
                if (leftIr && rightIr) { if (bothIrSinceNs == 0) bothIrSinceNs=now; if (now-bothIrSinceNs >= LiftingSequenceConfig.IR_DEBOUNCE_NS) transition(State.SAVE_SHELF_POSE); }
                else { bothIrSinceNs=0; if (elapsedNs() > LiftingSequenceConfig.STATE_TIMEOUT_NS/2) safeStop(FailureCode.IR_TIMEOUT); }
                break;
            case SAVE_SHELF_POSE: if (pose == null) { safeStop(FailureCode.POSE_INVALID); break; } pose.update();
                if (finitePose()) { shelfPose=new ShelfPose(shelf,level,pose.x(),pose.y(),pose.headingDeg(),now); transition(State.CALIBRATE_SHELF_COORDINATE); } else safeStop(FailureCode.POSE_INVALID); break;
            case CALIBRATE_SHELF_COORDINATE: transition(State.LIFT1); break;
            case LIFT1: if (actuators.elevatorAt(LiftingSequenceConfig.ElevatorTarget.LIFT1)) transition(State.BACK_OUT_FROM_SHELF); break;
            case LIFT2: if (actuators.elevatorAt(LiftingSequenceConfig.ElevatorTarget.LIFT2)) transition(State.BACK_OUT_FROM_SHELF); break;
            case BACK_OUT_FROM_SHELF: actuators.stopDrive(); transition(State.HOLD); break;
            case HOLD: actuators.setFork(LiftingSequenceConfig.ForkPose.HOLD); transition(State.CYCLE_COMPLETE); break;
            case CYCLE_COMPLETE: completedCycles++; if (completedCycles >= 6) { transition(State.SAFE_STOP); failure=FailureCode.NONE; } else { if (++level > 2) { level=1; shelf++; } transition(State.HOMING); } break;
            default: break;
        }
    }
    private boolean cameraOk(long now) { return camera != null && camera.fresh(now) && camera.classificationValid() && camera.stableLeftTarget(); }
    private boolean finitePose() { return Double.isFinite(pose.x()) && Double.isFinite(pose.y()) && Double.isFinite(pose.headingDeg()) && Math.abs(pose.headingDeg()) <= 360.0; }
    private void retry(FailureCode code) { if (++retries > LiftingSequenceConfig.MAX_RETRIES) safeStop(code); }
    private void transition(State next) { state=next; stateStartedNs=clock.nowNs(); retries=0; bothIrSinceNs=0; }
    public void rejectCamera(boolean stale, double x, double y, double heading) { if (stale || !Double.isFinite(x)||!Double.isFinite(y)||!Double.isFinite(heading)) safeStop(stale ? FailureCode.CAMERA_STALE : FailureCode.POSE_INVALID); }
    public void safeStop(FailureCode reason) { if (state==State.SAFE_STOP) return; failure=reason; state=State.SAFE_STOP; actuators.stop(); }
}
