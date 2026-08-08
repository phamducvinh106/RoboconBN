package org.firstinspires.ftc.teamcode.core;

public final class LiftingSequenceStateMachine {
    public enum State {
        START, HOMING, SET_PLACE, PLACE_AT_FACTORY, MOVE_TO_SHELF, SELECT_LEVEL, READY1_PUSH, READY1, READY2,
        SCAN_RIGHT, SCAN_LEFT, CENTER_LEFT_SLOW, APPROACH_IR_SLOW, CONFIRM_IR, SAVE_SHELF_POSE,
        CALIBRATE_SHELF_COORDINATE, LIFT1, LIFT2, BACK_OUT_FROM_SHELF, HOLD, MOVE_NEAR_FACTORY_LEFT,
        PLACE_LEFT, READY1_LEFT, MOVE_TO_PLACEMENT_LEFT, HOME_LEFT, BACK_OUT_AFTER_LEFT_RELEASE_20CM,
        RIGHT_RE_LIFT, MOVE_NEAR_FACTORY_RIGHT, PLACE_RIGHT, READY1_RIGHT, MOVE_TO_PLACEMENT_RIGHT,
        HOME_RIGHT, BACK_OUT_AFTER_RIGHT_RELEASE_20CM, CYCLE_COMPLETE, SAFE_STOP
    }

    public enum FailureCode {
        NONE, STOP_REQUESTED, CAMERA_STALE, POSE_INVALID, ENCODER_INVALID, NO_PROGRESS, RELEASE_UNCONFIRMED
    }

    public interface Clock {
        long nowNs();
    }

    public interface Actuators {
        void stop();
        boolean home();
        boolean elevatorAt(LiftingSequenceConfig.ElevatorTarget target);
        void setFork(LiftingSequenceConfig.ForkPose pose);
        void drive(double forward, double strafe);
        void stopDrive();
        void markBackOutAnchor();
        double backOutDistanceCm();
        PoseReading pose();
        boolean arrival(LiftingSequenceConfig.Pose target, long nowNs);
    }

    public interface CameraResult {
        boolean leftFresh(long nowNs);
        boolean rightFresh(long nowNs);
        boolean leftValid();
        boolean rightValid();
        boolean stableLeftTarget();
        double leftDxPx();
        String leftBlockType();
        String rightBlockType();
    }

    public static final class PoseReading {
        public final double x, y, heading;
        public final long timestampNs;
        public final boolean valid;

        public PoseReading(double x, double y, double heading, long timestampNs) {
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.timestampNs = timestampNs;
            valid = Double.isFinite(x) && Double.isFinite(y) && Double.isFinite(heading)
                    && Math.abs(heading) <= 360;
        }
    }

    public static final class ShelfPose {
        public final int shelf, level;
        public final double x, y, heading;
        public final long timestampNs;

        ShelfPose(int shelf, int level, double x, double y, double heading, long timestampNs) {
            this.shelf = shelf;
            this.level = level;
            this.x = x;
            this.y = y;
            this.heading = heading;
            this.timestampNs = timestampNs;
        }
    }

    private final Clock clock;
    private final Actuators actuators;
    private final CameraResult camera;
    private final LiftingSequenceConfig config;
    private State state = State.START;
    private FailureCode failure = FailureCode.NONE;
    private long stateStartedNs;
    private long bothIrSinceNs;
    private boolean stopRequested;
    private boolean active = true;
    private boolean leftIr;
    private boolean rightIr;
    private int shelf = 1;
    private int level = 1;
    private int completedCycles;
    private int retries;
    private ShelfPose shelfPose;
    private String leftType = "01";
    private String rightType = "02";

    public LiftingSequenceStateMachine(Clock clock, Actuators actuators) {
        this(clock, actuators, null, null);
    }

    public LiftingSequenceStateMachine(Clock clock, Actuators actuators, CameraResult camera) {
        this(clock, actuators, camera, null);
    }

    public LiftingSequenceStateMachine(Clock clock, Actuators actuators, CameraResult camera,
                                       LiftingSequenceConfig config) {
        if (clock == null || actuators == null) throw new NullPointerException();
        LiftingSequenceConfig.validate();
        this.clock = clock;
        this.actuators = actuators;
        this.camera = camera;
        this.config = config;
        stateStartedNs = clock.nowNs();
    }

    public void setBlockTypes(String left, String right) {
        factory(left);
        factory(right);
        leftType = left;
        rightType = right;
    }

    private LiftingSequenceConfig.Factory factory(String type) {
        return config == null ? LiftingSequenceConfig.factory(type) : config.factoryFor(type);
    }

    public void requestStop() {
        stopRequested = true;
    }

    public void setActive(boolean active) {
        this.active = active;
    }

    public State getState() {
        return state;
    }

    public FailureCode getFailure() {
        return failure;
    }

    public long elapsedNs() {
        return clock.nowNs() - stateStartedNs;
    }

    public int getShelf() {
        return shelf;
    }

    public int getLevel() {
        return level;
    }

    public int getCompletedCycles() {
        return completedCycles;
    }

    public int getRetries() {
        return retries;
    }

    public ShelfPose getShelfPose() {
        return shelfPose;
    }

    public String getLeftType() {
        return leftType;
    }

    public String getRightType() {
        return rightType;
    }

    public void setIrState(boolean left, boolean right) {
        leftIr = left;
        rightIr = right;
    }

    public void tick() {
        if (state == State.SAFE_STOP) return;
        long now = clock.nowNs();
        if (stopRequested || !active) {
            safeStop(FailureCode.STOP_REQUESTED);
            return;
        }
        if (stateTimedOut(now)) {
            safeStop(FailureCode.NO_PROGRESS);
            return;
        }
        switch (state) {
            case START:
                transition(State.HOMING);
                break;
            case HOMING:
                if (actuators.home()) transition(State.SET_PLACE);
                break;
            case SET_PLACE:
                actuators.setFork(LiftingSequenceConfig.ForkPose.PLACE);
                transition(State.PLACE_AT_FACTORY);
                break;
            case PLACE_AT_FACTORY:
                if (arrival(placeAtFactoryPose(), now)) transition(State.MOVE_TO_SHELF);
                break;
            case MOVE_TO_SHELF:
                if (arrival(shelfApproachPose(), now)) transition(State.SELECT_LEVEL);
                break;
            case SELECT_LEVEL:
                transition(level == 1 ? State.READY1 : State.READY2);
                break;
            case READY1:
                if (actuators.elevatorAt(LiftingSequenceConfig.ElevatorTarget.READY1)) transition(State.SCAN_RIGHT);
                break;
            case READY2:
                if (actuators.elevatorAt(LiftingSequenceConfig.ElevatorTarget.READY2)) transition(State.SCAN_RIGHT);
                break;
            case SCAN_RIGHT:
                if (dualScanReady(now)) {
                    setBlockTypes(camera.leftBlockType(), camera.rightBlockType());
                    transition(State.CENTER_LEFT_SLOW);
                }
                break;
            case SCAN_LEFT:
                transition(State.CENTER_LEFT_SLOW);
                break;
            case CENTER_LEFT_SLOW:
                if (!leftCenteringOk(now)) {
                    actuators.stopDrive();
                    safeStop(FailureCode.CAMERA_STALE);
                } else if (camera.stableLeftTarget()) {
                    actuators.stopDrive();
                    transition(State.APPROACH_IR_SLOW);
                } else {
                    double speed = centerSpeed();
                    actuators.drive(0, camera.leftDxPx() > 0 ? -speed : speed);
                }
                break;
            case APPROACH_IR_SLOW:
                if (leftIr && rightIr) {
                    actuators.stopDrive();
                    transition(State.CONFIRM_IR);
                } else {
                    actuators.drive(approachSpeed(), 0);
                }
                break;
            case CONFIRM_IR:
                if (leftIr && rightIr) {
                    if (bothIrSinceNs == 0) bothIrSinceNs = now;
                    if (now - bothIrSinceNs >= irDebounceNs()) transition(State.SAVE_SHELF_POSE);
                } else {
                    bothIrSinceNs = 0;
                }
                break;
            case SAVE_SHELF_POSE:
                PoseReading pose = actuators.pose();
                if (pose == null || !pose.valid) {
                    safeStop(FailureCode.POSE_INVALID);
                    break;
                }
                shelfPose = new ShelfPose(shelf, level, pose.x, pose.y, pose.heading, now);
                transition(State.CALIBRATE_SHELF_COORDINATE);
                break;
            case CALIBRATE_SHELF_COORDINATE:
                transition(level == 1 ? State.LIFT1 : State.LIFT2);
                break;
            case LIFT1:
                if (actuators.elevatorAt(LiftingSequenceConfig.ElevatorTarget.LIFT1)) {
                    transition(State.BACK_OUT_FROM_SHELF);
                }
                break;
            case LIFT2:
                if (actuators.elevatorAt(LiftingSequenceConfig.ElevatorTarget.LIFT2)) {
                    transition(State.BACK_OUT_FROM_SHELF);
                }
                break;
            case BACK_OUT_FROM_SHELF:
                if (arrival(retreatPose(), now)) transition(State.HOLD);
                break;
            case HOLD:
                actuators.setFork(LiftingSequenceConfig.ForkPose.HOLD);
                transition(State.MOVE_NEAR_FACTORY_LEFT);
                break;
            case MOVE_NEAR_FACTORY_LEFT:
                if (arrival(factory(leftType).near, now)) transition(State.PLACE_LEFT);
                break;
            case PLACE_LEFT:
                actuators.setFork(LiftingSequenceConfig.ForkPose.PLACE);
                transition(State.READY1_LEFT);
                break;
            case READY1_LEFT:
                if (actuators.elevatorAt(LiftingSequenceConfig.ElevatorTarget.READY1)) {
                    transition(State.MOVE_TO_PLACEMENT_LEFT);
                }
                break;
            case MOVE_TO_PLACEMENT_LEFT:
                if (arrival(factory(leftType).placement, now)) transition(State.HOME_LEFT);
                break;
            case HOME_LEFT:
                if (actuators.home()) {
                    actuators.markBackOutAnchor();
                    transition(State.BACK_OUT_AFTER_LEFT_RELEASE_20CM);
                }
                break;
            case BACK_OUT_AFTER_LEFT_RELEASE_20CM:
                if (actuators.backOutDistanceCm() >= backOutCm() && arrival(retreatPose(), now)) {
                    transition(State.RIGHT_RE_LIFT);
                }
                break;
            case RIGHT_RE_LIFT:
                if (actuators.elevatorAt(level == 1
                        ? LiftingSequenceConfig.ElevatorTarget.LIFT1
                        : LiftingSequenceConfig.ElevatorTarget.LIFT2)) {
                    transition(State.MOVE_NEAR_FACTORY_RIGHT);
                }
                break;
            case MOVE_NEAR_FACTORY_RIGHT:
                if (arrival(factory(rightType).near, now)) transition(State.PLACE_RIGHT);
                break;
            case PLACE_RIGHT:
                actuators.setFork(LiftingSequenceConfig.ForkPose.PLACE);
                transition(State.READY1_RIGHT);
                break;
            case READY1_RIGHT:
                if (actuators.elevatorAt(LiftingSequenceConfig.ElevatorTarget.READY1)) {
                    transition(State.MOVE_TO_PLACEMENT_RIGHT);
                }
                break;
            case MOVE_TO_PLACEMENT_RIGHT:
                if (arrival(factory(rightType).placement, now)) transition(State.HOME_RIGHT);
                break;
            case HOME_RIGHT:
                if (actuators.home()) {
                    actuators.markBackOutAnchor();
                    transition(State.BACK_OUT_AFTER_RIGHT_RELEASE_20CM);
                }
                break;
            case BACK_OUT_AFTER_RIGHT_RELEASE_20CM:
                if (actuators.backOutDistanceCm() >= backOutCm() && arrival(retreatPose(), now)) {
                    transition(State.CYCLE_COMPLETE);
                }
                break;
            case CYCLE_COMPLETE:
                completedCycles++;
                if (completedCycles >= totalCycles()) {
                    transition(State.SAFE_STOP);
                } else {
                    if (++level > 2) {
                        level = 1;
                        shelf++;
                    }
                    transition(State.HOMING);
                }
                break;
            default:
                break;
        }
    }

    private LiftingSequenceConfig.Pose shelfApproachPose() {
        return config == null ? new LiftingSequenceConfig.Pose(0, 0, 0) : config.shelfApproach;
    }

    private LiftingSequenceConfig.Pose retreatPose() {
        return config == null ? new LiftingSequenceConfig.Pose(0, 0, 0) : config.retreat;
    }

    private LiftingSequenceConfig.Pose placeAtFactoryPose() {
        return config == null ? new LiftingSequenceConfig.Pose(0, 0, 0) : config.placeAtFactory;
    }

    private int totalCycles() {
        return config == null ? LiftingSequenceConfig.TOTAL_CYCLES : config.totalCycles;
    }

    private double backOutCm() {
        return config == null ? LiftingSequenceConfig.RELEASE_BACK_OUT_CM : config.releaseBackOutCm;
    }

    private long irDebounceNs() {
        return config == null ? LiftingSequenceConfig.IR_DEBOUNCE_NS : config.irDebounceNs;
    }

    private double centerSpeed() {
        return config == null ? 0.08 : config.centerSpeed;
    }

    private double approachSpeed() {
        return config == null ? 0.08 : config.approachSpeed;
    }

    private boolean dualScanReady(long nowNs) {
        if (camera == null) return false;
        String left = camera.leftBlockType();
        String right = camera.rightBlockType();
        return camera.leftValid() && camera.leftFresh(nowNs) && camera.rightValid()
                && camera.rightFresh(nowNs) && knownFactory(left) && knownFactory(right);
    }

    private boolean leftCenteringOk(long nowNs) {
        return camera != null && camera.leftValid() && camera.leftFresh(nowNs);
    }

    private boolean knownFactory(String type) {
        if (type == null || type.isEmpty()) return false;
        return config == null || config.factories.containsKey(type);
    }

    private boolean stateTimedOut(long nowNs) {
        long limit = stateTimeoutLimitNs();
        return limit > 0 && nowNs - stateStartedNs > limit;
    }

    private long stateTimeoutLimitNs() {
        if (config == null) return LiftingSequenceConfig.STATE_TIMEOUT_NS;
        switch (state) {
            case HOMING:
            case READY1:
            case READY2:
            case LIFT1:
            case LIFT2:
            case READY1_LEFT:
            case READY1_RIGHT:
            case HOME_LEFT:
            case HOME_RIGHT:
            case RIGHT_RE_LIFT:
                return config.elevatorTimeoutNs;
            default:
                return config.stateTimeoutNs;
        }
    }

    private boolean arrival(LiftingSequenceConfig.Pose target, long nowNs) {
        PoseReading pose = actuators.pose();
        if (pose == null || !pose.valid) {
            safeStop(FailureCode.ENCODER_INVALID);
            return false;
        }
        return actuators.arrival(target, nowNs);
    }

    private void transition(State next) {
        state = next;
        stateStartedNs = clock.nowNs();
        retries = 0;
        bothIrSinceNs = 0;
    }

    public void safeStop(FailureCode failureCode) {
        failure = failureCode;
        state = State.SAFE_STOP;
        actuators.stop();
    }
}
