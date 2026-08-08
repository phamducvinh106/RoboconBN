package org.firstinspires.ftc.teamcode.core;

public final class LiftingSequenceStateMachine {
    public enum State {
        START, HOMING, SET_PLACE, PLACE_AT_FACTORY, MOVE_TO_SHELF, SELECT_LEVEL, READY1_PUSH, READY1, READY2,
        SCAN_RIGHT, APPROACH_IR_SLOW, CONFIRM_IR, SAVE_SHELF_POSE, LIFT1, LIFT2,
        BACK_OUT_FROM_SHELF, HOLD, MOVE_NEAR_FACTORY_LEFT,
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
        void resetNavigation();
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
    private int settleCount;
    private double progressAnchorX;
    private double progressAnchorY;
    private long progressCheckNs;
    private LiftingSequenceConfig.Pose lastArrivalTarget;
    private ShelfPose shelfPose;
    private String leftType = "01";
    private String rightType = "02";

    private static final long HOMING_TIMEOUT_NS = 15_000_000_000L;
    private static final long ELEVATOR_TIMEOUT_NS = 20_000_000_000L;
    private static final long NAV_TIMEOUT_NS = 30_000_000_000L;
    private static final long PROGRESS_WINDOW_NS = 3_000_000_000L;

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

    public int getSettleCount() {
        return settleCount;
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
        switch (state) {
            case START:
                transition(State.HOMING);
                break;
            case HOMING:
                if (actuators.home()) {
                    transition(State.SET_PLACE);
                } else if (elapsedNs() >= HOMING_TIMEOUT_NS) {
                    handleActuatorTimeout();
                }
                break;
            case SET_PLACE:
                actuators.setFork(LiftingSequenceConfig.ForkPose.PLACE);
                transition(State.PLACE_AT_FACTORY);
                break;
            case PLACE_AT_FACTORY:
                if (arrivalSettled(placeAtFactoryPose(), now)) transition(State.MOVE_TO_SHELF);
                break;
            case MOVE_TO_SHELF:
                if (arrivalSettled(currentShelfPose(), now)) transition(State.SELECT_LEVEL);
                break;
            case SELECT_LEVEL:
                transition(level == 1 ? State.READY1 : State.READY2);
                break;
            case READY1:
                if (actuators.elevatorAt(LiftingSequenceConfig.ElevatorTarget.READY1)) {
                    transition(State.SCAN_RIGHT);
                } else if (elapsedNs() >= ELEVATOR_TIMEOUT_NS) {
                    handleActuatorTimeout();
                }
                break;
            case READY2:
                if (actuators.elevatorAt(LiftingSequenceConfig.ElevatorTarget.READY2)) {
                    transition(State.SCAN_RIGHT);
                } else if (elapsedNs() >= ELEVATOR_TIMEOUT_NS) {
                    handleActuatorTimeout();
                }
                break;
            case SCAN_RIGHT:
                applyScanTypes(now);
                transition(State.APPROACH_IR_SLOW);
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
                transition(level == 1 ? State.LIFT1 : State.LIFT2);
                break;
            case LIFT1:
                if (actuators.elevatorAt(LiftingSequenceConfig.ElevatorTarget.LIFT1)) {
                    transition(State.BACK_OUT_FROM_SHELF);
                } else if (elapsedNs() >= ELEVATOR_TIMEOUT_NS) {
                    handleActuatorTimeout();
                }
                break;
            case LIFT2:
                if (actuators.elevatorAt(LiftingSequenceConfig.ElevatorTarget.LIFT2)) {
                    transition(State.BACK_OUT_FROM_SHELF);
                } else if (elapsedNs() >= ELEVATOR_TIMEOUT_NS) {
                    handleActuatorTimeout();
                }
                break;
            case BACK_OUT_FROM_SHELF:
                if (arrivalSettled(shelfBackOutPose(), now)) transition(State.HOLD);
                break;
            case HOLD:
                actuators.setFork(LiftingSequenceConfig.ForkPose.HOLD);
                transition(State.MOVE_NEAR_FACTORY_LEFT);
                break;
            case MOVE_NEAR_FACTORY_LEFT:
                if (arrivalSettled(factory(leftType).near, now)) transition(State.PLACE_LEFT);
                break;
            case PLACE_LEFT:
                actuators.setFork(LiftingSequenceConfig.ForkPose.PLACE);
                transition(State.READY1_LEFT);
                break;
            case READY1_LEFT:
                if (actuators.elevatorAt(LiftingSequenceConfig.ElevatorTarget.READY1)) {
                    transition(State.MOVE_TO_PLACEMENT_LEFT);
                } else if (elapsedNs() >= ELEVATOR_TIMEOUT_NS) {
                    handleActuatorTimeout();
                }
                break;
            case MOVE_TO_PLACEMENT_LEFT:
                if (arrivalSettled(factory(leftType).placement, now)) transition(State.HOME_LEFT);
                break;
            case HOME_LEFT:
                if (actuators.home()) {
                    actuators.markBackOutAnchor();
                    transition(State.BACK_OUT_AFTER_LEFT_RELEASE_20CM);
                } else if (elapsedNs() >= HOMING_TIMEOUT_NS) {
                    handleActuatorTimeout();
                }
                break;
            case BACK_OUT_AFTER_LEFT_RELEASE_20CM:
                if (arrivalSettled(retreatPose(), now)
                        && actuators.backOutDistanceCm() >= backOutCm()) {
                    transition(State.RIGHT_RE_LIFT);
                }
                break;
            case RIGHT_RE_LIFT:
                if (actuators.elevatorAt(level == 1
                        ? LiftingSequenceConfig.ElevatorTarget.LIFT1
                        : LiftingSequenceConfig.ElevatorTarget.LIFT2)) {
                    transition(State.MOVE_NEAR_FACTORY_RIGHT);
                } else if (elapsedNs() >= ELEVATOR_TIMEOUT_NS) {
                    handleActuatorTimeout();
                }
                break;
            case MOVE_NEAR_FACTORY_RIGHT:
                if (arrivalSettled(factory(rightType).near, now)) transition(State.PLACE_RIGHT);
                break;
            case PLACE_RIGHT:
                actuators.setFork(LiftingSequenceConfig.ForkPose.PLACE);
                transition(State.READY1_RIGHT);
                break;
            case READY1_RIGHT:
                if (actuators.elevatorAt(LiftingSequenceConfig.ElevatorTarget.READY1)) {
                    transition(State.MOVE_TO_PLACEMENT_RIGHT);
                } else if (elapsedNs() >= ELEVATOR_TIMEOUT_NS) {
                    handleActuatorTimeout();
                }
                break;
            case MOVE_TO_PLACEMENT_RIGHT:
                if (arrivalSettled(factory(rightType).placement, now)) transition(State.HOME_RIGHT);
                break;
            case HOME_RIGHT:
                if (actuators.home()) {
                    actuators.markBackOutAnchor();
                    transition(State.BACK_OUT_AFTER_RIGHT_RELEASE_20CM);
                } else if (elapsedNs() >= HOMING_TIMEOUT_NS) {
                    handleActuatorTimeout();
                }
                break;
            case BACK_OUT_AFTER_RIGHT_RELEASE_20CM:
                if (arrivalSettled(retreatPose(), now)
                        && actuators.backOutDistanceCm() >= backOutCm()) {
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

    private LiftingSequenceConfig.Pose currentShelfPose() {
        return config == null ? new LiftingSequenceConfig.Pose(0, 0, 0) : config.shelfFor(shelf);
    }

    private LiftingSequenceConfig.Pose shelfBackOutPose() {
        if (shelfPose == null) return retreatPose();
        return new LiftingSequenceConfig.Pose(
                shelfPose.x + backOutCm(), shelfPose.y, shelfPose.heading);
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

    private double approachSpeed() {
        return config == null ? 0.30 : config.approachSpeed;
    }

    private void applyScanTypes(long nowNs) {
        String left = "01";
        String right = "02";
        if (camera != null) {
            String detectedLeft = resolveBlockType(
                    camera.leftValid(), camera.leftFresh(nowNs), camera.leftBlockType());
            String detectedRight = resolveBlockType(
                    camera.rightValid(), camera.rightFresh(nowNs), camera.rightBlockType());
            if (detectedLeft != null) left = detectedLeft;
            if (detectedRight != null) right = detectedRight;
        }
        setBlockTypes(left, right);
    }

    private String resolveBlockType(boolean valid, boolean fresh, String rawType) {
        if (!valid || !fresh) return null;
        String type = normalizeBlockType(rawType);
        return knownFactory(type) ? type : null;
    }

    private String normalizeBlockType(String type) {
        if (type == null) return null;
        String trimmed = type.trim();
        if (trimmed.isEmpty()) return null;
        if (trimmed.length() == 1 && Character.isDigit(trimmed.charAt(0))) {
            return "0" + trimmed;
        }
        return trimmed;
    }

    private boolean knownFactory(String type) {
        if (type == null || type.isEmpty()) return false;
        return config == null || config.factories.containsKey(type);
    }

    private boolean arrivalSettled(LiftingSequenceConfig.Pose target, long nowNs) {
        PoseReading pose = actuators.pose();
        if (pose == null || !pose.valid) {
            safeStop(FailureCode.ENCODER_INVALID);
            return false;
        }
        if (!poseFresh(pose, nowNs)) {
            safeStop(FailureCode.ENCODER_INVALID);
            return false;
        }
        if (!samePose(lastArrivalTarget, target)) {
            settleCount = 0;
            lastArrivalTarget = target;
            progressAnchorX = pose.x;
            progressAnchorY = pose.y;
            progressCheckNs = nowNs;
        }
        if (elapsedNs() >= NAV_TIMEOUT_NS) {
            handleNavigationTimeout();
            return false;
        }
        if (actuators.arrival(target, nowNs)) {
            settleCount++;
            return settleCount >= settleCycles();
        }
        settleCount = 0;
        checkNavigationProgress(pose, nowNs);
        return false;
    }

    private void checkNavigationProgress(PoseReading pose, long nowNs) {
        double moved = Math.hypot(pose.x - progressAnchorX, pose.y - progressAnchorY);
        if (moved >= noProgressCm()) {
            progressAnchorX = pose.x;
            progressAnchorY = pose.y;
            progressCheckNs = nowNs;
            return;
        }
        if (nowNs - progressCheckNs < PROGRESS_WINDOW_NS) return;
        if (retries < maxRetries()) {
            actuators.resetNavigation();
            retries++;
            settleCount = 0;
            progressCheckNs = nowNs;
            return;
        }
        safeStop(FailureCode.NO_PROGRESS);
    }

    private void handleNavigationTimeout() {
        if (retries < maxRetries()) {
            actuators.resetNavigation();
            retries++;
            settleCount = 0;
            stateStartedNs = clock.nowNs();
            return;
        }
        safeStop(FailureCode.NO_PROGRESS);
    }

    private void handleActuatorTimeout() {
        if (retries < maxRetries()) {
            retries++;
            stateStartedNs = clock.nowNs();
            return;
        }
        safeStop(FailureCode.NO_PROGRESS);
    }

    private boolean poseFresh(PoseReading pose, long nowNs) {
        if (pose.timestampNs == 0) return true;
        long freshnessNs = config == null
                ? 250_000_000L
                : (long) config.encoderFreshnessNs;
        return nowNs - pose.timestampNs <= freshnessNs;
    }

    private int settleCycles() {
        return config == null ? 3 : config.settleCycles;
    }

    private int maxRetries() {
        return config == null ? LiftingSequenceConfig.MAX_RETRIES : config.maxRetries;
    }

    private double noProgressCm() {
        return config == null ? 0.1 : config.noProgressCm;
    }

    private static boolean samePose(LiftingSequenceConfig.Pose a, LiftingSequenceConfig.Pose b) {
        if (a == null || b == null) return a == b;
        return Double.compare(a.x, b.x) == 0
                && Double.compare(a.y, b.y) == 0
                && Double.compare(a.heading, b.heading) == 0;
    }

    private void transition(State next) {
        state = next;
        stateStartedNs = clock.nowNs();
        retries = 0;
        settleCount = 0;
        lastArrivalTarget = null;
        bothIrSinceNs = 0;
    }

    public void safeStop(FailureCode failureCode) {
        failure = failureCode;
        state = State.SAFE_STOP;
        actuators.stop();
    }
}
