package org.firstinspires.ftc.teamcode.core;

public final class LiftingSequenceConfig {
    public enum ElevatorTarget {
        HOME(0), READY1(844), LIFT1(1781), READY2(4688), LIFT2(5625);
        public final int steps;
        ElevatorTarget(int steps) { this.steps = steps; }
    }

    public enum ForkPose { PLACE, HOLD }

    public static final double PLACE_LEFT = 0.25;
    public static final double PLACE_RIGHT = 0.75;
    public static final double HOLD_LEFT = 0.50;
    public static final double HOLD_RIGHT = 0.50;
    public static final long STEP_HIGH_NS = 1_000_000L;
    public static final long STEP_LOW_NS = 1_000_000L;
    public static final long IR_DEBOUNCE_NS = 100_000_000L;
    public static final long SENSOR_STALE_NS = 250_000_000L;
    public static final long ELEVATOR_TIMEOUT_NS = 8_000_000_000L;
    public static final long STATE_TIMEOUT_NS = 10_000_000_000L;
    public static final int MAX_RETRIES = 2;
    public static final double RELEASE_BACK_OUT_CM = 20.0;
    public static final double POSITION_TOLERANCE_CM = 1.0;
    public static final double HEADING_TOLERANCE_DEG = 3.0;

    private LiftingSequenceConfig() { }

    public static void validate() {
        if (STEP_HIGH_NS <= 0 || STEP_LOW_NS <= 0 || IR_DEBOUNCE_NS <= 0
                || SENSOR_STALE_NS <= 0 || ELEVATOR_TIMEOUT_NS <= 0 || STATE_TIMEOUT_NS <= 0
                || MAX_RETRIES < 0 || RELEASE_BACK_OUT_CM <= 0 || POSITION_TOLERANCE_CM <= 0
                || HEADING_TOLERANCE_DEG <= 0) {
            throw new IllegalStateException("non-positive lifting configuration");
        }
        int previous = -1;
        for (ElevatorTarget target : ElevatorTarget.values()) {
            if (target.steps < previous) throw new IllegalStateException("elevator targets not ordered");
            previous = target.steps;
        }
    }
}
