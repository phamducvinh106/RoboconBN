package org.firstinspires.ftc.teamcode.core;

public final class StepperElevatorManager {
    private final HardwareContracts.BinaryChannel step, dir;
    private final EndstopManager endstop;
    private final long highNs, lowNs;
    private final int maxSteps;
    private int position;
    private boolean known;

    public StepperElevatorManager(HardwareContracts.BinaryChannel step, HardwareContracts.BinaryChannel dir,
                                  EndstopManager endstop, long highNs, long lowNs, int maxSteps) {
        if (step == null || dir == null || endstop == null || highNs <= 0 || lowNs <= 0 || maxSteps < 1) {
            throw new IllegalArgumentException();
        }
        this.step = step;
        this.dir = dir;
        this.endstop = endstop;
        this.highNs = highNs;
        this.lowNs = lowNs;
        this.maxSteps = maxSteps;
    }

    /** Blocks until target is reached or homing completes. */
    public boolean moveToward(int target, long ignoredNowNs) {
        if (target < 0 || target > maxSteps) throw new IllegalArgumentException("target out of bounds");
        if (endstop.active()) {
            step.setHigh(false);
            position = 0;
            known = true;
            if (target == 0) return true;
        }
        if (target != 0 && !known) return false;
        int delta = target - position;
        if (delta == 0) {
            step.setHigh(false);
            return true;
        }
        boolean up = delta > 0;
        int remaining = Math.abs(delta);
        step.setHigh(false);
        dir.setHigh(up);
        for (int i = 0; i < remaining; i++) {
            if (!up && endstop.active()) {
                step.setHigh(false);
                position = 0;
                known = true;
                return target == 0;
            }
            step.setHigh(true);
            busyWaitNs(highNs);
            step.setHigh(false);
            busyWaitNs(lowNs);
            if (!up && endstop.active()) {
                position = 0;
                known = true;
                return target == 0;
            }
            position += up ? 1 : -1;
        }
        return position == target;
    }

    public void stop() { step.setHigh(false); }
    public int position() { return position; }
    public boolean positionKnown() { return known; }
    public boolean pulsing() { return false; }

    private static void busyWaitNs(long durationNs) {
        if (durationNs <= 1L) return;
        long deadline = System.nanoTime() + durationNs;
        while (System.nanoTime() < deadline) { }
    }
}
