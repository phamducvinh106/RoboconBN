package org.firstinspires.ftc.teamcode.core;

public final class StepperElevatorManager {
    private static final int HOMING_POLL_STRIDE = 32;

    private final boolean dirInverted;
    private final HardwareContracts.BinaryChannel step, dir;
    private final EndstopManager endstop;
    private final int maxSteps;
    private int position;
    private boolean known;

    public StepperElevatorManager(HardwareContracts.BinaryChannel step, HardwareContracts.BinaryChannel dir,
                                  EndstopManager endstop, long highNs, long lowNs, int maxSteps) {
        this(step, dir, endstop, highNs, lowNs, maxSteps, false);
    }

    public StepperElevatorManager(HardwareContracts.BinaryChannel step, HardwareContracts.BinaryChannel dir,
                                  EndstopManager endstop, long highNs, long lowNs, int maxSteps,
                                  boolean dirInverted) {
        if (step == null || dir == null || endstop == null || highNs <= 0 || lowNs <= 0 || maxSteps < 1) {
            throw new IllegalArgumentException();
        }
        this.step = step;
        this.dir = dir;
        this.dirInverted = dirInverted;
        this.endstop = endstop;
        this.maxSteps = maxSteps;
    }

    /** Blocks until target is reached or homing completes. */
    public boolean moveToward(int target, long ignoredNowNs) {
        if (target < 0 || target > maxSteps) throw new IllegalArgumentException("target out of bounds");
        if (target == 0) {
            if (!endstop.active()) {
                pulseDown(maxSteps);
            }
            if (endstop.active()) {
                step.setHigh(false);
                position = 0;
                known = true;
                return true;
            }
            known = false;
            return false;
        }
        if (!known) return false;
        int delta = target - position;
        if (delta == 0) {
            step.setHigh(false);
            return true;
        }
        if (delta > 0) {
            setDir(true);
            step.setHigh(false);
            for (int i = 0; i < delta; i++) {
                step.setHigh(true);
                step.setHigh(false);
                position++;
            }
        } else {
            pulseDown(-delta);
            if (endstop.active()) {
                position = 0;
                known = true;
            }
        }
        return position == target;
    }

    private void pulseDown(int maxDownSteps) {
        setDir(false);
        step.setHigh(false);
        int moved = 0;
        while (!endstop.active() && moved < maxDownSteps) {
            int burst = Math.min(HOMING_POLL_STRIDE, maxDownSteps - moved);
            for (int i = 0; i < burst; i++) {
                step.setHigh(true);
                step.setHigh(false);
                moved++;
            }
        }
        if (endstop.active()) {
            position = 0;
            known = true;
        } else {
            position = Math.max(0, position - moved);
        }
    }

    public void stop() { step.setHigh(false); }
    public int position() { return position; }
    public boolean positionKnown() { return known; }
    public boolean pulsing() { return false; }

    private void setDir(boolean up) {
        dir.setHigh(dirInverted ? !up : up);
    }
}
