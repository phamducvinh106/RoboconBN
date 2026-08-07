package org.firstinspires.ftc.teamcode.test;

/** Deterministic executable fixtures for Localizer's calibrated integration contract. */
public final class LocalizerMathTest {
    private static final double CM_PER_TICK = Math.PI * 3.2 / 2000.0;
    private static final double EPS = 1e-9;

    public static void main(String[] args) {
        Model model = new Model();
        checkClose(0, model.x, "zero x");
        checkClose(0, model.y, "zero y");

        model.update(0, 0, 0);
        checkClose(0, model.forward, "zero forward");
        model.update(-1000, 0, 0);
        checkClose(1000 * CM_PER_TICK, model.forward, "forward distance");
        checkClose(0, model.left, "forward left");
        checkClose(0, model.x, "forward x at zero heading");
        checkClose(1000 * CM_PER_TICK, model.y, "forward y at zero heading");

        model.resetPose();
        model.update(0, -1000, 0);
        checkClose(1000 * CM_PER_TICK, model.left, "left distance");
        checkClose(-1000 * CM_PER_TICK, model.x, "left x at zero heading");
        checkClose(0, model.y, "left y at zero heading");

        model.resetPose();
        model.update(-1000, 0, Math.PI / 2);
        checkClose(-(1000 * CM_PER_TICK) , model.x, "forward x at +90");
        checkClose(0, model.y, "forward y at +90");
        model.resetPose();
        model.update(0, -1000, Math.PI / 2);
        checkClose(0, model.x, "left x at +90");
        checkClose(-(1000 * CM_PER_TICK), model.y, "left y at +90");

        model.resetPose();
        model.update(-500, -500, -Math.PI / 2);
        checkClose(500 * CM_PER_TICK, model.x, "combined x at -90");
        checkClose(500 * CM_PER_TICK, model.y, "combined y at -90");

        Model wrap = new Model();
        wrap.heading = Math.PI - 0.01;
        wrap.update(0, 0, -Math.PI + 0.01);
        checkClose(0.02, wrap.deltaHeading, "positive pi wrap");
        wrap.heading = -Math.PI + 0.01;
        wrap.update(0, 0, Math.PI - 0.01);
        checkClose(-0.02, wrap.deltaHeading, "negative pi wrap");

        wrap.resetPose();
        checkClose(0, wrap.x, "reset x");
        checkClose(0, wrap.accumHeading, "reset heading accumulator");
        check(Double.isNaN(wrap.parallelOffset()), "offset denominator guard");
        wrap.update(-100, 0, 0.3);
        check(Double.isFinite(wrap.parallelOffset()), "finite offset after heading");
        check(Double.isNaN(new Model().parallelOffset()), "fresh offset remains NaN");
        System.out.println("LocalizerMathTest PASS");
    }

    private static final class Model {
        double x, y, heading, deltaHeading, forward, left, accumHeading, accumParallel;
        double lastParallel;
        void update(int parallelTicks, int perpendicularTicks, double newHeading) {
            deltaHeading = wrap(newHeading - heading);
            double avg = heading + deltaHeading / 2;
            heading = newHeading;
            if (Math.abs(deltaHeading) < 1e-5 && Math.abs(parallelTicks) < 2 && Math.abs(perpendicularTicks) < 2) return;
            lastParallel = -parallelTicks * CM_PER_TICK;
            double perpendicular = -perpendicularTicks * CM_PER_TICK;
            forward = lastParallel + 5 * deltaHeading;
            left = perpendicular + 25 * deltaHeading;
            x += -forward * Math.sin(avg) - left * Math.cos(avg);
            y += forward * Math.cos(avg) - left * Math.sin(avg);
            accumHeading += deltaHeading;
            accumParallel += lastParallel;
        }
        void resetPose() { x = y = deltaHeading = forward = left = accumHeading = accumParallel = 0; }
        double parallelOffset() { return Math.abs(accumHeading) < 0.2 ? Double.NaN : -accumParallel / accumHeading / -1; }
        static double wrap(double value) { while (value > Math.PI) value -= 2 * Math.PI; while (value < -Math.PI) value += 2 * Math.PI; return value; }
    }

    private static void check(boolean value, String name) { if (!value) throw new AssertionError(name); }
    private static void checkClose(double expected, double actual, String name) {
        if (Math.abs(expected - actual) > EPS) throw new AssertionError(name + ": " + actual + " != " + expected);
    }
}
