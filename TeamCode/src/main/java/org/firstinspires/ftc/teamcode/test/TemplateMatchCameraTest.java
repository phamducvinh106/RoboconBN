package org.firstinspires.ftc.teamcode.test;

/**
 * Test offline cho bộ lọc temporal cúa TemplateMatchCamera.
 * Chạy: java TemplateMatchCameraTest (không cần phần cứng FTC).
 *
 * Bô lọc hoàn toàn giống TemplateMatchPipeline, kiểm tra 6 scenario:
 *
 *   1. Jitter reduction     — noise +-4px → output <= 1.8px
 *   2. Smooth motion        — 2px/frame, 0 misses
 *   3. Spike rejection      — 200px spike, 4-frame confirm
 *   4. Miss hold            — 5 miss frames hold, 10 clears
 *   5. No label flapping    — one label, never changes
 *   6. setTarget reset      — filter + state reset on new target
 *
 * Note: matchTemplate không duọc test o dây vì can OpenCV.
 * Test này chi kiêm tra bô lọc temporal và state machine.
 */
public final class TemplateMatchCameraTest {

    // -------------- Hằng số filter (copy từ TemplateMatchCamera) --------------

    private static final double OUTLIER_LIMIT_PX       = 45.0;
    private static final int    OUTLIER_CONFIRM_FRAMES = 4;
    private static final int    HOLD_MISS_FRAMES       = 6;
    private static final long   HOLD_MISS_TIMEOUT_MS   = 300;
    private static final long   MISS_REINIT_TIMEOUT_MS = 250;

    private static final double CENTER_DEADBAND_PX     = 2.0;
    private static final double EMA_ALPHA_SLOW         = 0.08;
    private static final double EMA_ALPHA_FAST         = 0.45;
    private static final double FAST_MOTION_THRESHOLD_PX = 8.0;
    private static final double MAX_OUTPUT_STEP_PX     = 20.0;

    // ============================================================
    //  Mini filter (copy từ TemplateMatchPipeline)
    // ============================================================

    static final class FilterResult {
        final double  dxPx, dyPx;
        final boolean valid;
        final String  label;

        FilterResult(double dxPx, double dyPx, boolean valid, String label) {
            this.dxPx = dxPx; this.dyPx = dyPx; this.valid = valid; this.label = label;
        }

        static FilterResult empty() {
            return new FilterResult(Double.NaN, Double.NaN, false, "");
        }
    }

    boolean filterInitialized;
    double  smoothedDx, smoothedDy;
    double  lastRawDx, lastRawDy;
    int     missStreak, outlierStreak;
    long    missStartMs;
    long    lastSuccessfulDetectionMs;
    boolean lastDetectWasValid;
    String  activeLabel = "";

    FilterResult feed(double rawDx, double rawDy, long nowMs,
                       String label, double confidence) {
        if (label == null || label.isEmpty()) {
            handleMiss(nowMs);
            return lastDetectWasValid
                    ? new FilterResult(smoothedDx, smoothedDy, true, activeLabel)
                    : FilterResult.empty();
        }

        if (!label.equals(activeLabel)) {
            activeLabel = label;
            resetPositionFilter(rawDx, rawDy);
        }

        if (missStreak > 0) {
            long missAge = nowMs - missStartMs;
            if (missAge > MISS_REINIT_TIMEOUT_MS)
                resetPositionFilter(rawDx, rawDy);
        }

        if (isOutlier(rawDx, rawDy)) {
            outlierStreak++;
            if (outlierStreak < OUTLIER_CONFIRM_FRAMES) {
                handleMiss(nowMs);
                return lastDetectWasValid
                        ? new FilterResult(smoothedDx, smoothedDy, true, activeLabel)
                        : FilterResult.empty();
            }
            resetPositionFilter(rawDx, rawDy);
        }

        outlierStreak = 0;
        updatePositionFilter(rawDx, rawDy, confidence);
        lastRawDx = rawDx;
        lastRawDy = rawDy;

        missStreak = 0;
        lastSuccessfulDetectionMs = nowMs;
        lastDetectWasValid = true;

        return new FilterResult(smoothedDx, smoothedDy, true, activeLabel);
    }

    void handleMiss(long nowMs) {
        if (missStreak == 0) missStartMs = nowMs;
        missStreak++;

        long ageMs = lastSuccessfulDetectionMs <= 0
                ? Long.MAX_VALUE : nowMs - lastSuccessfulDetectionMs;

        boolean canHold = lastDetectWasValid
                && missStreak <= HOLD_MISS_FRAMES
                && ageMs <= HOLD_MISS_TIMEOUT_MS;

        if (!canHold) {
            if (lastDetectWasValid || missStreak > HOLD_MISS_FRAMES)
                clearTrackingState();
        }
    }

    private boolean isOutlier(double newDx, double newDy) {
        if (!filterInitialized) return false;
        return Math.hypot(newDx - lastRawDx, newDy - lastRawDy)
                > OUTLIER_LIMIT_PX;
    }

    private void updatePositionFilter(double newDx, double newDy, double confidence) {
        if (!filterInitialized) {
            smoothedDx = newDx; smoothedDy = newDy;
            filterInitialized = true;
            return;
        }
        double diffX = newDx - smoothedDx;
        double diffY = newDy - smoothedDy;
        double distance = Math.hypot(diffX, diffY);

        double deadband = CENTER_DEADBAND_PX * (1.0 + 1.0 - confidence);
        if (distance <= deadband) return;

        double baseAlpha = distance >= FAST_MOTION_THRESHOLD_PX
                ? EMA_ALPHA_FAST : EMA_ALPHA_SLOW;
        double effAlpha = baseAlpha * (0.5 + 0.5 * confidence);

        double stepX = effAlpha * diffX;
        double stepY = effAlpha * diffY;
        double stepDist = Math.hypot(stepX, stepY);

        if (stepDist > MAX_OUTPUT_STEP_PX) {
            double scale = MAX_OUTPUT_STEP_PX / stepDist;
            stepX *= scale; stepY *= scale;
        }
        smoothedDx += stepX; smoothedDy += stepY;
    }

    private void resetPositionFilter(double rawDx, double rawDy) {
        filterInitialized = false;
        smoothedDx = rawDx; smoothedDy = rawDy;
        outlierStreak = 0;
    }

    private void clearTrackingState() {
        resetPositionFilter(0.0, 0.0);
        activeLabel = "";
        missStreak = 0;
        missStartMs = 0;
        lastDetectWasValid = false;
    }

    // ============================================================
    //  TEST HELPERS
    // ============================================================

    static int passed, failed;

    static void check(String name, boolean cond) {
        if (cond) { passed++; System.out.println("  PASS " + name); }
        else { failed++; System.out.println("  FAIL " + name); }
    }

    static void checkEq(String name, double actual, double expected, double tol) {
        boolean ok = Math.abs(actual - expected) <= tol;
        if (ok) { passed++; System.out.println("  PASS " + name); }
        else { failed++; System.out.printf(
                "  FAIL %s  (expected %.2f +/- %.2f, got %.2f)%n",
                name, expected, tol, actual); }
    }

    static void checkLt(String name, double actual, double limit) {
        boolean ok = actual <= limit;
        if (ok) { passed++; System.out.println("  PASS " + name); }
        else { failed++; System.out.printf(
                "  FAIL %s  (expected <= %.2f, got %.2f)%n",
                name, limit, actual); }
    }

    // ================================================================
    //  Test 1: Jitter reduction
    // ================================================================

    static void testJitterReduction() {
        System.out.println("\n--- Test 1: Jitter reduction ---");
        TemplateMatchCameraTest f = new TemplateMatchCameraTest();

        double base = 100.0;
        double[] warmup = { +3.8, -4.1, +2.0, -3.5, +4.2, -3.9, +1.8, -4.0, +3.3, -2.7 };
        double[] noise  = { +3.1, -3.8, +4.0, -2.9, +3.7, -4.3, +2.5, -3.2, +3.9, -4.0,
                            +2.8, -3.5, +3.6, -3.1, +4.1, -2.6, +3.4, -3.7, +3.0, -3.3 };

        for (int i = 0; i < warmup.length; i++)
            f.feed(base + warmup[i], 0, i * 30, "target", 1.0);

        double maxDev = 0;
        int tOff = warmup.length;
        for (int i = 0; i < noise.length; i++) {
            FilterResult r = f.feed(base + noise[i], 0, (tOff + i) * 30, "target", 1.0);
            double dev = Math.abs(r.dxPx - base);
            if (dev > maxDev) maxDev = dev;
        }

        checkLt("Steady-state jitter <= 1.8 px", maxDev, 1.8);
        System.out.printf("    max deviation = %.2f px%n", maxDev);
    }

    // ================================================================
    //  Test 2: Smooth motion
    // ================================================================

    static void testSmoothMotion() {
        System.out.println("\n--- Test 2: Smooth motion (2 px/frame) ---");
        TemplateMatchCameraTest f = new TemplateMatchCameraTest();

        int misses = 0;
        double rawDx = 0;

        for (int i = 0; i < 100; i++) {
            rawDx += 2.0;
            FilterResult r = f.feed(rawDx, 0, i * 30, "target", 1.0);
            if (!r.valid) misses++;
        }

        check("No frame rejected as outlier", misses == 0);
        check("Tracks input (final error < 30 px)",
                Math.abs(f.smoothedDx - rawDx) < 30.0);
        System.out.printf("    final raw=%.0f  smoothed=%.1f  misses=%d%n",
                rawDx, f.smoothedDx, misses);
    }

    // ================================================================
    //  Test 3: Spike rejection
    // ================================================================

    static void testSpikeRejection() {
        System.out.println("\n--- Test 3: Spike rejection (200 px jump) ---");
        TemplateMatchCameraTest f = new TemplateMatchCameraTest();

        for (int i = 0; i < 10; i++)
            f.feed(50.0 + (Math.random() - 0.5) * 2, 0, i * 30, "target", 1.0);
        double before = f.smoothedDx;

        FilterResult s1 = f.feed(250.0, 0, 300, "target", 1.0);
        check("Spike #1: held prev (valid, same dx)", s1.valid && s1.dxPx == before);
        f.feed(250.0, 0, 330, "target", 1.0);
        f.feed(250.0, 0, 360, "target", 1.0);
        FilterResult s4 = f.feed(250.0, 0, 390, "target", 1.0);
        check("Spike #4: confirmed (valid)", s4.valid);
        check("Spike #4: dx near 250", Math.abs(s4.dxPx - 250.0) < 10.0);

        f.feed(52.0, 0, 420, "target", 1.0);
        f.feed(52.0, 0, 450, "target", 1.0);
        f.feed(52.0, 0, 480, "target", 1.0);
        FilterResult rec = f.feed(52.0, 0, 510, "target", 1.0);
        check("Recovery: confirm at 52", rec.valid);
        check("Recovery near original", Math.abs(rec.dxPx - 52.0) < 10.0);
        System.out.printf("    before=%.1f  spike-4=%.1f  recover=%.1f%n",
                before, s4.dxPx, rec.dxPx);
    }

    // ================================================================
    //  Test 4: Miss hold
    // ================================================================

    static void testMissHold() {
        System.out.println("\n--- Test 4: Miss hold (5 frames) ---");
        TemplateMatchCameraTest f = new TemplateMatchCameraTest();

        for (int i = 0; i < 10; i++)
            f.feed(75.0, 0, i * 30, "target", 1.0);
        check("After detect, valid = true", f.lastDetectWasValid);

        double heldDx = f.smoothedDx;

        for (int i = 0; i < 5; i++)
            f.feed(0, 0, 300 + i * 30, null, 1.0);

        check("After 5 misses, still valid", f.lastDetectWasValid);
        checkEq("Smoothed preserved", f.smoothedDx, heldDx, 0.001);

        for (int i = 0; i < 5; i++)
            f.feed(0, 0, 450 + i * 30, null, 1.0);
        check("After 10 misses, cleared", !f.lastDetectWasValid);

        System.out.printf("    held=%.1f  after 5 miss=%.1f%n", heldDx, f.smoothedDx);
    }

    // ================================================================
    //  Test 5: No label flapping
    // ================================================================

    static void testNoLabelFlapping() {
        System.out.println("\n--- Test 5: No label fluctuation ---");
        TemplateMatchCameraTest f = new TemplateMatchCameraTest();

        FilterResult r1 = f.feed(50.0, 0, 0, "target", 1.0);
        check("Label = 'target'", "target".equals(r1.label));
        check("Valid after first detect", r1.valid);

        for (int i = 0; i < 20; i++) {
            FilterResult r = f.feed(50.0 + (Math.random() - 0.5), 0, i * 30, "target", 1.0);
            check("Label unchanged", "target".equals(r.label));
        }

        System.out.printf("    label: %s%n", f.activeLabel);
    }

    // ================================================================
    //  Test 6: setTarget resets filter
    // ================================================================

    static void testSetTargetReset() {
        System.out.println("\n--- Test 6: setTarget reset ---");
        TemplateMatchCameraTest f = new TemplateMatchCameraTest();

        for (int i = 0; i < 10; i++)
            f.feed(100.0, 0, i * 30, "target_a", 1.0);
        check("Filter initialized after target_a", f.filterInitialized);
        check("Label = target_a", "target_a".equals(f.activeLabel));

        f.clearTrackingState();
        f.activeLabel = "target_b";

        check("Filter not initialized after reset", !f.filterInitialized);
        check("Label = target_b", "target_b".equals(f.activeLabel));

        FilterResult r = f.feed(200.0, 0, 500, "target_b", 1.0);
        check("Detects target_b at new position", r.valid && r.dxPx >= 190);
        check("Filter re-initialized", f.filterInitialized);

        System.out.printf("    smoothed=%.1f  label=%s%n", f.smoothedDx, f.activeLabel);
    }

    static void testCandidateSuppression() {
        System.out.println("\n--- Test 7: Candidate suppression ---");
        java.util.List<org.firstinspires.ftc.teamcode.core.TemplateMatchCamera.Candidate> candidates = new java.util.ArrayList<>();
        candidates.add(new org.firstinspires.ftc.teamcode.core.TemplateMatchCamera.Candidate("right", 0.91,
                new org.opencv.core.Point[] { new org.opencv.core.Point(10, 10), new org.opencv.core.Point(40, 10), new org.opencv.core.Point(40, 40), new org.opencv.core.Point(10, 40) }, 0));
        candidates.add(new org.firstinspires.ftc.teamcode.core.TemplateMatchCamera.Candidate("duplicate", 0.80,
                new org.opencv.core.Point[] { new org.opencv.core.Point(12, 12), new org.opencv.core.Point(42, 12), new org.opencv.core.Point(42, 42), new org.opencv.core.Point(12, 42) }, 1));
        candidates.add(new org.firstinspires.ftc.teamcode.core.TemplateMatchCamera.Candidate("left", 0.72,
                new org.opencv.core.Point[] { new org.opencv.core.Point(100, 10), new org.opencv.core.Point(130, 10), new org.opencv.core.Point(130, 40), new org.opencv.core.Point(100, 40) }, 2));
        java.util.List<org.firstinspires.ftc.teamcode.core.TemplateMatchCamera.Candidate> kept =
                org.firstinspires.ftc.teamcode.core.TemplateMatchCamera.suppressCandidates(candidates, 0.5, 12.0);
        check("Overlap suppression keeps distinct boxes", kept.size() == 2);
        check("Raw confidence retained", kept.get(0).confidence == 0.91);
        check("Labels remain associated", "right".equals(kept.get(0).label) && "left".equals(kept.get(1).label));
    }

    // ================================================================
    //  main
    // ================================================================

    public static void main(String[] args) {
        System.out.println("TemplateMatchCamera filter tests");
        System.out.println("================================");

        passed = 0;
        failed = 0;

        testJitterReduction();
        testSmoothMotion();
        testSpikeRejection();
        testMissHold();
        testNoLabelFlapping();
        testSetTargetReset();
        testCandidateSuppression();

        System.out.println("\n================================");
        System.out.printf("RESULT: %d passed, %d failed%n", passed, failed);

        if (failed > 0) System.exit(1);
    }
}
