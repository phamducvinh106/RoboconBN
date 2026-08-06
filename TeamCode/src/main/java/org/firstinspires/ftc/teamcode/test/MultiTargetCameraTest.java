package org.firstinspires.ftc.teamcode.test;

/**
 * Test offline cho bộ lọc temporal của MultiTargetCamera.
 * Chạy: java MultiTargetCameraTest (không cần phần cứng FTC).
 *
 * Kiểm tra 6 scenario:
 *   1. Jitter reduction            — noise +-4px  → output smoothed <= 1.5px
 *   2. Smooth motion not outlier   — target di chuyển đều, không bị reject
 *   3. Spike rejection             — spike 200px   → handleMiss, phục hồi sau confirm
 *   4. Miss hold                   — miss 5 frame  → giữ kết quả cũ
 *   5. Label hysteresis            — label A/B đổi → cần 3 frame xác nhận mới chuyển
 *   6. Confidence-weighted damping — low-confidence detections have reduced impact
 */
public final class MultiTargetCameraTest {

    // -------------- Bản sao chính xác các hằng số filter --------------
    private static final double OUTLIER_LIMIT_PX       = 45.0;
    private static final int    OUTLIER_CONFIRM_FRAMES = 4;
    private static final int    HOLD_MISS_FRAMES       = 6;
    private static final long   HOLD_MISS_TIMEOUT_MS   = 300;
    private static final int    LABEL_SWITCH_CONFIRM_FRAMES = 3;

    private static final double CENTER_DEADBAND_PX     = 2.0;
    private static final double EMA_ALPHA_SLOW         = 0.08;
    private static final double EMA_ALPHA_FAST         = 0.45;
    private static final double FAST_MOTION_THRESHOLD_PX = 8.0;
    private static final double MAX_OUTPUT_STEP_PX     = 20.0;
    private static final long   MISS_REINIT_TIMEOUT_MS = 250;

    // ------------------- Bộ lọc (bản sao) -------------------

    boolean filterInitialized;
    double  smoothedDx, smoothedDy;
    double  lastRawDx, lastRawDy;

    int     missStreak, outlierStreak;
    long    missStartMs;
    long    lastSuccessfulDetectionMs;
    boolean lastDetectWasValid;

    // Label hysteresis
    String  activeLabel  = "";
    String  pendingLabel = "";
    int     pendingLabelHits;

    // ------------------- API test -------------------

    /**
     * Mô phỏng một frame detect thành công (default confidence = 1.0).
     */
    double feed(double rawDx, double rawDy, long nowMs, String label) {
        return feedFull(rawDx, rawDy, nowMs, label, null, 1.0);
    }

    /**
     * Mô phỏng với confidence tùy chỉnh.
     */
    double feedConfident(double rawDx, double rawDy, long nowMs,
                         String label, double confidence) {
        return feedFull(rawDx, rawDy, nowMs, label, null, confidence);
    }

    double feedFull(double rawDx, double rawDy, long nowMs, String nearestLabel,
                    java.util.Map<String, Double> detections, double confidence) {
        if (nearestLabel == null || nearestLabel.isEmpty()) {
            return Double.NaN;
        }

        String selectedLabel = applyLabelHysteresis(nearestLabel, detections);

        if (selectedLabel == null) {
            handleMiss(nowMs);
            return Double.NaN;
        }

        boolean labelChanged = !selectedLabel.equals(activeLabel);
        if (labelChanged) {
            activeLabel = selectedLabel;
            resetPositionFilter(rawDx, rawDy);
        }

        if (missStreak > 0) {
            long missAgeMs = nowMs - missStartMs;
            if (missAgeMs > MISS_REINIT_TIMEOUT_MS) {
                resetPositionFilter(rawDx, rawDy);
            }
        }

        if (isOutlier(rawDx, rawDy)) {
            outlierStreak++;
            if (outlierStreak < OUTLIER_CONFIRM_FRAMES) {
                handleMiss(nowMs);
                return Double.NaN;
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

        return smoothedDx;
    }

    private String applyLabelHysteresis(String nearestLabel,
                                        java.util.Map<String, Double> detections) {
        if (activeLabel.isEmpty()) {
            clearPendingLabel();
            return nearestLabel;
        }

        if (nearestLabel.equals(activeLabel)) {
            clearPendingLabel();
            return nearestLabel;
        }

        if (nearestLabel.equals(pendingLabel)) {
            pendingLabelHits++;
        } else {
            pendingLabel = nearestLabel;
            pendingLabelHits = 1;
        }

        if (pendingLabelHits >= LABEL_SWITCH_CONFIRM_FRAMES) {
            clearPendingLabel();
            return nearestLabel;
        }

        if (detections != null && detections.containsKey(activeLabel)) {
            return activeLabel;
        }

        return null;
    }

    private void clearPendingLabel() {
        pendingLabel = "";
        pendingLabelHits = 0;
    }

    void miss(long nowMs) {
        if (missStreak == 0) {
            missStartMs = nowMs;
        }
        missStreak++;

        long ageMs = lastSuccessfulDetectionMs <= 0
                ? Long.MAX_VALUE
                : nowMs - lastSuccessfulDetectionMs;

        boolean canHold = lastDetectWasValid
                && missStreak <= HOLD_MISS_FRAMES
                && ageMs <= HOLD_MISS_TIMEOUT_MS;

        if (!canHold) {
            if (lastDetectWasValid || missStreak > HOLD_MISS_FRAMES) {
                clearTrackingState();
            }
        }
    }

    // ------ copy exactly from MultiTargetCamera (with confidence weighting) ------

    private boolean isOutlier(double newDx, double newDy) {
        if (!filterInitialized) return false;
        double diffX = newDx - lastRawDx;
        double diffY = newDy - lastRawDy;
        return Math.hypot(diffX, diffY) > OUTLIER_LIMIT_PX;
    }

    private void updatePositionFilter(double newDx, double newDy, double confidence) {
        if (!filterInitialized) {
            smoothedDx = newDx;
            smoothedDy = newDy;
            filterInitialized = true;
            return;
        }
        double diffX = newDx - smoothedDx;
        double diffY = newDy - smoothedDy;
        double distance = Math.hypot(diffX, diffY);

        double effectiveDeadband =
                CENTER_DEADBAND_PX * (1.0 + 1.0 - confidence);

        if (distance <= effectiveDeadband) return;

        double baseAlpha =
                distance >= FAST_MOTION_THRESHOLD_PX
                        ? EMA_ALPHA_FAST
                        : EMA_ALPHA_SLOW;

        double effectiveAlpha =
                baseAlpha * (0.5 + 0.5 * confidence);

        double stepX = effectiveAlpha * diffX;
        double stepY = effectiveAlpha * diffY;
        double stepDist = Math.hypot(stepX, stepY);

        if (stepDist > MAX_OUTPUT_STEP_PX) {
            double scale = MAX_OUTPUT_STEP_PX / stepDist;
            stepX *= scale;
            stepY *= scale;
        }
        smoothedDx += stepX;
        smoothedDy += stepY;
    }

    private void handleMiss(long nowMs) {
        miss(nowMs);
    }

    private void resetPositionFilter(double rawDx, double rawDy) {
        filterInitialized = false;
        smoothedDx = rawDx;
        smoothedDy = rawDy;
        outlierStreak = 0;
    }

    private void clearTrackingState() {
        resetPositionFilter(0.0, 0.0);
        activeLabel = "";
        clearPendingLabel();
        missStreak = 0;
        missStartMs = 0;
        lastDetectWasValid = false;
    }

    // ================================================================
    //  TEST CASES
    // ================================================================

    static int passed, failed;

    static void check(String name, boolean cond) {
        if (cond) {
            passed++;
            System.out.println("  PASS " + name);
        } else {
            failed++;
            System.out.println("  FAIL " + name);
        }
    }

    static void checkEq(String name, double actual, double expected, double tol) {
        boolean ok = Math.abs(actual - expected) <= tol;
        if (ok) {
            passed++;
            System.out.println("  PASS " + name);
        } else {
            failed++;
            System.out.printf("  FAIL %s  (expected %.2f +/- %.2f, got %.2f)%n",
                    name, expected, tol, actual);
        }
    }

    static void checkLt(String name, double actual, double limit) {
        boolean ok = actual <= limit;
        if (ok) {
            passed++;
            System.out.println("  PASS " + name);
        } else {
            failed++;
            System.out.printf("  FAIL %s  (expected <= %.2f, got %.2f)%n",
                    name, limit, actual);
        }
    }

    // ---------- Test 1: Jitter reduction ----------

    static void testJitterReduction() {
        System.out.println("\n--- Test 1: Jitter reduction ---");
        MultiTargetCameraTest f = new MultiTargetCameraTest();

        double rawBase = 100.0;
        double[] warmupNoise = { +3.8, -4.1, +2.0, -3.5, +4.2, -3.9, +1.8, -4.0, +3.3, -2.7 };
        double[] testNoise   = { +3.1, -3.8, +4.0, -2.9, +3.7, -4.3, +2.5, -3.2, +3.9, -4.0,
                                 +2.8, -3.5, +3.6, -3.1, +4.1, -2.6, +3.4, -3.7, +3.0, -3.3 };

        for (int i = 0; i < warmupNoise.length; i++) {
            f.feed(rawBase + warmupNoise[i], 0, i * 30, "A");
        }

        double maxDeviation = 0.0;
        int tOff = warmupNoise.length;
        for (int i = 0; i < testNoise.length; i++) {
            f.feed(rawBase + testNoise[i], 0, (tOff + i) * 30, "A");
            double dev = Math.abs(f.smoothedDx - rawBase);
            if (dev > maxDeviation) maxDeviation = dev;
        }

        // Alpha 0.08 + deadband 2.0: deviation under 1.5px for +/-4px noise
        check("Steady-state jitter <= 1.5 px", maxDeviation <= 1.5);
        System.out.printf("    max steady-state deviation = %.2f px%n", maxDeviation);
    }

    // ---------- Test 2: Smooth motion ----------

    static void testSmoothMotion() {
        System.out.println("\n--- Test 2: Smooth motion (2 px/frame) ---");
        MultiTargetCameraTest f = new MultiTargetCameraTest();

        int totalMisses = 0;
        double rawDx = 0.0;

        for (int i = 0; i < 100; i++) {
            rawDx += 2.0;
            double out = f.feed(rawDx, 0, i * 30, "A");
            if (Double.isNaN(out)) totalMisses++;
        }

        check("No frame rejected as outlier", totalMisses == 0);
        // With alpha=0.08, lag is ~25px at velocity 2px/frame after 100 frames
        check("Output tracks input (final error < 30 px)",
                Math.abs(f.smoothedDx - rawDx) < 30.0);
        System.out.printf("    final raw=%.0f  smoothed=%.1f  misses=%d%n",
                rawDx, f.smoothedDx, totalMisses);
    }

    // ---------- Test 3: Spike rejection ----------

    static void testSpikeRejection() {
        System.out.println("\n--- Test 3: Spike rejection (200 px jump) ---");
        MultiTargetCameraTest f = new MultiTargetCameraTest();

        for (int i = 0; i < 10; i++) {
            f.feed(50.0 + (Math.random() - 0.5) * 2, 0, i * 30, "A");
        }

        double beforeSpike = f.smoothedDx;

        double outSpike = f.feed(250.0, 0, 300, "A");
        check("Spike frame #1 returns NaN (miss)", Double.isNaN(outSpike));

        f.feed(250.0, 0, 330, "A");
        f.feed(250.0, 0, 360, "A");
        // OUTLIER_CONFIRM_FRAMES=4: frame #4 confirms
        f.feed(250.0, 0, 390, "A");

        // Recovery (3 more misses, then confirm back)
        f.feed(52.0, 0, 420, "A");
        f.feed(52.0, 0, 450, "A");
        f.feed(52.0, 0, 480, "A");
        f.feed(52.0, 0, 510, "A");

        double outRecover = f.feed(52.0, 0, 540, "A");
        check("After recovery, output near original",
                Math.abs(outRecover - beforeSpike) < 5.0);
        System.out.printf("    before spike=%.1f  recover=%.1f%n", beforeSpike, outRecover);
    }

    // ---------- Test 4: Miss hold ----------

    static void testMissHold() {
        System.out.println("\n--- Test 4: Miss hold (5 frames) ---");
        MultiTargetCameraTest f = new MultiTargetCameraTest();

        for (int i = 0; i < 10; i++) {
            f.feed(75.0, 0, i * 30, "A");
        }

        boolean heldBefore = f.lastDetectWasValid;
        check("After detect, lastDetectWasValid = true", heldBefore);

        double heldDx = f.smoothedDx;

        for (int i = 0; i < 5; i++) {
            f.miss(300 + i * 30);
        }

        boolean heldAfter = f.lastDetectWasValid;
        double heldAfterDx = f.smoothedDx;

        check("After 5 misses, lastDetectWasValid still true", heldAfter);
        checkEq("smoothed preserved during miss", heldAfterDx, heldDx, 0.001);

        for (int i = 0; i < 5; i++) {
            f.miss(450 + i * 30);
        }

        check("After 10 misses, state cleared", !f.lastDetectWasValid);

        System.out.printf("    held dx=%.1f  after 5 miss=%.1f%n", heldDx, heldAfterDx);
    }

    // ---------- Test 5: Label hysteresis ----------

    static void testLabelHysteresis() {
        System.out.println("\n--- Test 5: Label hysteresis (2 targets A, B) ---");
        MultiTargetCameraTest f = new MultiTargetCameraTest();

        java.util.Map<String, Double> dets1 = new java.util.HashMap<>();
        dets1.put("A", 50.0);
        dets1.put("B", 200.0);

        java.util.Map<String, Double> dets2 = new java.util.HashMap<>();
        dets2.put("A", 200.0);
        dets2.put("B", 48.0);

        f.feedFull(50.0, 0, 0, "A", dets1, 1.0);
        check("Frame 1: activeLabel = A", f.activeLabel.equals("A"));

        f.feedFull(48.0, 0, 30, "B", dets2, 1.0);
        check("Frame 2: activeLabel still A (B not confirmed)", f.activeLabel.equals("A"));

        f.feedFull(48.0, 0, 60, "B", dets2, 1.0);
        check("Frame 3: activeLabel still A (hits=2/3)", f.activeLabel.equals("A"));

        f.feedFull(48.0, 0, 90, "B", dets2, 1.0);
        check("Frame 4: activeLabel = B (confirmed)", f.activeLabel.equals("B"));

        f.feedFull(50.0, 0, 120, "A", dets1, 1.0);
        check("Frame 5: activeLabel still B (A not yet confirmed)", f.activeLabel.equals("B"));

        System.out.printf("    final label: %s%n", f.activeLabel);
    }

    // ---------- Test 6: Confidence-weighted damping ----------

    static void testConfidenceWeighting() {
        System.out.println("\n--- Test 6: Confidence-weighted EMA ---");

        // Test A: high confidence response
        MultiTargetCameraTest fHigh = new MultiTargetCameraTest();
        for (int i = 0; i < 15; i++) {
            fHigh.feedConfident(100.0, 0, i * 30, "A", 1.0);
        }
        double baseHigh = fHigh.smoothedDx;

        // Step to 120 with confidence 1.0, measure 1-frame response
        fHigh.feedConfident(120.0, 0, 500, "A", 1.0);
        double moveHigh = Math.abs(fHigh.smoothedDx - baseHigh);

        // Test B: low confidence response (same starting point)
        MultiTargetCameraTest fLow = new MultiTargetCameraTest();
        for (int i = 0; i < 15; i++) {
            fLow.feedConfident(100.0, 0, i * 30, "A", 1.0);
        }
        double baseLow = fLow.smoothedDx;

        // Step to 120 with confidence 0.4
        fLow.feedConfident(120.0, 0, 500, "A", 0.4);
        double moveLow = Math.abs(fLow.smoothedDx - baseLow);

        // Low confidence should produce < 85% of the high-confidence response
        check("Low-confidence step < 85% of high-conf step",
                moveLow < moveHigh * 0.85);

        System.out.printf("    high-conf: %.2f -> %.2f  (move %.2f)%n",
                baseHigh, fHigh.smoothedDx, moveHigh);
        System.out.printf("    low-conf:  %.2f -> %.2f (move %.2f)%n",
                baseLow, fLow.smoothedDx, moveLow);
    }

    // ================================================================
    //  main
    // ================================================================

    public static void main(String[] args) {
        System.out.println("MultiTargetCamera filter tests (stability-tuned)");
        System.out.println("==================================================");

        passed = 0;
        failed = 0;

        testJitterReduction();
        testSmoothMotion();
        testSpikeRejection();
        testMissHold();
        testLabelHysteresis();
        testConfidenceWeighting();

        System.out.println("\n==================================================");
        System.out.printf("RESULT: %d passed, %d failed%n", passed, failed);

        if (failed > 0) {
            System.exit(1);
        }
    }
}
