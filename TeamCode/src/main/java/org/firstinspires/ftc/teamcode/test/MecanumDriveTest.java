package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.core.PidController;

/**
 * Test offline cho mô phỏng Mecanum drive.
 * Chạy: java MecanumDriveTest (không cần phần cứng FTC).
 *
 * File này sao chép logic cốt lõi cúa MecanumDrive
 * (kinematics, PID, script, state machine) dê test không cần
 * FTC SDK. Xem MecanumDrive.java dê biết chi tiết.
 *
 * Kiểm tra 7 scenario:
 *   1. Kinematics forward
 *   2. Kinematics strafe
 *   3. Kinematics rotate
 *   4. PID converge
 *   5. PID hold
 *   6. Script execution
 *   7. Script timeout
 */
public final class MecanumDriveTest {

    // ============================================================
    //  Các hằng số — copy từ MecanumDrive.java
    // ============================================================

    private static final double POS_KP   = 0.05;
    private static final double POS_KI   = 0.012;
    private static final double POS_KD   = 0.025;
    private static final double HEAD_KP  = 0.04;
    private static final double HEAD_KI  = 0.006;
    private static final double HEAD_KD  = 0.012;
    private static final double MAX_TRANSLATIONAL = 0.75;
    private static final double MAX_ROTATIONAL    = 0.45;
    private static final double TOLERANCE_CM      = 1.5;
    private static final double TOLERANCE_DEG     = 2.5;

    private static final double MAX_SPEED_CM_S = 80.0;
    private static final double MAX_ROT_DEG_S  = 120.0;

    // ============================================================
    //  FakeOdometry — mô phỏng chuyển dộng robot từ công suất motor
    // ============================================================

    static final class FakeOdometry {

        double xCm, yCm, headingDeg;

        /**
         * Mecanum inverse → forward kinematics nguọc:
         *   forward = (fl + fr + bl + br) / 4
         *   strafe  = (fl - fr - bl + br) / 4
         *   rotate  = (fl - fr + bl - br) / 4
         *
         * Trong robot frame:
         *   forward = chuyên dộng doc (+Y robot)
         *   strafe  = chuyên dộng ngang (+X robot)
         *
         * Chuyên sang global frame với heading.
         */
        void step(double fl, double fr, double bl, double br, double dtSec) {
            double forward = (fl + fr + bl + br) / 4.0;
            double strafe  = (fl - fr - bl + br) / 4.0;
            double rotate  = (fl - fr + bl - br) / 4.0;

            double headingRad = Math.toRadians(headingDeg);
            double cos = Math.cos(headingRad);
            double sin = Math.sin(headingRad);

            // Robot frame: (strafe, forward) → global frame
            double gx = strafe * cos - forward * sin;
            double gy = strafe * sin + forward * cos;

            xCm += MAX_SPEED_CM_S * gx * dtSec;
            yCm += MAX_SPEED_CM_S * gy * dtSec;
            headingDeg += MAX_ROT_DEG_S * rotate * dtSec;

            if (headingDeg > 180)  headingDeg -= 360;
            if (headingDeg < -180) headingDeg += 360;
        }
    }

    // ============================================================
    //  State machine (copy từ MecanumDrive)
    // ============================================================

    enum DriveState {
        IDLE, MOVING, HOLDING, SCRIPT_RUNNING, SCRIPT_DONE,
        SCRIPT_STEP_RUNNING, SCRIPT_STEP_DONE
    }

    DriveState state = DriveState.IDLE;

    // ============================================================
    //  Mini Mecanum logic (copy từ MecanumDrive)
    // ============================================================

    final PidController xPid = new PidController(POS_KP, POS_KI, POS_KD,
            -MAX_TRANSLATIONAL, MAX_TRANSLATIONAL);
    final PidController yPid = new PidController(POS_KP, POS_KI, POS_KD,
            -MAX_TRANSLATIONAL, MAX_TRANSLATIONAL);
    final PidController hPid = new PidController(HEAD_KP, HEAD_KI, HEAD_KD,
            -MAX_ROTATIONAL, MAX_ROTATIONAL);

    double targetXCm, targetYCm, targetHeadingDeg;

    double lastFlPower, lastFrPower, lastBlPower, lastBrPower;
    double lastRobotForward, lastRobotStrafe, lastRobotRotate;

    // ---- Script ----
    java.util.List<ScriptStep> scriptSteps = new java.util.ArrayList<>();
    int scriptIndex;
    long scriptStepStartMs;

    // ============================================================
    //  Drive API (copy từ MecanumDrive)
    // ============================================================

    void driveFieldCentric(double fwd, double strafe, double rot, FakeOdometry odo) {
        state = DriveState.IDLE;
        double headingRad = Math.toRadians(odo.headingDeg);
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);
        double robotFwd = fwd * cos - strafe * sin;
        double robotStr = fwd * sin + strafe * cos;
        applyMecanumPowers(robotFwd, robotStr, rot);
    }

    void goToPosition(double x, double y, double hDeg) {
        targetXCm = x;
        targetYCm = y;
        targetHeadingDeg = hDeg;
        xPid.reset();
        yPid.reset();
        hPid.reset();
        state = DriveState.MOVING;
    }

    void loadScript(String json) {
        scriptSteps.clear();
        scriptIndex = 0;
        if (json == null || json.trim().isEmpty()) return;
        String trimmed = json.trim();
        if (!trimmed.startsWith("[")) return;
        String inner = trimmed.substring(1, trimmed.length() - 1).trim();
        if (inner.isEmpty()) return;
        int depth = 0, start = -1;
        for (int i = 0; i < inner.length(); i++) {
            char c = inner.charAt(i);
            if (c == '{') { if (depth == 0) start = i; depth++; }
            else if (c == '}') { depth--; if (depth == 0 && start >= 0) parseStep(inner.substring(start, i + 1)); }
        }
    }

    private void parseStep(String obj) {
        double x = 0, y = 0, h = 0;
        long timeout = 5000;
        String inner = obj.substring(1, obj.length() - 1);
        java.util.List<String> fields = splitJsonFields(inner);
        for (String field : fields) {
            int colon = field.indexOf(':');
            if (colon < 0) continue;
            String key = field.substring(0, colon).replace("\"", "").trim();
            String val = field.substring(colon + 1).replace("\"", "").trim();
            try {
                switch (key) {
                    case "x":       x = Double.parseDouble(val); break;
                    case "y":       y = Double.parseDouble(val); break;
                    case "h":       h = Double.parseDouble(val); break;
                    case "timeout": timeout = Long.parseLong(val); break;
                }
            } catch (NumberFormatException ignored) {}
        }
        scriptSteps.add(new ScriptStep(x, y, h, timeout));
    }

    private java.util.List<String> splitJsonFields(String inner) {
        java.util.List<String> res = new java.util.ArrayList<>();
        int start = 0, brace = 0;
        boolean inStr = false;
        for (int i = 0; i < inner.length(); i++) {
            char c = inner.charAt(i);
            if (c == '"' && (i == 0 || inner.charAt(i - 1) != '\\')) inStr = !inStr;
            if (!inStr) { if (c == '{') brace++; if (c == '}') brace--; }
            if (!inStr && brace == 0 && c == ',') {
                res.add(inner.substring(start, i).trim());
                start = i + 1;
            }
        }
        if (start < inner.length()) res.add(inner.substring(start).trim());
        return res;
    }

    void startScript() {
        if (scriptSteps.isEmpty()) { state = DriveState.SCRIPT_DONE; return; }
        scriptIndex = 0;
        scriptStepStartMs = System.currentTimeMillis();
        xPid.reset(); yPid.reset(); hPid.reset();
        state = DriveState.SCRIPT_RUNNING;
    }

    void stop() { state = DriveState.IDLE; stopMotors(); }

    boolean executeNextStep() {
        if (scriptSteps.isEmpty() || scriptIndex >= scriptSteps.size()) {
            state = DriveState.SCRIPT_DONE;
            return false;
        }
        scriptStepStartMs = System.currentTimeMillis();
        xPid.reset(); yPid.reset(); hPid.reset();
        state = DriveState.SCRIPT_STEP_RUNNING;
        return true;
    }

    void update(FakeOdometry odo) {
        long nowMs = System.currentTimeMillis();
        switch (state) {
            case MOVING: case HOLDING: updatePositionControl(odo); break;
            case SCRIPT_RUNNING:      updateScript(odo, nowMs);    break;
            case SCRIPT_STEP_RUNNING: updateScriptStep(odo, nowMs); break;
            default: stopMotors(); break;
        }
    }

    // ============================================================
    //  Core logic (copy từ MecanumDrive)
    // ============================================================

    private void updatePositionControl(FakeOdometry odo) {
        double robotStrafe  = xPid.calculate(targetXCm, odo.xCm);
        double robotForward = yPid.calculate(targetYCm, odo.yCm);
        double rotate       = hPid.calculate(targetHeadingDeg, odo.headingDeg);

        double headingRad = Math.toRadians(odo.headingDeg);
        double cos = Math.cos(headingRad);
        double sin = Math.sin(headingRad);
        double fieldFwd = robotForward * cos + robotStrafe * sin;
        double fieldStr = -robotForward * sin + robotStrafe * cos;

        lastRobotForward = fieldFwd;
        lastRobotStrafe  = fieldStr;
        lastRobotRotate  = rotate;

        applyMecanumPowers(fieldFwd, fieldStr, rotate);

        if (xPid.atSetpoint(TOLERANCE_CM)
                && yPid.atSetpoint(TOLERANCE_CM)
                && hPid.atSetpoint(TOLERANCE_DEG)) {
            state = DriveState.HOLDING;
        } else if (state == DriveState.HOLDING) {
            state = DriveState.MOVING;
        }
    }

    private void updateScript(FakeOdometry odo, long nowMs) {
        if (scriptIndex >= scriptSteps.size()) {
            state = DriveState.SCRIPT_DONE;
            stopMotors();
            return;
        }
        ScriptStep step = scriptSteps.get(scriptIndex);
        double rs = xPid.calculate(step.x, odo.xCm);
        double rf = yPid.calculate(step.y, odo.yCm);
        double rot = hPid.calculate(step.h, odo.headingDeg);

        double hr = Math.toRadians(odo.headingDeg);
        double cos = Math.cos(hr);
        double sin = Math.sin(hr);
        double ff = rf * cos + rs * sin;
        double fs = -rf * sin + rs * cos;

        lastRobotForward = ff;
        lastRobotStrafe  = fs;
        lastRobotRotate  = rot;
        applyMecanumPowers(ff, fs, rot);

        boolean atTarget = xPid.atSetpoint(TOLERANCE_CM)
                && yPid.atSetpoint(TOLERANCE_CM)
                && hPid.atSetpoint(TOLERANCE_DEG);
        long elapsed = nowMs - scriptStepStartMs;

        if (atTarget || elapsed >= step.timeoutMs) {
            scriptIndex++;
            scriptStepStartMs = nowMs;
            xPid.reset(); yPid.reset(); hPid.reset();
        }
    }

    private void updateScriptStep(FakeOdometry odo, long nowMs) {
        if (scriptIndex >= scriptSteps.size()) {
            state = DriveState.SCRIPT_DONE;
            stopMotors();
            return;
        }
        ScriptStep step = scriptSteps.get(scriptIndex);
        double rs = xPid.calculate(step.x, odo.xCm);
        double rf = yPid.calculate(step.y, odo.yCm);
        double rot = hPid.calculate(step.h, odo.headingDeg);

        double hr = Math.toRadians(odo.headingDeg);
        double cos = Math.cos(hr);
        double sin = Math.sin(hr);
        double ff = rf * cos + rs * sin;
        double fs = -rf * sin + rs * cos;

        lastRobotForward = ff;
        lastRobotStrafe  = fs;
        lastRobotRotate  = rot;
        applyMecanumPowers(ff, fs, rot);

        boolean atTarget = xPid.atSetpoint(TOLERANCE_CM)
                && yPid.atSetpoint(TOLERANCE_CM)
                && hPid.atSetpoint(TOLERANCE_DEG);
        long elapsed = nowMs - scriptStepStartMs;

        if (atTarget || elapsed >= step.timeoutMs) {
            scriptIndex++;
            xPid.reset(); yPid.reset(); hPid.reset();
            if (scriptIndex >= scriptSteps.size()) {
                state = DriveState.SCRIPT_DONE;
                stopMotors();
            } else {
                state = DriveState.SCRIPT_STEP_DONE;
                stopMotors();
            }
        }
    }

    private void applyMecanumPowers(double fwd, double str, double rot) {
        double fl = fwd + str + rot;
        double fr = fwd - str - rot;
        double bl = fwd - str + rot;
        double br = fwd + str - rot;
        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br)));
        if (max > 1.0) { fl /= max; fr /= max; bl /= max; br /= max; }
        lastFlPower = fl; lastFrPower = fr;
        lastBlPower = bl; lastBrPower = br;
    }

    private void stopMotors() {
        lastFlPower = lastFrPower = lastBlPower = lastBrPower = 0;
    }

    // ============================================================
    //  ScriptStep (copy từ inner class)
    // ============================================================

    static final class ScriptStep {
        final double x, y, h;
        final long timeoutMs;
        ScriptStep(double x, double y, double h, long t) {
            this.x = x; this.y = y; this.h = h; this.timeoutMs = t;
        }
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
                "  FAIL %s  (expected %.2f +/- %.2f, got %.2f)%n", name, expected, tol, actual); }
    }

    // ================================================================
    //  Test 1: Kinematics forward
    // ================================================================

    static void testKinematicsForward() {
        System.out.println("\n--- Test 1: Kinematics forward ---");
        FakeOdometry odo = new FakeOdometry();
        MecanumDriveTest drive = new MecanumDriveTest();
        drive.driveFieldCentric(1.0, 0.0, 0.0, odo);

        check("fl = 1", drive.lastFlPower == 1.0);
        check("fr = 1", drive.lastFrPower == 1.0);
        check("bl = 1", drive.lastBlPower == 1.0);
        check("br = 1", drive.lastBrPower == 1.0);

        odo.step(drive.lastFlPower, drive.lastFrPower,
                drive.lastBlPower, drive.lastBrPower, 0.02);
        checkEq("X stays 0", odo.xCm, 0.0, 0.01);
        check("Y increases", odo.yCm > 0);
        System.out.printf("    pos: (%.2f, %.2f)%n", odo.xCm, odo.yCm);
    }

    // ================================================================
    //  Test 2: Kinematics strafe
    // ================================================================

    static void testKinematicsStrafe() {
        System.out.println("\n--- Test 2: Kinematics strafe right ---");
        FakeOdometry odo = new FakeOdometry();
        MecanumDriveTest drive = new MecanumDriveTest();
        drive.driveFieldCentric(0.0, 1.0, 0.0, odo);

        check("fl = +1", drive.lastFlPower == 1.0);
        check("fr = -1", drive.lastFrPower == -1.0);
        check("bl = -1", drive.lastBlPower == -1.0);
        check("br = +1", drive.lastBrPower == 1.0);

        odo.step(drive.lastFlPower, drive.lastFrPower,
                drive.lastBlPower, drive.lastBrPower, 0.02);
        checkEq("Y stays 0", odo.yCm, 0.0, 0.01);
        check("X increases", odo.xCm > 0);
        System.out.printf("    pos: (%.2f, %.2f)%n", odo.xCm, odo.yCm);
    }

    // ================================================================
    //  Test 3: Kinematics rotate
    // ================================================================

    static void testKinematicsRotate() {
        System.out.println("\n--- Test 3: Kinematics rotate ---");
        FakeOdometry odo = new FakeOdometry();
        MecanumDriveTest drive = new MecanumDriveTest();
        drive.driveFieldCentric(0.0, 0.0, 1.0, odo);

        check("fl = +1", drive.lastFlPower == 1.0);
        check("fr = -1", drive.lastFrPower == -1.0);
        check("bl = +1", drive.lastBlPower == 1.0);
        check("br = -1", drive.lastBrPower == -1.0);

        odo.step(drive.lastFlPower, drive.lastFrPower,
                drive.lastBlPower, drive.lastBrPower, 0.02);
        checkEq("X stays 0", odo.xCm, 0.0, 0.01);
        checkEq("Y stays 0", odo.yCm, 0.0, 0.01);
        check("Heading increases", odo.headingDeg > 0);
        System.out.printf("    heading: %.1f deg%n", odo.headingDeg);
    }

    // ================================================================
    //  Test 4: PID converge
    // ================================================================

    static void testPidConverge() {
        System.out.println("\n--- Test 4: PID converge (50, 0, 0) ---");
        FakeOdometry odo = new FakeOdometry();
        MecanumDriveTest drive = new MecanumDriveTest();
        drive.goToPosition(50.0, 0.0, 0.0);

        int frames = 0;
        for (int i = 0; i < 600; i++) {
            drive.update(odo);
            odo.step(drive.lastFlPower, drive.lastFrPower,
                    drive.lastBlPower, drive.lastBrPower, 0.02);
            frames = i;
            if (Math.abs(odo.xCm - 50.0) < 0.5
                    && Math.abs(odo.yCm) < 0.5
                    && drive.state == DriveState.HOLDING) break;
        }

        checkEq("X near 50", odo.xCm, 50.0, 1.0);
        checkEq("Y near 0", odo.yCm, 0.0, 1.0);
        check("Settles to HOLDING", drive.state == DriveState.HOLDING);
        System.out.printf("    pos: (%.2f, %.2f)  frames=%d  state=%s%n",
                odo.xCm, odo.yCm, frames, drive.state);
    }

    // ================================================================
    //  Test 5: PID hold
    // ================================================================

    static void testPidHold() {
        System.out.println("\n--- Test 5: PID hold (offset correction) ---");
        FakeOdometry odo = new FakeOdometry();
        MecanumDriveTest drive = new MecanumDriveTest();
        drive.goToPosition(50.0, 0.0, 0.0);

        for (int i = 0; i < 500; i++) {
            drive.update(odo);
            odo.step(drive.lastFlPower, drive.lastFrPower,
                    drive.lastBlPower, drive.lastBrPower, 0.02);
            if (drive.state == DriveState.HOLDING) break;
        }
        check("Reached HOLDING", drive.state == DriveState.HOLDING);

        odo.xCm = 55.0;
        drive.update(odo);
        // Pushed right → x need to go left → robotStrafe < 0
        // fl = fwd + strafe + rot ≈ 0 + (-) + 0 = negative
        boolean correcting = drive.lastFlPower < 0;
        check("Motors correcting (fl < 0)", correcting);

        for (int i = 0; i < 300; i++) {
            drive.update(odo);
            odo.step(drive.lastFlPower, drive.lastFrPower,
                    drive.lastBlPower, drive.lastBrPower, 0.02);
            if (odo.xCm <= 50.2) break;
        }
        checkEq("Returned near target", odo.xCm, 50.0, 1.0);
        System.out.printf("    after push: (%.2f, %.2f)%n", odo.xCm, odo.yCm);
    }

    // ================================================================
    //  Test 6: Script execution
    // ================================================================

    static void testScriptExecution() {
        System.out.println("\n--- Test 6: Script execution (2 steps) ---");
        FakeOdometry odo = new FakeOdometry();
        MecanumDriveTest drive = new MecanumDriveTest();

        String json =
                "[{\"x\":30,\"y\":0,\"h\":0,\"timeout\":5000}," +
                " {\"x\":30,\"y\":30,\"h\":0,\"timeout\":5000}]";
        drive.loadScript(json);
        drive.startScript();
        check("State = SCRIPT_RUNNING", drive.state == DriveState.SCRIPT_RUNNING);

        for (int i = 0; i < 1000; i++) {
            drive.update(odo);
            odo.step(drive.lastFlPower, drive.lastFrPower,
                    drive.lastBlPower, drive.lastBrPower, 0.02);
            if (drive.state == DriveState.SCRIPT_DONE) break;
        }

        check("State = SCRIPT_DONE", drive.state == DriveState.SCRIPT_DONE);
        checkEq("X near 30", odo.xCm, 30.0, 2.0);
        checkEq("Y near 30", odo.yCm, 30.0, 2.0);
        System.out.printf("    final: (%.2f, %.2f)  state=%s%n",
                odo.xCm, odo.yCm, drive.state);
    }

    // ================================================================
    //  Test 7: Script timeout
    // ================================================================

    static void testScriptTimeout() {
        System.out.println("\n--- Test 7: Script timeout ---");
        FakeOdometry odo = new FakeOdometry();
        MecanumDriveTest drive = new MecanumDriveTest();

        // Step 1: timeout 100ms (impossible to reach x=200)
        String json =
                "[{\"x\":200,\"y\":0,\"h\":0,\"timeout\":100}," +
                " {\"x\":0,\"y\":0,\"h\":0,\"timeout\":5000}]";
        drive.loadScript(json);
        drive.startScript();

        // Run past timeout
        for (int i = 0; i < 300; i++) {
            drive.update(odo);
            odo.step(drive.lastFlPower, drive.lastFrPower,
                    drive.lastBlPower, drive.lastBrPower, 0.02);
        }

        check("Advanced past step 1 (scriptIndex > 0)",
                drive.scriptIndex > 0
                || drive.state == DriveState.SCRIPT_DONE);
        System.out.printf("    pos: (%.2f, %.2f)  idx=%d/%d  state=%s%n",
                odo.xCm, odo.yCm,
                drive.scriptIndex, drive.scriptSteps.size(),
                drive.state);
        System.out.printf("    (robot moved %.1f cm before timeout)%n", odo.xCm);
    }

    // ================================================================
    //  Test 8: Step-by-step script
    // ================================================================

    static void testStepByStepScript() {
        System.out.println("\n--- Test 8: Step-by-step script ---");
        FakeOdometry odo = new FakeOdometry();
        MecanumDriveTest drive = new MecanumDriveTest();

        String json =
                "[{\"x\":25,\"y\":0,\"h\":0,\"timeout\":5000}," +
                " {\"x\":25,\"y\":25,\"h\":0,\"timeout\":5000}]";
        drive.loadScript(json);

        // --- Step 1 ---
        boolean ok1 = drive.executeNextStep();
        check("executeNextStep #1 returns true", ok1);
        check("State = SCRIPT_STEP_RUNNING after step 1",
                drive.state == DriveState.SCRIPT_STEP_RUNNING);

        for (int i = 0; i < 500; i++) {
            drive.update(odo);
            odo.step(drive.lastFlPower, drive.lastFrPower,
                    drive.lastBlPower, drive.lastBrPower, 0.02);
            if (drive.state != DriveState.SCRIPT_STEP_RUNNING) break;
        }

        check("Step 1 done → SCRIPT_STEP_DONE",
                drive.state == DriveState.SCRIPT_STEP_DONE);
        checkEq("After step 1: X near 25", odo.xCm, 25.0, 2.0);
        checkEq("After step 1: Y = 0", odo.yCm, 0.0, 1.0);

        // --- Step 2 ---
        boolean ok2 = drive.executeNextStep();
        check("executeNextStep #2 returns true", ok2);
        check("State = SCRIPT_STEP_RUNNING after step 2",
                drive.state == DriveState.SCRIPT_STEP_RUNNING);

        for (int i = 0; i < 500; i++) {
            drive.update(odo);
            odo.step(drive.lastFlPower, drive.lastFrPower,
                    drive.lastBlPower, drive.lastBrPower, 0.02);
            if (drive.state != DriveState.SCRIPT_STEP_RUNNING) break;
        }

        check("Step 2 done → SCRIPT_DONE",
                drive.state == DriveState.SCRIPT_DONE);
        checkEq("After step 2: X near 25", odo.xCm, 25.0, 2.0);
        checkEq("After step 2: Y near 25", odo.yCm, 25.0, 2.0);

        // --- No more steps ---
        boolean ok3 = drive.executeNextStep();
        check("executeNextStep #3 returns false (no more steps)", !ok3);
        check("State = SCRIPT_DONE after exhausting steps",
                drive.state == DriveState.SCRIPT_DONE);

        System.out.printf("    final: (%.2f, %.2f)  state=%s%n",
                odo.xCm, odo.yCm, drive.state);
    }

    // ================================================================
    //  main
    // ================================================================

    public static void main(String[] args) {
        System.out.println("MecanumDrive offline tests");
        System.out.println("==========================");

        passed = 0;
        failed = 0;

        testKinematicsForward();
        testKinematicsStrafe();
        testKinematicsRotate();
        testPidConverge();
        testPidHold();
        testScriptExecution();
        testScriptTimeout();
        testStepByStepScript();

        System.out.println("\n==========================");
        System.out.printf("RESULT: %d passed, %d failed%n", passed, failed);

        if (failed > 0) {
            System.exit(1);
        }
    }
}
