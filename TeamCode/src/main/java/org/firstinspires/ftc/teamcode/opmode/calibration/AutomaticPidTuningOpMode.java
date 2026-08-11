package org.firstinspires.ftc.teamcode.opmode.calibration;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.odometry.Localizer;
import org.firstinspires.ftc.teamcode.core.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.core.RobotHardware;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Automatic PID Tuning", group = "Calibration")
public final class AutomaticPidTuningOpMode extends LinearOpMode {
    private static final double OUTER_HALF_SQUARE_CM = 100.0;
    private static final double SAFE_MARGIN_CM = 10.0;
    private static final double SAFE_HALF_SQUARE_CM = OUTER_HALF_SQUARE_CM - SAFE_MARGIN_CM;
    private static final double LIVE_WARNING_THRESHOLD_CM = 90.0;
    private static final double LIVE_ABORT_THRESHOLD_CM = 95.0;
    private static final double POWER_LIMIT = 0.20;
    private static final double HEADING_POWER_LIMIT = 0.20;
    private static final double TOLERANCE_CM = 1.5;
    private static final double TOLERANCE_DEG = 2.5;
    private static final int SETTLE_LOOPS = 8;
    private static final long TRIAL_TIMEOUT_MS = 30000;
    private static final long BETWEEN_TRIALS_MS = 150;

    private static final class Gains {
        final String name;
        final double posKp, posKi, posKd, headKp, headKi, headKd;
        Gains(String name, double posKp, double posKi, double posKd,
              double headKp, double headKi, double headKd) {
            this.name = name;
            this.posKp = posKp; this.posKi = posKi; this.posKd = posKd;
            this.headKp = headKp; this.headKi = headKi; this.headKd = headKd;
        }
    }

    private static final class Trial {
        final String id;
        final double x, y, heading;
        final Gains gains;
        Trial(String id, double x, double y, double heading, Gains gains) {
            this.id = id; this.x = x; this.y = y; this.heading = heading; this.gains = gains;
        }
    }

    private static final class Result {
        String id, state;
        double score, finalXError, finalYError, finalHeadingError, peakError, elapsedSec;
        double overshootCm, settlingJitterCm;
        boolean warned, boundaryAborted, timeout;
        Result(String id) { this.id = id; }
    }

    private final ArrayDeque<Result> recentResults = new ArrayDeque<>();
    private Result best;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotHardware robot = new RobotHardware(hardwareMap);
        Localizer localizer = robot.localizer;
        MecanumDrive drive = new MecanumDrive(
                hardwareMap, "leftfront", "rightfront", "leftback", "rightback", localizer);
        List<Trial> trials = buildTrials();
        showSetup(trials.size());
        waitForStart();
        if (isStopRequested()) return;

        try {
            for (int i = 0; i < trials.size() && opModeIsActive(); i++) {
                Result result = runTrial(i, trials.get(i), localizer, drive, trials.size());
                recentResults.addLast(result);
                if (recentResults.size() > 4) recentResults.removeFirst();
                if (best == null || result.score < best.score) best = result;
                sleep(BETWEEN_TRIALS_MS);
            }
            drive.stop();
            telemetry.addLine("MATRIX COMPLETE — gains remain telemetry-only");
            publishResults();
            telemetry.update();
            while (opModeIsActive()) sleep(100);
        } finally {
            drive.stop();
        }
    }

    private Result runTrial(int index, Trial trial, Localizer localizer,
                            MecanumDrive drive, int total) throws InterruptedException {
        Result result = new Result(trial.id);
        drive.stop();
        localizer.resetPoseAndHeading();
        drive.setPositionGains(trial.gains.posKp, trial.gains.posKi, trial.gains.posKd);
        drive.setHeadingGains(trial.gains.headKp, trial.gains.headKi, trial.gains.headKd);
        drive.setTolerance(TOLERANCE_CM, TOLERANCE_DEG);
        drive.setPowerLimits(POWER_LIMIT, HEADING_POWER_LIMIT);
        localizer.update();
        double clampedX = clampFinite(trial.x, -SAFE_HALF_SQUARE_CM, SAFE_HALF_SQUARE_CM);
        double clampedY = clampFinite(trial.y, -SAFE_HALF_SQUARE_CM, SAFE_HALF_SQUARE_CM);
        boolean clamped = clampedX != trial.x || clampedY != trial.y;
        drive.goToPosition(clampedX, clampedY, trial.heading);
        long started = System.nanoTime();
        int settledLoops = 0;
        double peakError = 0.0;
        double overshootCm = 0.0;
        double previousError = Double.POSITIVE_INFINITY;
        double settlingJitterCm = 0.0;
        String state = "RUNNING";

        while (opModeIsActive()) {
            localizer.update();
            drive.update();
            double x = localizer.getX();
            double y = localizer.getY();
            double h = localizer.getHeadingDeg();
            double clearance = SAFE_HALF_SQUARE_CM - Math.max(Math.abs(x), Math.abs(y));
            double translationalError = Math.hypot(clampedX - x, clampedY - y);
            double headingError = Math.abs(MecanumDrive.wrapHeadingError(trial.heading - h));
            peakError = Math.max(peakError, translationalError);
            if (translationalError > previousError) overshootCm = Math.max(overshootCm, translationalError);
            if (settledLoops > 0) settlingJitterCm = Math.max(settlingJitterCm, Math.abs(translationalError - previousError));
            previousError = translationalError;
            boolean inTolerance = drive.atTarget(TOLERANCE_CM, TOLERANCE_DEG);
            if (inTolerance) settledLoops++; else settledLoops = 0;
            if (Math.max(Math.abs(x), Math.abs(y)) >= LIVE_WARNING_THRESHOLD_CM) result.warned = true;
            if (Math.max(Math.abs(x), Math.abs(y)) >= LIVE_ABORT_THRESHOLD_CM) {
                result.boundaryAborted = true;
                state = "BOUNDARY_ABORT";
                drive.stop();
                break;
            }
            long elapsedMs = (System.nanoTime() - started) / 1_000_000L;
            if (settledLoops >= SETTLE_LOOPS) {
                state = "SETTLED";
                drive.stop();
                result.elapsedSec = elapsedMs / 1000.0;
                break;
            }
            if (elapsedMs >= TRIAL_TIMEOUT_MS) {
                result.timeout = true;
                state = "TIMEOUT";
                drive.stop();
                result.elapsedSec = elapsedMs / 1000.0;
                break;
            }
            publishTrial(index, total, trial, clampedX, clampedY, clamped, localizer,
                    drive, state, elapsedMs, clearance, settledLoops, peakError);
            telemetry.update();
            sleep(20);
        }
        if (isStopRequested()) {
            drive.stop();
            state = "STOP_REQUEST";
        }
        result.state = state;
        result.finalXError = drive.getLastFieldErrorX();
        result.finalYError = drive.getLastFieldErrorY();
        result.finalHeadingError = drive.getLastHeadingErrorDeg();
        result.peakError = peakError;
        result.overshootCm = overshootCm;
        result.settlingJitterCm = settlingJitterCm;
        if (result.elapsedSec == 0.0) result.elapsedSec = (System.nanoTime() - started) / 1_000_000_000.0;
        result.score = result.elapsedSec + result.overshootCm * 2.0 + result.settlingJitterCm * 4.0
                + Math.hypot(result.finalXError, result.finalYError)
                + Math.abs(result.finalHeadingError) * 0.02
                + (result.timeout || result.boundaryAborted ? 1000.0 : 0.0);
        publishTrial(index, total, trial, clampedX, clampedY, clamped, localizer,
                drive, state, (long) (result.elapsedSec * 1000),
                SAFE_HALF_SQUARE_CM - Math.max(Math.abs(localizer.getX()), Math.abs(localizer.getY())),
                settledLoops, peakError);
        telemetry.update();
        return result;
    }

    private void publishTrial(int index, int total, Trial trial, double x, double y, boolean clamped,
                              Localizer localizer, MecanumDrive drive, String state, long elapsedMs,
                              double clearance, int settledLoops, double peakError) {
        telemetry.addData("trial", "%d/%d %s", index + 1, total, trial.id);
        telemetry.addData("gains", "%s P %.4f/%.4f/%.4f H %.4f/%.4f/%.4f", trial.gains.name,
                trial.gains.posKp, trial.gains.posKi, trial.gains.posKd,
                trial.gains.headKp, trial.gains.headKi, trial.gains.headKd);
        telemetry.addData("target", "raw X %.0f Y %.0f H %.0f | cmd X %.0f Y %.0f | dist %.0fcm | clamped %s",
                trial.x, trial.y, trial.heading, x, y, Math.hypot(trial.x, trial.y), clamped);
        telemetry.addData("bounds", "outer +/-%.1f cm, safe +/-%.1f cm, warning %.1f, abort %.1f, clearance %.1f",
                OUTER_HALF_SQUARE_CM, SAFE_HALF_SQUARE_CM, LIVE_WARNING_THRESHOLD_CM,
                LIVE_ABORT_THRESHOLD_CM, clearance);
        telemetry.addData("pose", "X %.2f Y %.2f H %.2f", localizer.getX(), localizer.getY(), localizer.getHeadingDeg());
        telemetry.addData("errors", "X %.2f Y %.2f H %.2f peak %.2f", drive.getLastFieldErrorX(),
                drive.getLastFieldErrorY(), drive.getLastHeadingErrorDeg(), peakError);
        telemetry.addData("power", "FL %.2f FR %.2f BL %.2f BR %.2f", drive.getLastFlPower(),
                drive.getLastFrPower(), drive.getLastBlPower(), drive.getLastBrPower());
        telemetry.addData("drive", drive.getState());
        telemetry.addData("state", "%s elapsed %.2fs settle %d/%d warned %s", state,
                elapsedMs / 1000.0, settledLoops, SETTLE_LOOPS, clearance <= SAFE_HALF_SQUARE_CM - LIVE_WARNING_THRESHOLD_CM);
        publishResults();
    }

    private void publishResults() {
        if (best != null) telemetry.addData("BEST PID", "%s | score %.2f | time %.2fs | overshoot %.2fcm | jitter %.2fcm | %s",
                best.id, best.score, best.elapsedSec, best.overshootCm, best.settlingJitterCm, best.state);
        for (Result result : recentResults) telemetry.addData("result", "%s %s %.2fs final %.1f/%.1f/%.1f",
                result.id, result.state, result.elapsedSec, result.finalXError,
                result.finalYError, result.finalHeadingError);
    }

    private void showSetup(int trialCount) {
        telemetry.addLine("AUTOMATIC PID TUNING — 2m x 2m square, origin at center");
        telemetry.addData("matrix", "%d trials (cardinal/diag/mixed/heading); X right, Y fwd, +H CCW", trialCount);
        telemetry.addData("safety", "outer +/-%.0f cm; safe +/-%.0f cm; timeout %.0fs; power %.0f%%",
                OUTER_HALF_SQUARE_CM, SAFE_HALF_SQUARE_CM, TRIAL_TIMEOUT_MS / 1000.0, POWER_LIMIT * 100.0);
        telemetry.addData("LIVE_WARNING_THRESHOLD_CM", LIVE_WARNING_THRESHOLD_CM);
        telemetry.addData("LIVE_ABORT_THRESHOLD_CM", LIVE_ABORT_THRESHOLD_CM);
        telemetry.addLine("Telemetry only; no persistence or automatic gain changes");
        telemetry.update();
    }

    private static List<Trial> buildTrials() {
        Gains[] gains = {
                new Gains("default", MecanumDrive.DEFAULT_POS_KP, MecanumDrive.DEFAULT_POS_KI,
                        MecanumDrive.DEFAULT_POS_KD, MecanumDrive.DEFAULT_HEAD_KP,
                        MecanumDrive.DEFAULT_HEAD_KI, MecanumDrive.DEFAULT_HEAD_KD),
                new Gains("lower-I", 0.030, 0.006, 0.050, 0.030, 0.003, 0.050),
                new Gains("higher-P", 0.040, 0.008, 0.050, 0.040, 0.004, 0.050)
        };
        double[][] targets = {
                // cardinal — full safe range
                {90, 0, 0}, {-90, 0, 0}, {0, 90, 0}, {0, -90, 0},
                // diagonal — corners of safe square
                {64, 64, 0}, {-64, 64, 0}, {64, -64, 0}, {-64, -64, 0},
                // mixed X/Y
                {90, 45, 0}, {-45, 90, 0}, {45, -90, 0}, {-90, -45, 0},
                // heading only
                {0, 0, 45}, {0, 0, -45}, {0, 0, 90}, {0, 0, -90},
                // combined pose
                {60, 60, 30}, {-60, 60, -30}, {60, -60, -30}, {-60, -60, 30}
        };
        String[] labels = {
                "card+R", "card-L", "card+F", "card-B",
                "diag+NE", "diag+NW", "diag+SE", "diag+SW",
                "mix+R+F", "mix+L+F", "mix+R-B", "mix+L-B",
                "head+45", "head-45", "head+90", "head-90",
                "combo+NE", "combo+NW", "combo+SE", "combo+SW"
        };
        List<Trial> trials = new ArrayList<>();
        for (Gains gain : gains) for (int i = 0; i < targets.length; i++)
            trials.add(new Trial(gain.name + "-" + labels[i], targets[i][0], targets[i][1], targets[i][2], gain));
        return trials;
    }

    private static double clampFinite(double value, double min, double max) {
        if (!Double.isFinite(value)) throw new IllegalArgumentException("non-finite target");
        return Math.max(min, Math.min(max, value));
    }
}
