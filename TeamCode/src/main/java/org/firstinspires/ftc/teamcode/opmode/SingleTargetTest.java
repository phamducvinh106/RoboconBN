package org.firstinspires.ftc.teamcode.opmode;

import org.firstinspires.ftc.teamcode.core.SingleTargetCamera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * OpMode dành riêng dể test SingleTargetCamera với 4 target có sẵn.
 *
 * D-pad / gamepad buttons chuyên target:
 *   dpad_up    → target1.png
 *   dpad_right → target2.png
 *   dpad_down  → target3.png
 *   dpad_left  → target4.png
 *
 * Nút X trên gamepad1: chuyên target theo vòng tròn (1→2→3→4→1).
 *
 * Telemetry hiển thị:
 *   - Target hiện tại & camera state
 *   - dx/dy pixel, confidence, matches/inliers
 *   - FPS & processing time
 *   - Filter debug (raw/smoothed dx, outlier streak, miss streak)
 */
@TeleOp(name = "Single Target Test (4 targets)")
public final class SingleTargetTest extends LinearOpMode {

    private static final String[] TARGETS = {
            "target.png"
    };

    private static final String[] LABELS = {
            "target"
    };

    private SingleTargetCamera camera;
    private int activeTargetIndex;

    @Override
    public void runOpMode() throws InterruptedException {
        activeTargetIndex = 0;

        camera = new SingleTargetCamera(
                hardwareMap,
                "Webcam 1",
                true,
                TARGETS[activeTargetIndex]
        );
        camera.startAsync();

        telemetry.addLine("=== SingleTargetCamera Test ===");
        telemetry.addData("Target", "%d/%d: %s",
                activeTargetIndex + 1, TARGETS.length,
                LABELS[activeTargetIndex]);
        telemetry.addData("Template loaded", camera.isTemplateLoaded());
        String loadError = camera.getLoadError();
        if (loadError != null) {
            telemetry.addData("Template error", loadError);
        }
        telemetry.addLine("DPAD ↑→↓← to switch target");
        telemetry.addLine("X to cycle targets");
        telemetry.update();

        waitForStart();

        boolean prevDpadUp    = false;
        boolean prevDpadRight = false;
        boolean prevDpadDown  = false;
        boolean prevDpadLeft  = false;
        boolean prevX         = false;

        try {
            while (opModeIsActive()) {
                // --- Handle target switching ---
                boolean dpUp    = gamepad1.dpad_up;
                boolean dpRight = gamepad1.dpad_right;
                boolean dpDown  = gamepad1.dpad_down;
                boolean dpLeft  = gamepad1.dpad_left;
                boolean xBtn    = gamepad1.x;

                if ((dpUp    && !prevDpadUp)
                        || (dpRight && !prevDpadRight)
                        || (dpDown  && !prevDpadDown)
                        || (dpLeft  && !prevDpadLeft)) {
                    int newIdx;
                    if (dpUp && !prevDpadUp)         newIdx = 0;
                    else if (dpRight && !prevDpadRight) newIdx = 1;
                    else if (dpDown && !prevDpadDown)   newIdx = 2;
                    else                                newIdx = 3;

                    switchTarget(newIdx);
                }

                if (xBtn && !prevX) {
                    int next = (activeTargetIndex + 1) % TARGETS.length;
                    switchTarget(next);
                }

                prevDpadUp    = dpUp;
                prevDpadRight = dpRight;
                prevDpadDown  = dpDown;
                prevDpadLeft  = dpLeft;
                prevX         = xBtn;

                // --- Telemetry ---
                sendTelemetry();
                telemetry.update();

                sleep(20);
            }
        } finally {
            camera.stop();
        }
    }

    private void switchTarget(int index) {
        if (index == activeTargetIndex) return;
        activeTargetIndex = index;
        camera.setTarget(TARGETS[index]);

        telemetry.addData("SWITCHED", "→ %s", LABELS[index]);
        telemetry.addData("Template loaded", camera.isTemplateLoaded());
        String loadError = camera.getLoadError();
        if (loadError != null) {
            telemetry.addData("Template error", loadError);
        }
    }

    private void sendTelemetry() {
        SingleTargetCamera.CameraResult result = camera.getLatestResult();
        long ageMs = camera.getLastSuccessfulDetectionMs() == 0
                ? Long.MAX_VALUE
                : System.currentTimeMillis()
                - camera.getLastSuccessfulDetectionMs();
        boolean fresh = result.isValid() && ageMs < 500;

        // --- Status ---
        telemetry.addLine("=== TARGET & STATE ===");
        telemetry.addData("Active target", "%d/%d: %s",
                activeTargetIndex + 1, TARGETS.length,
                LABELS[activeTargetIndex]);
        telemetry.addData("Camera state", camera.getCameraState());
        if ("ERROR".equals(camera.getCameraState())) {
            telemetry.addData("Camera error", camera.getCameraErrorCode());
        }
        telemetry.addData("Template loaded", camera.isTemplateLoaded());
        String loadError = camera.getLoadError();
        if (loadError != null) {
            telemetry.addData("Template error", loadError);
        }
        telemetry.addData("Fresh (<500ms)", fresh);

        // --- Performance ---
        telemetry.addLine("--- PERFORMANCE ---");
        telemetry.addData("Camera FPS", "%.1f", camera.getFrameFps());
        telemetry.addData("Processing FPS", "%.1f", camera.getProcessingFps());
        telemetry.addData("Processing ms", "%.1f", camera.getProcessingMs());

        // --- Detection ---
        telemetry.addLine("--- DETECTION ---");
        if (result.isValid()) {
            SingleTargetCamera.Detection d = result.detection;
            telemetry.addData("Label", d.label);
            telemetry.addData("dx px (right +)", "%.1f", result.dxPx);
            telemetry.addData("dy px (down +)", "%.1f", result.dyPx);
            telemetry.addData("Center px", "(%.0f, %.0f)", d.centerX, d.centerY);
            telemetry.addData("Dist to center px", "%.1f", d.distanceToCenter);
            telemetry.addData("Confidence", "%.2f", d.confidence);
            telemetry.addData("Matches / inliers", "%d / %d",
                    d.goodMatches, d.inliers);
        } else {
            telemetry.addData("Detection", "NONE");
            telemetry.addData("dx px", "NaN");
            telemetry.addData("dy px", "NaN");
        }

        // --- Filter debug ---
        telemetry.addLine("--- FILTER DEBUG ---");
        telemetry.addData("rawDx", "%.1f", camera.getRawDx());
        telemetry.addData("smoothedDx", "%.1f", camera.getSmoothedDx());
        telemetry.addData("outlierStreak", camera.getOutlierStreak());
        telemetry.addData("missStreak", camera.getMissStreak());
        telemetry.addData("filterInit", camera.isFilterInitialized());
        telemetry.addData("activeLabel", camera.getActiveLabel());
    }
}
