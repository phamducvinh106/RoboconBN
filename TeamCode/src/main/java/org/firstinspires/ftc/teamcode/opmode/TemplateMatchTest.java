package org.firstinspires.ftc.teamcode.opmode;

import org.firstinspires.ftc.teamcode.core.TemplateMatchCamera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * OpMode dành riêng dể test TemplateMatchCamera với 4 target có sẵn.
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
 *   - dx/dy pixel, confidence (matchTemplate CCOEFF_NORMED)
 *   - FPS & processing time
 *   - Filter debug (raw/smoothed dx, outlier streak, miss streak)
 */
@TeleOp(name = "Template Match Test (4 targets)")
public final class TemplateMatchTest extends LinearOpMode {

    private static final String[] TARGETS = {
            "target.png", "target2.png", "target3.png", "target4.png"
    };
    private static final String[] LABELS = {
            "target", "target2", "target3", "target4"
    };

    private TemplateMatchCamera camera;
    private int activeTargetIndex;

    @Override
    public void runOpMode() throws InterruptedException {
        activeTargetIndex = 0;

        camera = new TemplateMatchCamera(
                hardwareMap, "Webcam 1", true, TARGETS[activeTargetIndex]);
        camera.startAsync();

        telemetry.addLine("=== TemplateMatchCamera Test ===");
        telemetry.addData("Target", "%d/%d: %s",
                activeTargetIndex + 1, TARGETS.length, LABELS[activeTargetIndex]);
        telemetry.addData("Template loaded", camera.isTemplateLoaded());
        String loadError = camera.getLoadError();
        if (loadError != null) telemetry.addData("Template error", loadError);
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
                boolean dpUp    = gamepad1.dpad_up;
                boolean dpRight = gamepad1.dpad_right;
                boolean dpDown  = gamepad1.dpad_down;
                boolean dpLeft  = gamepad1.dpad_left;
                boolean xBtn    = gamepad1.x;

                if ((dpUp && !prevDpadUp) || (dpRight && !prevDpadRight)
                        || (dpDown && !prevDpadDown) || (dpLeft && !prevDpadLeft)) {
                    int newIdx;
                    if (dpUp && !prevDpadUp)           newIdx = 0;
                    else if (dpRight && !prevDpadRight) newIdx = 1;
                    else if (dpDown && !prevDpadDown)   newIdx = 2;
                    else                                newIdx = 3;
                    switchTarget(newIdx);
                }

                if (xBtn && !prevX) {
                    switchTarget((activeTargetIndex + 1) % TARGETS.length);
                }

                prevDpadUp = dpUp; prevDpadRight = dpRight;
                prevDpadDown = dpDown; prevDpadLeft = dpLeft;
                prevX = xBtn;

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
        if (loadError != null) telemetry.addData("Template error", loadError);
    }

    private void sendTelemetry() {
        TemplateMatchCamera.CameraResult result = camera.getLatestResult();
        long ageMs = camera.getLastSuccessfulDetectionMs() == 0
                ? Long.MAX_VALUE
                : System.currentTimeMillis() - camera.getLastSuccessfulDetectionMs();
        boolean fresh = result.isValid() && ageMs < 500;

        telemetry.addLine("=== TARGET & STATE ===");
        telemetry.addData("Active target", "%d/%d: %s",
                activeTargetIndex + 1, TARGETS.length, LABELS[activeTargetIndex]);
        telemetry.addData("Camera state", camera.getCameraState());
        if ("ERROR".equals(camera.getCameraState()))
            telemetry.addData("Camera error", camera.getCameraErrorCode());
        telemetry.addData("Template loaded", camera.isTemplateLoaded());
        String loadError = camera.getLoadError();
        if (loadError != null) telemetry.addData("Template error", loadError);
        telemetry.addData("Fresh (<500ms)", fresh);

        telemetry.addLine("--- PERFORMANCE ---");
        telemetry.addData("Camera FPS", "%.1f", camera.getFrameFps());
        telemetry.addData("Processing FPS", "%.1f", camera.getProcessingFps());
        telemetry.addData("Processing ms", "%.1f", camera.getProcessingMs());

        telemetry.addLine("--- DETECTION ---");
        if (result.isValid()) {
            TemplateMatchCamera.Detection d = result.detection;
            telemetry.addData("Label", d.label);
            telemetry.addData("dx px (right +)", "%.1f", result.dxPx);
            telemetry.addData("dy px (down +)", "%.1f", result.dyPx);
            telemetry.addData("Center px", "(%.0f, %.0f)", d.centerX, d.centerY);
            telemetry.addData("Dist to center px", "%.1f", d.distanceToCenter);
            telemetry.addData("Confidence (CCOEFF)", "%.3f", d.confidence);
        } else {
            telemetry.addData("Detection", "NONE");
        }

        telemetry.addLine("--- FILTER DEBUG ---");
        telemetry.addData("rawDx", "%.1f", camera.getRawDx());
        telemetry.addData("smoothedDx", "%.1f", camera.getSmoothedDx());
        telemetry.addData("outlierStreak", camera.getOutlierStreak());
        telemetry.addData("missStreak", camera.getMissStreak());
        telemetry.addData("filterInit", camera.isFilterInitialized());
        telemetry.addData("activeLabel", camera.getActiveLabel());
    }
}
