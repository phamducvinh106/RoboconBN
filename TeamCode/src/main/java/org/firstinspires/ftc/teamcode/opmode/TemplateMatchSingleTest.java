package org.firstinspires.ftc.teamcode.opmode;

import org.firstinspires.ftc.teamcode.core.TemplateMatchCamera;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * OpMode tối giản dể test TemplateMatchCamera với target.png duy nhất.
 *
 * Không cần gamepad — chỉ cần bật camera, detect, và xem telemetry.
 * Dùng A để chụp và in raw dx/dy ra logcat (dể debug offline).
 */
@TeleOp(name = "TMatch Single (target.png)", group = "Test")
public final class TemplateMatchSingleTest extends LinearOpMode {

    private static final String TARGET_ASSET = "target.png";

    private TemplateMatchCamera camera;

    @Override
    public void runOpMode() throws InterruptedException {
        camera = new TemplateMatchCamera(
                hardwareMap, "Webcam 1", true, TARGET_ASSET);
        camera.startAsync();

        telemetry.addLine("=== TemplateMatchCamera: Single Target Test ===");
        telemetry.addData("Target", TARGET_ASSET);
        telemetry.addData("Template loaded", camera.isTemplateLoaded());
        String loadErr = camera.getLoadError();
        if (loadErr != null) telemetry.addData("Template error", loadErr);
        telemetry.update();

        waitForStart();

        boolean prevA = false;

        try {
            while (opModeIsActive()) {
                /* ---- snapshot on A ---- */
                boolean aBtn = gamepad1.a;
                if (aBtn && !prevA) {
                    TemplateMatchCamera.CameraResult snap = camera.getLatestResult();
                    telemetry.addData("SNAPSHOT", "");
                    if (snap.isValid()) {
                        TemplateMatchCamera.Detection d = snap.detection;
                        telemetry.addData("  label",      d.label);
                        telemetry.addData("  center",     "(%.1f, %.1f)", d.centerX, d.centerY);
                        telemetry.addData("  dx dy",      "(%.1f, %.1f)", snap.dxPx, snap.dyPx);
                        telemetry.addData("  confidence", "%.4f", d.confidence);
                    } else {
                        telemetry.addData("  detection", "NONE");
                    }
                }
                prevA = aBtn;

                /* ---- live telemetry ---- */
                TemplateMatchCamera.CameraResult r = camera.getLatestResult();
                long ageMs = camera.getLastSuccessfulDetectionMs() == 0
                        ? Long.MAX_VALUE
                        : System.currentTimeMillis() - camera.getLastSuccessfulDetectionMs();
                boolean fresh = r.isValid() && ageMs < 500;

                telemetry.addLine("--- CAMERA ---");
                telemetry.addData("State", camera.getCameraState());
                telemetry.addData("Template loaded", camera.isTemplateLoaded());
                telemetry.addData("Fresh (%s)", fresh ? "YES" : "NO");

                telemetry.addLine("--- PERFORMANCE ---");
                telemetry.addData("Camera FPS",     "%.1f", camera.getFrameFps());
                telemetry.addData("Processing FPS", "%.1f", camera.getProcessingFps());
                telemetry.addData("Processing ms",  "%.1f", camera.getProcessingMs());

                telemetry.addLine("--- DETECTION ---");
                if (r.isValid()) {
                    TemplateMatchCamera.Detection d = r.detection;
                    telemetry.addData("Label",       d.label);
                    telemetry.addData("dx (right +)", "%.1f", r.dxPx);
                    telemetry.addData("dy (down +)",  "%.1f", r.dyPx);
                    telemetry.addData("Center px",    "(%.0f, %.0f)", d.centerX, d.centerY);
                    telemetry.addData("Dist px",      "%.1f", d.distanceToCenter);
                    telemetry.addData("Confidence",   "%.4f", d.confidence);
                } else {
                    telemetry.addData("Detection", "NONE (miss/hold)");
                }

                telemetry.addLine("--- FILTER ---");
                telemetry.addData("rawDx",       "%.1f", camera.getRawDx());
                telemetry.addData("smoothedDx",  "%.1f", camera.getSmoothedDx());
                telemetry.addData("outlierStr",  camera.getOutlierStreak());
                telemetry.addData("missStr",     camera.getMissStreak());
                telemetry.addData("filterInit",  camera.isFilterInitialized());

                telemetry.update();
                sleep(20);
            }
        } finally {
            camera.stop();
        }
    }
}
