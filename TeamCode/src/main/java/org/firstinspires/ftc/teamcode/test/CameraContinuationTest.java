package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.core.FourTargetCameraOrchestrator;
import org.firstinspires.ftc.teamcode.core.OrbTemplateCamera;

/** Dependency-free executable checks for production camera policy seams. */
public final class CameraContinuationTest {
    private static int checks;

    public static void main(String[] args) {
        check(OrbTemplateCamera.STREAM_WIDTH == 640 && OrbTemplateCamera.STREAM_HEIGHT == 480, "stream bounds");
        check(OrbTemplateCamera.MAX_FEATURES == 400 && OrbTemplateCamera.MAX_PYRAMID_LEVELS == 6, "ORB bounds");
        check(OrbTemplateCamera.MAX_MATCHES == 80 && OrbTemplateCamera.MIN_GOOD_MATCHES == 12, "match bounds");
        check(OrbTemplateCamera.MIN_INLIERS == 10, "inlier bounds");
        check(OrbTemplateCamera.MAX_ROI_WIDTH == 480 && OrbTemplateCamera.MAX_ROI_HEIGHT == 360, "ROI bounds");
        check(OrbTemplateCamera.MAX_RESULT_AGE_MS == 120 && OrbTemplateCamera.MAX_FRAME_LATENCY_MS == 100, "timing budget");
        check(OrbTemplateCamera.RATIO > 0 && OrbTemplateCamera.RATIO < 1, "ratio policy");
        check(OrbTemplateCamera.MIN_INLIER_RATIO >= 0.5 && OrbTemplateCamera.MIN_INLIER_RATIO <= 1, "inlier ratio policy");
        check(OrbTemplateCamera.ACQUIRE_FRAMES == 3 && OrbTemplateCamera.LOSE_MISS_FRAMES == 3, "temporal gates");
        check(!OrbTemplateCamera.fresh(null, System.currentTimeMillis()), "null rejects");
        check(OrbTemplateCamera.Mode.SINGLE_TARGET != OrbTemplateCamera.Mode.MULTI_TARGET, "two modes");
        check(OrbTemplateCamera.CameraId.WEBCAM1 != OrbTemplateCamera.CameraId.WEBCAM2, "two webcams");
        check(OrbTemplateCamera.State.CREATED != OrbTemplateCamera.State.STREAMING, "lifecycle states");
        check(OrbTemplateCamera.State.CLOSED != OrbTemplateCamera.State.ERROR, "terminal states");
        check(OrbTemplateCamera.DetectionState.LOCKED != OrbTemplateCamera.DetectionState.LOST, "detection states");
        check(FourTargetCameraOrchestrator.TARGET_IDS.length == 4, "four target instances");
        check(FourTargetCameraOrchestrator.TARGET_IDS[0].equals("target1") && FourTargetCameraOrchestrator.TARGET_IDS[3].equals("target4"), "explicit target IDs");
        System.out.println("CameraContinuationTest passed " + checks + " checks");
    }

    private static void check(boolean condition, String name) {
        if (!condition) throw new AssertionError(name);
        checks++;
    }
}
