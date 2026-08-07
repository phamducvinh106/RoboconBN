package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.opencv.core.Mat;

/** Explicit four-target ownership: two targets per named webcam. */
public final class FourTargetCameraOrchestrator {
    public static final String[] TARGET_IDS = {"target1", "target2", "target3", "target4"};
    private final OrbTemplateCamera[] cameras;

    public FourTargetCameraOrchestrator(HardwareMap map, boolean preview, Mat[] templates) {
        if (templates == null || templates.length != TARGET_IDS.length) {
            throw new IllegalArgumentException("exactly four target templates required");
        }
        cameras = new OrbTemplateCamera[] {
                new OrbTemplateCamera(map, "webcam1", preview, OrbTemplateCamera.Mode.SINGLE_TARGET, TARGET_IDS[0], templates[0]),
                new OrbTemplateCamera(map, "webcam1", preview, OrbTemplateCamera.Mode.SINGLE_TARGET, TARGET_IDS[1], templates[1]),
                new OrbTemplateCamera(map, "webcam2", preview, OrbTemplateCamera.Mode.MULTI_TARGET, TARGET_IDS[2], templates[2]),
                new OrbTemplateCamera(map, "webcam2", preview, OrbTemplateCamera.Mode.MULTI_TARGET, TARGET_IDS[3], templates[3])
        };
    }

    public void start() { for (OrbTemplateCamera camera : cameras) camera.startAsync(); }
    public void stop() { for (OrbTemplateCamera camera : cameras) camera.stop(); }
    public OrbTemplateCamera.Result latest(int index) {
        if (index < 0 || index >= cameras.length) throw new IndexOutOfBoundsException("target index");
        return cameras[index].getLatestResult();
    }
}
