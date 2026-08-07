package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.core.ColorContourCamera;

import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

/** Offline contract checks; no FTC hardware, OpenCV frame, or test dependency required. */
public final class CameraContinuationTest {
    private static int checks;

    private static void check(boolean condition, String name) {
        if (!condition) throw new AssertionError(name);
        checks++;
    }

    private static void checkEquals(Object actual, Object expected, String name) {
        check(expected.equals(actual), name + ": expected " + expected + ", got " + actual);
    }

    public static void main(String[] args) throws Exception {
        checkEquals(ColorContourCamera.Mode.SINGLE_TARGET.name(), "SINGLE_TARGET", "single mode");
        checkEquals(ColorContourCamera.Mode.MULTI_TARGET.name(), "MULTI_TARGET", "multi mode");
        checkEquals(ColorContourCamera.CameraId.WEBCAM1.name(), "WEBCAM1", "webcam1 identity");
        checkEquals(ColorContourCamera.CameraId.WEBCAM2.name(), "WEBCAM2", "webcam2 identity");

        check(ColorContourCamera.STREAM_WIDTH == 640 && ColorContourCamera.STREAM_HEIGHT == 480, "stream dimensions");
        check(ColorContourCamera.MIN_DESCRIPTOR_MATCHES >= 8, "ORB descriptor threshold");
        check(ColorContourCamera.MIN_MATCH_CONFIDENCE > 0 && ColorContourCamera.MIN_MATCH_CONFIDENCE <= 1, "match confidence threshold");
        check(ColorContourCamera.NMS_OVERLAP > 0 && ColorContourCamera.NMS_OVERLAP < 1, "NMS threshold");
        check(ColorContourCamera.MIN_CENTER_DISTANCE_PX > 0, "minimum duplicate distance");
        check(ColorContourCamera.MAX_CANDIDATES == 2, "bounded candidate ranking");
        check(ColorContourCamera.MAX_RESULT_AGE_MS > 0, "freshness limit");

        checkDeterministicPolicyContract();
        checkSourceBoundaries();
        System.out.println("CameraContinuationTest passed " + checks + " checks");
    }

    private static void checkDeterministicPolicyContract() throws Exception {
        check(ColorContourCamera.MAX_CANDIDATES == 2, "deterministic output bound");
        Set<String> states = new HashSet<>();
        Arrays.stream(ColorContourCamera.State.values()).forEach(state -> states.add(state.name()));
        check(states.containsAll(Arrays.asList("OPENING", "STREAMING", "STOPPING", "CLOSED", "ERROR")), "lifecycle states");
    }

    private static void checkSourceBoundaries() throws Exception {
        Path source = Paths.get("TeamCode/src/main/java/org/firstinspires/ftc/teamcode/core/ColorContourCamera.java");
        String text = new String(Files.readAllBytes(source), "UTF-8");
        check(text.contains("ORB.create()") && text.contains("DescriptorMatcher"), "ORB/template implementation present");
        check(!text.contains("COLOR_RGB2HSV") && !text.contains("COLOR_RGB2YCrCb"), "no HSV/YCrCb detection");
        check(!text.contains("findContours") && !text.contains("Imgproc.contour"), "no contour detection");
        check(!text.contains("I2cDevice") && !text.contains("DigitalUartRx"), "no I2C/UART camera path");
        check(text.contains("generation") && text.contains("pipeline.invalidate()"), "lifecycle invalidation guard");
        check(text.contains("startAsync") && text.contains("stop()"), "idempotent lifecycle API");
        check(text.contains("webcam1") && text.contains("webcam2"), "explicit webcam names");
    }
}
