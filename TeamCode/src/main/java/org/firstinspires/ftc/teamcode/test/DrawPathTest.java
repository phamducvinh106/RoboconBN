package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.core.pen.DrawPathConfig;
import org.firstinspires.ftc.teamcode.core.pen.DrawPathLoader;
import org.firstinspires.ftc.teamcode.core.pen.RobotConfig;
import org.firstinspires.ftc.teamcode.core.pen.RobotConfigLoader;

import java.util.List;
import java.util.Map;

/** Offline checks: parse JSON, mm→cm, thứ tự pen up/down. */
public final class DrawPathTest {

    private static final String DEMO_PATH = "{\n"
            + "  \"version\": 1,\n"
            + "  \"name\": \"demo-square\",\n"
            + "  \"startPose\": { \"xMm\": 0, \"yMm\": 0, \"headingDeg\": 0 },\n"
            + "  \"drawingHeadingDeg\": 0,\n"
            + "  \"waypoints\": [\n"
            + "    { \"xMm\": 0, \"yMm\": 0, \"penDown\": false },\n"
            + "    { \"xMm\": 100, \"yMm\": 0, \"penDown\": true },\n"
            + "    { \"xMm\": 100, \"yMm\": 100, \"penDown\": true },\n"
            + "    { \"xMm\": 0, \"yMm\": 100, \"penDown\": true },\n"
            + "    { \"xMm\": 0, \"yMm\": 0, \"penDown\": true }\n"
            + "  ]\n"
            + "}";

    private static final String ROBOT_CONFIG = "{\n"
            + "  \"version\": 1,\n"
            + "  \"pen\": {\n"
            + "    \"deviceName\": \"penServo\",\n"
            + "    \"upPosition\": 0.05,\n"
            + "    \"downPosition\": 0.45\n"
            + "  },\n"
            + "  \"motion\": {\n"
            + "    \"drivePower\": 0.20,\n"
            + "    \"positionToleranceMm\": 3.0,\n"
            + "    \"headingToleranceDeg\": 1.0,\n"
            + "    \"headingHoldPower\": 0.35,\n"
            + "    \"settleCycles\": 2\n"
            + "  }\n"
            + "}";

    private static int passed;
    private static int failed;

    public static void main(String[] args) {
        DrawPathConfig path = DrawPathLoader.load(DEMO_PATH);
        check("name", "demo-square".equals(path.name));
        check("start x cm", Math.abs(path.startPose.xCm() - 0.0) < 1e-9);
        check("start y cm", Math.abs(path.startPose.yCm() - 0.0) < 1e-9);
        check("waypoint count", path.waypoints.size() == 5);

        DrawPathConfig.Waypoint second = path.waypoints.get(1);
        check("mm to cm x", Math.abs(second.xCm() - 10.0) < 1e-9);
        check("mm to cm y", Math.abs(second.yCm() - 0.0) < 1e-9);

        List<Boolean> penSeq = DrawPathLoader.penSequence(DEMO_PATH);
        check("pen transitions", penSeq.size() == 2);
        check("first pen up", !penSeq.get(0));
        check("then pen down", penSeq.get(1));

        Map<String, Double> firstCm = DrawPathLoader.firstWaypointCm(DEMO_PATH);
        check("first waypoint x cm", Math.abs(firstCm.get("xCm")) < 1e-9);

        RobotConfig robot = RobotConfigLoader.load(ROBOT_CONFIG);
        check("tolerance cm", Math.abs(robot.motion.positionToleranceCm() - 0.3) < 1e-9);
        check("pen device", "penServo".equals(robot.pen.deviceName));

        expectInvalidPath("{");
        expectInvalidRobotConfig("{\"version\": 99}");

        System.out.printf("RESULT: %d passed, %d failed%n", passed, failed);
        if (failed > 0) System.exit(1);
    }

    private static void check(String name, boolean cond) {
        if (cond) {
            passed++;
            System.out.println("  PASS " + name);
        } else {
            failed++;
            System.out.println("  FAIL " + name);
        }
    }

    private static void expectInvalidPath(String json) {
        try {
            DrawPathLoader.load(json);
            failed++;
            System.out.println("  FAIL expected invalid path");
        } catch (IllegalArgumentException expected) {
            passed++;
            System.out.println("  PASS reject invalid path");
        }
    }

    private static void expectInvalidRobotConfig(String json) {
        try {
            RobotConfigLoader.load(json);
            failed++;
            System.out.println("  FAIL expected invalid robot config");
        } catch (IllegalArgumentException expected) {
            passed++;
            System.out.println("  PASS reject invalid robot config");
        }
    }
}
