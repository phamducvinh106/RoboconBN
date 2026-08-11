package org.firstinspires.ftc.teamcode.core.pen;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Parse draw-path.json — đơn vị mm trong file, chuyển sang cm qua {@link DrawPathConfig}.
 */
public final class DrawPathLoader {

    private static final Pattern VERSION = Pattern.compile("\"version\"\\s*:\\s*(\\d+)");
    private static final Pattern NAME = Pattern.compile("\"name\"\\s*:\\s*\"([^\"]+)\"");
    private static final Pattern DRAWING_HEADING = Pattern.compile(
            "\"drawingHeadingDeg\"\\s*:\\s*(-?(?:\\d+(?:\\.\\d*)?|\\.\\d+)(?:[eE][+-]?\\d+)?)");
    private static final Pattern START_POSE = Pattern.compile(
            "\"startPose\"\\s*:\\s*\\{\\s*\"xMm\"\\s*:\\s*(-?(?:\\d+(?:\\.\\d*)?|\\.\\d+)(?:[eE][+-]?\\d+)?)"
                    + "\\s*,\\s*\"yMm\"\\s*:\\s*(-?(?:\\d+(?:\\.\\d*)?|\\.\\d+)(?:[eE][+-]?\\d+)?)"
                    + "\\s*,\\s*\"headingDeg\"\\s*:\\s*(-?(?:\\d+(?:\\.\\d*)?|\\.\\d+)(?:[eE][+-]?\\d+)?)\\s*\\}");
    private static final Pattern WAYPOINT = Pattern.compile(
            "\\{\\s*\"xMm\"\\s*:\\s*(-?(?:\\d+(?:\\.\\d*)?|\\.\\d+)(?:[eE][+-]?\\d+)?)"
                    + "\\s*,\\s*\"yMm\"\\s*:\\s*(-?(?:\\d+(?:\\.\\d*)?|\\.\\d+)(?:[eE][+-]?\\d+)?)"
                    + "\\s*,\\s*\"penDown\"\\s*:\\s*(true|false)\\s*\\}");

    private DrawPathLoader() {}

    public static DrawPathConfig load(String json) {
        if (json == null || json.trim().isEmpty()) {
            throw new IllegalArgumentException("missing draw path");
        }
        String trimmed = json.trim();
        if (!trimmed.startsWith("{") || !trimmed.endsWith("}")) {
            throw new IllegalArgumentException("malformed draw path");
        }

        Matcher versionMatcher = VERSION.matcher(json);
        if (!versionMatcher.find()
                || Integer.parseInt(versionMatcher.group(1)) != DrawPathConfig.SCHEMA_VERSION) {
            throw new IllegalArgumentException("wrong draw path version");
        }

        Matcher nameMatcher = NAME.matcher(json);
        if (!nameMatcher.find()) {
            throw new IllegalArgumentException("missing name");
        }
        String name = nameMatcher.group(1);

        Matcher startMatcher = START_POSE.matcher(json);
        if (!startMatcher.find()) {
            throw new IllegalArgumentException("missing startPose");
        }
        DrawPathConfig.Pose startPose = new DrawPathConfig.Pose(
                Double.parseDouble(startMatcher.group(1)),
                Double.parseDouble(startMatcher.group(2)),
                Double.parseDouble(startMatcher.group(3)));

        Matcher headingMatcher = DRAWING_HEADING.matcher(json);
        if (!headingMatcher.find()) {
            throw new IllegalArgumentException("missing drawingHeadingDeg");
        }
        double drawingHeadingDeg = Double.parseDouble(headingMatcher.group(1));
        if (!Double.isFinite(drawingHeadingDeg)) {
            throw new IllegalArgumentException("drawingHeadingDeg must be finite");
        }

        List<DrawPathConfig.Waypoint> waypoints = new ArrayList<>();
        Matcher waypointMatcher = WAYPOINT.matcher(json);
        while (waypointMatcher.find()) {
            waypoints.add(new DrawPathConfig.Waypoint(
                    Double.parseDouble(waypointMatcher.group(1)),
                    Double.parseDouble(waypointMatcher.group(2)),
                    Boolean.parseBoolean(waypointMatcher.group(3))));
        }
        if (waypoints.isEmpty()) {
            throw new IllegalArgumentException("at least one waypoint required");
        }

        return new DrawPathConfig(name, startPose, drawingHeadingDeg, waypoints);
    }

    /** Trả về thứ tự pen up/down theo waypoint — dùng cho test offline. */
    public static List<Boolean> penSequence(String json) {
        DrawPathConfig config = load(json);
        List<Boolean> sequence = new ArrayList<>();
        Boolean last = null;
        for (DrawPathConfig.Waypoint waypoint : config.waypoints) {
            if (last == null || last != waypoint.penDown) {
                sequence.add(waypoint.penDown);
                last = waypoint.penDown;
            }
        }
        return sequence;
    }

    /** mm → cm cho một waypoint — dùng cho test offline. */
    public static Map<String, Double> firstWaypointCm(String json) {
        DrawPathConfig config = load(json);
        DrawPathConfig.Waypoint wp = config.waypoints.get(0);
        Map<String, Double> out = new LinkedHashMap<>();
        out.put("xCm", wp.xCm());
        out.put("yCm", wp.yCm());
        return out;
    }
}
