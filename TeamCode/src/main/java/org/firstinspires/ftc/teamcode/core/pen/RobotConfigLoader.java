package org.firstinspires.ftc.teamcode.core.pen;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public final class RobotConfigLoader {

    private static final Pattern VERSION = Pattern.compile("\"version\"\\s*:\\s*(\\d+)");
    private static final Pattern NUMBER = Pattern.compile(
            "\"([A-Za-z0-9_]+)\"\\s*:\\s*(-?(?:\\d+(?:\\.\\d*)?|\\.\\d+)(?:[eE][+-]?\\d+)?)");
    private static final Pattern STRING = Pattern.compile(
            "\"deviceName\"\\s*:\\s*\"([^\"]+)\"");

    private static final String[] REQUIRED_NUMBERS = {
            "upPosition", "downPosition",
            "drivePower", "positionToleranceMm", "headingToleranceDeg",
            "headingHoldPower", "settleCycles"
    };

    private RobotConfigLoader() {}

    public static RobotConfig load(String json) {
        if (json == null || json.trim().isEmpty()) {
            throw new IllegalArgumentException("missing robot config");
        }
        String trimmed = json.trim();
        if (!trimmed.startsWith("{") || !trimmed.endsWith("}")) {
            throw new IllegalArgumentException("malformed robot config");
        }

        Matcher versionMatcher = VERSION.matcher(json);
        if (!versionMatcher.find()
                || Integer.parseInt(versionMatcher.group(1)) != RobotConfig.SCHEMA_VERSION) {
            throw new IllegalArgumentException("wrong robot config version");
        }

        Map<String, Double> numbers = new LinkedHashMap<>();
        Matcher numberMatcher = NUMBER.matcher(json);
        while (numberMatcher.find()) {
            numbers.put(numberMatcher.group(1), Double.valueOf(numberMatcher.group(2)));
        }
        for (String key : REQUIRED_NUMBERS) {
            if (!numbers.containsKey(key) || !Double.isFinite(numbers.get(key))) {
                throw new IllegalArgumentException("missing or non-finite field: " + key);
            }
        }

        Matcher stringMatcher = STRING.matcher(json);
        if (!stringMatcher.find()) {
            throw new IllegalArgumentException("missing pen deviceName");
        }
        String deviceName = stringMatcher.group(1);

        range(numbers, "upPosition", 0, 1);
        range(numbers, "downPosition", 0, 1);
        range(numbers, "drivePower", 0.01, 1);
        range(numbers, "positionToleranceMm", 0.1, 500);
        range(numbers, "headingToleranceDeg", 0.01, 45);
        range(numbers, "headingHoldPower", 0.01, 1);
        range(numbers, "settleCycles", 1, 100);

        RobotConfig.PenConfig pen = new RobotConfig.PenConfig(
                deviceName,
                numbers.get("upPosition"),
                numbers.get("downPosition"));
        RobotConfig.MotionConfig motion = new RobotConfig.MotionConfig(
                numbers.get("drivePower"),
                numbers.get("positionToleranceMm"),
                numbers.get("headingToleranceDeg"),
                numbers.get("headingHoldPower"),
                numbers.get("settleCycles").intValue());
        return new RobotConfig(pen, motion);
    }

    private static void range(Map<String, Double> values, String key, double min, double max) {
        double value = values.get(key);
        if (value < min || value > max) {
            throw new IllegalArgumentException("out of range: " + key);
        }
    }
}
