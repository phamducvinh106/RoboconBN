package org.firstinspires.ftc.teamcode.core;

import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public final class LiftingSequenceConfigLoader {
    private static final Pattern NUMBER = Pattern.compile(
            "\\\"([A-Za-z0-9_]+)\\\"\\s*:\\s*(-?(?:\\d+(?:\\.\\d*)?|\\.\\d+)(?:[eE][+-]?\\d+)?)");
    private static final Pattern BOOLEAN = Pattern.compile(
            "\\\"(endstopActiveLow|irActiveLow|stepperDirInverted)\\\"\\s*:\\s*(true|false)");
    private static final String[] REQUIRED = {
            "placeLeft", "placeRight", "holdLeft", "holdRight",
            "stepHighNs", "stepLowNs", "irDebounceNs", "sensorStaleNs",
            "homeSteps", "ready1Steps", "lift1Steps", "ready2Steps", "lift2Steps",
            "maxRetries", "settleCycles", "totalCycles",
            "releaseBackOutCm", "positionToleranceCm", "headingToleranceDeg", "encoderFreshnessNs", "noProgressCm",
            "approachSpeed", "frameWidth", "frameHeight"
    };

    private LiftingSequenceConfigLoader() {}

    public static LiftingSequenceConfig load(String json) {
        if (json == null || json.trim().isEmpty()) throw new IllegalArgumentException("missing config");
        String trimmed = json.trim();
        if (!trimmed.startsWith("{") || !trimmed.endsWith("}")
                || trimmed.chars().filter(c -> c == '{').count() != trimmed.chars().filter(c -> c == '}').count()) {
            throw new IllegalArgumentException("malformed config");
        }
        Matcher versionMatcher = Pattern.compile("\\\"version\\\"\\s*:\\s*(\\d+)").matcher(json);
        if (!versionMatcher.find()
                || Integer.parseInt(versionMatcher.group(1)) != LiftingSequenceConfig.SCHEMA_VERSION) {
            throw new IllegalArgumentException("wrong config version");
        }
        Map<String, Double> numbers = new LinkedHashMap<>();
        Matcher numberMatcher = NUMBER.matcher(json);
        while (numberMatcher.find()) {
            numbers.put(numberMatcher.group(1), Double.valueOf(numberMatcher.group(2)));
        }
        for (String key : REQUIRED) {
            if (!numbers.containsKey(key) || !Double.isFinite(numbers.get(key))) {
                throw new IllegalArgumentException("missing or non-finite field: " + key);
            }
        }
        Map<String, Boolean> booleans = new LinkedHashMap<>();
        Matcher booleanMatcher = BOOLEAN.matcher(json);
        while (booleanMatcher.find()) {
            booleans.put(booleanMatcher.group(1), Boolean.valueOf(booleanMatcher.group(2)));
        }
        if (booleans.size() != 3) throw new IllegalArgumentException("missing sensor polarity");

        range(numbers, "placeLeft", 0, 1);
        range(numbers, "placeRight", 0, 1);
        range(numbers, "holdLeft", 0, 1);
        range(numbers, "holdRight", 0, 1);
        range(numbers, "stepHighNs", 1, 1e9);
        range(numbers, "stepLowNs", 1, 1e9);
        range(numbers, "irDebounceNs", 1, 1e12);
        range(numbers, "sensorStaleNs", 1, 1e12);
        range(numbers, "homeSteps", 0, 0);
        range(numbers, "ready1Steps", 1, 1e6);
        range(numbers, "lift1Steps", 1, 1e6);
        range(numbers, "ready2Steps", 1, 1e6);
        range(numbers, "lift2Steps", 1, 1e6);
        range(numbers, "maxRetries", 0, 10);
        range(numbers, "settleCycles", 1, 100);
        range(numbers, "totalCycles", 1, 12);
        range(numbers, "releaseBackOutCm", 0.01, 1000);
        range(numbers, "positionToleranceCm", 0.01, 100);
        range(numbers, "headingToleranceDeg", 0.01, 360);
        range(numbers, "encoderFreshnessNs", 1, 1e12);
        range(numbers, "noProgressCm", 0.01, 100);
        range(numbers, "approachSpeed", 0.01, 1);
        range(numbers, "frameWidth", 32, 4096);
        range(numbers, "frameHeight", 32, 4096);

        Map<String, LiftingSequenceConfig.Factory> factories = new LinkedHashMap<>();
        return LiftingSequenceConfig.create(
                Integer.parseInt(versionMatcher.group(1)), numbers, booleans,
                new LiftingSequenceConfig.Pose(0, 0, 0),
                new LiftingSequenceConfig.Pose(0, 0, 0),
                new LiftingSequenceConfig.Pose(0, 0, 0),
                new LiftingSequenceConfig.Pose[0],
                factories, fingerprint(json));
    }

    private static void range(Map<String, Double> numbers, String key, double min, double max) {
        double value = numbers.get(key);
        if (value < min || value > max || value != Math.rint(value) && key.endsWith("Steps")) {
            throw new IllegalArgumentException("out of range: " + key);
        }
    }

    private static String fingerprint(String source) {
        try {
            byte[] hash = MessageDigest.getInstance("SHA-256")
                    .digest(source.getBytes(StandardCharsets.UTF_8));
            StringBuilder builder = new StringBuilder();
            for (int i = 0; i < 4; i++) builder.append(String.format("%02x", hash[i]));
            return builder.toString();
        } catch (Exception error) {
            throw new IllegalStateException(error);
        }
    }
}
