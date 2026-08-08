package org.firstinspires.ftc.teamcode.core;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

public final class FieldBlueConfigLoader {
    private static final Pattern POINT = Pattern.compile(
            "\\\"(placeAtFactory|shelfApproach|retreat)\\\"\\s*:\\s*\\{\\s*\\\"x\\\"\\s*:\\s*(-?[0-9.]+)"
                    + "\\s*,\\s*\\\"y\\\"\\s*:\\s*(-?[0-9.]+)\\s*,\\s*\\\"heading\\\"\\s*:\\s*(-?[0-9.]+)\\s*\\}");
    private static final Pattern SHELF_FAC = Pattern.compile(
            "\\\"(fac1|fac2|fac3)\\\"\\s*:\\s*\\{\\s*\\\"x\\\"\\s*:\\s*(-?[0-9.]+)"
                    + "\\s*,\\s*\\\"y\\\"\\s*:\\s*(-?[0-9.]+)\\s*,\\s*\\\"heading\\\"\\s*:\\s*(-?[0-9.]+)\\s*\\}");
    private static final Pattern SLOT = Pattern.compile(
            "\\\"near\\\"\\s*:\\s*\\{\\s*\\\"x\\\"\\s*:\\s*(-?[0-9.]+)\\s*,\\s*\\\"y\\\"\\s*:\\s*(-?[0-9.]+)"
                    + "\\s*,\\s*\\\"heading\\\"\\s*:\\s*(-?[0-9.]+)\\s*\\}\\s*,\\s*\\\"placement\\\"\\s*:\\s*\\{"
                    + "\\s*\\\"x\\\"\\s*:\\s*(-?[0-9.]+)\\s*,\\s*\\\"y\\\"\\s*:\\s*(-?[0-9.]+)"
                    + "\\s*,\\s*\\\"heading\\\"\\s*:\\s*(-?[0-9.]+)\\s*\\}");

    private FieldBlueConfigLoader() {}

    public static FieldBlueConfig load(String json) {
        if (json == null || json.trim().isEmpty()) throw new IllegalArgumentException("missing field config");
        Matcher versionMatcher = Pattern.compile("\\\"version\\\"\\s*:\\s*(\\d+)").matcher(json);
        if (!versionMatcher.find() || Integer.parseInt(versionMatcher.group(1)) != 1) {
            throw new IllegalArgumentException("wrong field config version");
        }
        LiftingSequenceConfig.Pose placeAtFactory = null;
        LiftingSequenceConfig.Pose shelfApproach = null;
        LiftingSequenceConfig.Pose retreat = null;
        Matcher pointMatcher = POINT.matcher(json);
        while (pointMatcher.find()) {
            LiftingSequenceConfig.Pose pose = pose(pointMatcher, 2);
            switch (pointMatcher.group(1)) {
                case "placeAtFactory": placeAtFactory = pose; break;
                case "shelfApproach": shelfApproach = pose; break;
                case "retreat": retreat = pose; break;
                default: break;
            }
        }
        if (placeAtFactory == null || shelfApproach == null || retreat == null) {
            throw new IllegalArgumentException("missing field points");
        }
        LiftingSequenceConfig.Pose[] shelfFacs = new LiftingSequenceConfig.Pose[3];
        Matcher shelfMatcher = SHELF_FAC.matcher(json);
        while (shelfMatcher.find()) {
            int idx = Integer.parseInt(shelfMatcher.group(1).substring(3)) - 1;
            if (idx < 0 || idx >= shelfFacs.length) throw new IllegalArgumentException("invalid shelf id");
            shelfFacs[idx] = pose(shelfMatcher, 2);
        }
        for (int i = 0; i < shelfFacs.length; i++) {
            if (shelfFacs[i] == null) throw new IllegalArgumentException("missing shelf fac" + (i + 1));
        }
        FieldBlueConfig.Slot[] slots = new FieldBlueConfig.Slot[4];
        Matcher slotMatcher = SLOT.matcher(json);
        int count = 0;
        while (slotMatcher.find()) {
            if (count >= 4) throw new IllegalArgumentException("too many factory slots");
            slots[count++] = new FieldBlueConfig.Slot(
                    new LiftingSequenceConfig.Pose(
                            Double.parseDouble(slotMatcher.group(1)),
                            Double.parseDouble(slotMatcher.group(2)),
                            Double.parseDouble(slotMatcher.group(3))),
                    new LiftingSequenceConfig.Pose(
                            Double.parseDouble(slotMatcher.group(4)),
                            Double.parseDouble(slotMatcher.group(5)),
                            Double.parseDouble(slotMatcher.group(6))));
        }
        if (count != 4) throw new IllegalArgumentException("missing factory slots");
        Matcher calibratedMatcher = Pattern.compile("\"calibrated\"\\s*:\\s*(true|false)").matcher(json);
        if (!calibratedMatcher.find()) throw new IllegalArgumentException("missing calibrated flag");
        boolean calibrated = Boolean.parseBoolean(calibratedMatcher.group(1));
        return new FieldBlueConfig(1, calibrated, placeAtFactory, shelfApproach, retreat, shelfFacs, slots);
    }

    private static LiftingSequenceConfig.Pose pose(Matcher matcher, int from) {
        return new LiftingSequenceConfig.Pose(
                Double.parseDouble(matcher.group(from)),
                Double.parseDouble(matcher.group(from + 1)),
                Double.parseDouble(matcher.group(from + 2)));
    }
}
