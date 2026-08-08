package org.firstinspires.ftc.teamcode.core;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

/** Validated Pi5 USB CDC NDJSON packet contract. */
public final class PiCdcPacket {
    public static final int PROTOCOL_VERSION = 1;
    public static final int MAX_LINE_BYTES = 2048;
    public static final int DEFAULT_FRAME_WIDTH = 640;
    public static final int DEFAULT_FRAME_HEIGHT = 480;

    private PiCdcPacket() {}

    public static final class ChannelDetection {
        public final String camera;
        public final boolean found;
        public final int blockType;
        public final String className;
        public final double confidence;
        public final double x;
        public final double y;
        public final boolean valid;

        private ChannelDetection(String camera, boolean found, int blockType, String className,
                                 double confidence, double x, double y, boolean valid) {
            this.camera = camera;
            this.found = found;
            this.blockType = blockType;
            this.className = className;
            this.confidence = confidence;
            this.x = x;
            this.y = y;
            this.valid = valid;
        }
    }

    public static final class Frame {
        public final int version;
        public final int heartbeat;
        public final boolean frameValid;
        public final long receivedNs;
        public final ChannelDetection left;
        public final ChannelDetection right;
        public final boolean valid;

        private Frame(int version, int heartbeat, boolean frameValid, long receivedNs,
                      ChannelDetection left, ChannelDetection right, boolean valid) {
            this.version = version;
            this.heartbeat = heartbeat;
            this.frameValid = frameValid;
            this.receivedNs = receivedNs;
            this.left = left;
            this.right = right;
            this.valid = valid;
        }

        static Frame invalid(long receivedNs) {
            return new Frame(-1, -1, false, receivedNs, null, null, false);
        }
    }

    public static Frame parse(String line, long receivedNs) {
        if (line == null || line.isEmpty() || line.length() > MAX_LINE_BYTES) {
            return Frame.invalid(receivedNs);
        }
        try {
            int version = readInt(line, "v", -1);
            int heartbeat = readInt(line, "hb", -1);
            if (version != PROTOCOL_VERSION || heartbeat < 0) return Frame.invalid(receivedNs);
            Boolean frameValid = readBoolean(line, "frame_valid");
            if (frameValid == null || !frameValid) return Frame.invalid(receivedNs);
            String leftJson = extractObject(line, "left");
            String rightJson = extractObject(line, "right");
            if (leftJson == null || rightJson == null) return Frame.invalid(receivedNs);
            ChannelDetection left = parseSide(leftJson, "left");
            ChannelDetection right = parseSide(rightJson, "right");
            if (left == null || right == null) return Frame.invalid(receivedNs);
            boolean valid = frameValid && left.valid && right.valid;
            return new Frame(version, heartbeat, frameValid, receivedNs, left, right, valid);
        } catch (RuntimeException ignored) {
            return Frame.invalid(receivedNs);
        }
    }

    private static ChannelDetection parseSide(String json, String expectedCamera) {
        String camera = readString(json, "camera");
        if (!expectedCamera.equals(camera)) return null;
        boolean found = Boolean.TRUE.equals(readBoolean(json, "found"));
        int blockType = readInt(json, "block_type", -1);
        String className = readString(json, "class_name");
        Double confidence = readDouble(json, "confidence");
        Double x = readDouble(json, "x");
        Double y = readDouble(json, "y");
        if (confidence == null || !Double.isFinite(confidence) || confidence < 0.0 || confidence > 1.0) {
            return null;
        }
        if (x == null || y == null || !Double.isFinite(x) || !Double.isFinite(y)
                || x < 0.0 || x > 1.0 || y < 0.0 || y > 1.0) {
            if (found) return null;
            x = 0.0;
            y = 0.0;
        }
        if (found && (!isValidBlockCode(blockType) || className == null || className.isEmpty())) {
            return null;
        }
        if (!found) blockType = -1;
        return new ChannelDetection(camera, found, blockType, className == null ? "" : className,
                confidence, x, y, true);
    }

    private static String extractObject(String json, String key) {
        String marker = "\"" + key + "\":";
        int markerIndex = json.indexOf(marker);
        if (markerIndex < 0) return null;
        int brace = json.indexOf('{', markerIndex);
        if (brace < 0) return null;
        int depth = 0;
        for (int i = brace; i < json.length(); i++) {
            char c = json.charAt(i);
            if (c == '{') depth++;
            else if (c == '}') {
                depth--;
                if (depth == 0) return json.substring(brace, i + 1);
            }
        }
        return null;
    }

    private static int readInt(String json, String key, int defaultValue) {
        Matcher matcher = Pattern.compile("\"" + Pattern.quote(key) + "\"\\s*:\\s*(-?\\d+)").matcher(json);
        return matcher.find() ? Integer.parseInt(matcher.group(1)) : defaultValue;
    }

    private static Double readDouble(String json, String key) {
        Matcher matcher = Pattern.compile(
                "\"" + Pattern.quote(key) + "\"\\s*:\\s*(-?\\d+(?:\\.\\d+)?(?:[eE][+-]?\\d+)?)"
        ).matcher(json);
        return matcher.find() ? Double.parseDouble(matcher.group(1)) : null;
    }

    private static Boolean readBoolean(String json, String key) {
        Matcher matcher = Pattern.compile("\"" + Pattern.quote(key) + "\"\\s*:\\s*(true|false)").matcher(json);
        if (!matcher.find()) return null;
        return Boolean.parseBoolean(matcher.group(1));
    }

    private static String readString(String json, String key) {
        Matcher matcher = Pattern.compile("\"" + Pattern.quote(key) + "\"\\s*:\\s*\"([^\"]*)\"").matcher(json);
        return matcher.find() ? matcher.group(1) : null;
    }

    public static boolean isValidBlockCode(int code) {
        return code >= 0 && code <= 3;
    }

    public static double dxPx(double normalizedX, int frameWidth) {
        if (frameWidth <= 0 || !Double.isFinite(normalizedX)) return Double.NaN;
        return (normalizedX - 0.5) * frameWidth;
    }

    public static int centerPx(double normalized, int frameSize) {
        if (frameSize <= 0 || !Double.isFinite(normalized)) return -1;
        return (int) Math.round(normalized * frameSize);
    }
}
