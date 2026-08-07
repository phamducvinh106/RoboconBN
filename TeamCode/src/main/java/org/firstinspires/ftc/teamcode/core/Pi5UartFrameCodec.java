package org.firstinspires.ftc.teamcode.core;

public final class Pi5UartFrameCodec {
    public static final String PREFIX = "$V1,";

    private Pi5UartFrameCodec() {}

    public static String encode(int heartbeat, int status, int payload) {
        String body = PREFIX + (heartbeat & 0xFF) + "," + (status & 0xFF) + ","
                + String.format("%05X", payload & 0xFFFFF);
        int crc = crc8Ascii(body);
        return body + "," + String.format("%02X", crc) + "\n";
    }

    public static Pi5UartFrame decode(String line) {
        if (line == null) {
            throw new IllegalArgumentException("missing line");
        }
        String text = line.trim();
        if (!text.startsWith(PREFIX)) {
            throw new IllegalArgumentException("invalid prefix");
        }
        String[] parts = text.split(",");
        if (parts.length != 5) {
            throw new IllegalArgumentException("invalid field count");
        }
        int heartbeat = Integer.parseInt(parts[1], 10);
        int status = Integer.parseInt(parts[2], 10);
        int payload = Integer.parseInt(parts[3], 16);
        int expected = Integer.parseInt(parts[4], 16);
        String body = parts[0] + "," + parts[1] + "," + parts[2] + "," + parts[3];
        if (crc8Ascii(body) != expected) {
            throw new IllegalArgumentException("crc mismatch");
        }
        if (payload < 0 || payload > 0xFFFFF) {
            throw new IllegalArgumentException("payload out of range");
        }
        return new Pi5UartFrame(heartbeat, status, payload);
    }

    public static byte[] toRegisters(Pi5UartFrame frame) {
        return new byte[] {
                (byte) frame.status,
                (byte) (frame.payload & 0xFF),
                (byte) ((frame.payload >> 8) & 0xFF),
                (byte) ((frame.payload >> 16) & 0x0F),
                (byte) frame.heartbeat,
                (byte) Pi5PayloadDecoder.PROTO_VERSION
        };
    }

    public static int crc8Ascii(String body) {
        int value = 0;
        for (int i = 0; i < body.length(); i++) {
            value ^= body.charAt(i);
        }
        return value & 0xFF;
    }

    public static final class Pi5UartFrame {
        public final int heartbeat;
        public final int status;
        public final int payload;

        public Pi5UartFrame(int heartbeat, int status, int payload) {
            this.heartbeat = heartbeat;
            this.status = status;
            this.payload = payload;
        }
    }
}
