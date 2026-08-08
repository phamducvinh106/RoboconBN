package org.firstinspires.ftc.teamcode.core;

public final class Pi5PayloadDecoder {
    public static final int X_MASK = 0x000FF;
    public static final int Y_MASK = 0x0FF00;
    public static final int LEFT_TYPE_MASK = 0x30000;
    public static final int RIGHT_TYPE_MASK = 0xC0000;
    public static final int STATUS_FRAME_VALID = 0x01;
    public static final int STATUS_LEFT_FOUND = 0x02;
    public static final int STATUS_RIGHT_FOUND = 0x04;
    public static final int STATUS_PROTO_OK = 0x80;
    public static final int PROTO_VERSION = 1;
    public static final int REGISTER_COUNT = 6;

    private Pi5PayloadDecoder() {}

    public static final class Decoded {
        public final int x;
        public final int y;
        public final int leftCode;
        public final int rightCode;

        public Decoded(int x, int y, int leftCode, int rightCode) {
            this.x = x;
            this.y = y;
            this.leftCode = leftCode;
            this.rightCode = rightCode;
        }
    }

    public static Decoded decode(int payload) {
        return new Decoded(
                payload & X_MASK,
                (payload & Y_MASK) >> 8,
                (payload & LEFT_TYPE_MASK) >> 16,
                (payload & RIGHT_TYPE_MASK) >> 18
        );
    }

    public static int composePayload(int x, int y, int leftCode, int rightCode) {
        return (x & 0xFF)
                | ((y & 0xFF) << 8)
                | ((leftCode & 0x3) << 16)
                | ((rightCode & 0x3) << 18);
    }

    public static double dxPx(int x, int frameWidth) {
        if (frameWidth <= 0) return Double.NaN;
        return ((x / 255.0) - 0.5) * frameWidth;
    }

    public static String blockTypeForCode(int code, String[] blockTypes) {
        if (blockTypes == null || code < 0 || code >= blockTypes.length) return null;
        String type = blockTypes[code];
        return type == null || type.isEmpty() ? null : type;
    }

    public static boolean isValidBlockCode(int code) {
        return code >= 0 && code <= 3;
    }

    public static Pi5CameraSnapshot fromRegisters(byte[] data, long timestampNs) {
        if (data == null || data.length < REGISTER_COUNT) {
            return Pi5CameraSnapshot.incomplete(timestampNs);
        }
        int status = data[0] & 0xFF;
        if ((status & STATUS_PROTO_OK) == 0 || data[5] != PROTO_VERSION) {
            return Pi5CameraSnapshot.invalid(timestampNs, data[4] & 0xFF, 0);
        }
        int payload = (data[1] & 0xFF) | ((data[2] & 0xFF) << 8) | ((data[3] & 0x0F) << 16);
        return new Pi5CameraSnapshot(
                timestampNs,
                true,
                (status & STATUS_FRAME_VALID) != 0,
                (status & STATUS_LEFT_FOUND) != 0,
                (status & STATUS_RIGHT_FOUND) != 0,
                data[5] & 0xFF,
                data[4] & 0xFF,
                payload,
                decode(payload)
        );
    }
}
