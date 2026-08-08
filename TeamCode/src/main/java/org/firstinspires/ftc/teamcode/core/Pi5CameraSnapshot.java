package org.firstinspires.ftc.teamcode.core;

public final class Pi5CameraSnapshot {
    public final long timestampNs;
    public final boolean readComplete;
    public final boolean frameValid;
    public final boolean leftFound;
    public final boolean rightFound;
    public final int protoVersion;
    public final int heartbeat;
    public final int payload;
    public final Pi5PayloadDecoder.Decoded decoded;

    public Pi5CameraSnapshot(
            long timestampNs,
            boolean readComplete,
            boolean frameValid,
            boolean leftFound,
            boolean rightFound,
            int protoVersion,
            int heartbeat,
            int payload,
            Pi5PayloadDecoder.Decoded decoded) {
        this.timestampNs = timestampNs;
        this.readComplete = readComplete;
        this.frameValid = frameValid;
        this.leftFound = leftFound;
        this.rightFound = rightFound;
        this.protoVersion = protoVersion;
        this.heartbeat = heartbeat;
        this.payload = payload;
        this.decoded = decoded;
    }

    public static Pi5CameraSnapshot incomplete(long timestampNs) {
        return new Pi5CameraSnapshot(timestampNs, false, false, false, false, 0, -1, 0,
                new Pi5PayloadDecoder.Decoded(0, 0, -1, -1));
    }

    public static Pi5CameraSnapshot invalid(long timestampNs, int heartbeat, int payload) {
        return new Pi5CameraSnapshot(timestampNs, true, false, false, false,
                Pi5PayloadDecoder.PROTO_VERSION, heartbeat, payload,
                Pi5PayloadDecoder.decode(payload));
    }

    public CameraFrameContract toFrame(CameraChannel channel, int frameWidth, String[] blockTypes) {
        if (!readComplete || !frameValid) {
            return CameraFrameContract.invalid(channel, timestampNs);
        }
        if (channel == CameraChannel.WEBCAM1) {
            if (!leftFound || !Pi5PayloadDecoder.isValidBlockCode(decoded.leftCode)) {
                return CameraFrameContract.invalid(channel, timestampNs);
            }
            String blockType = Pi5PayloadDecoder.blockTypeForCode(decoded.leftCode, blockTypes);
            if (blockType == null) {
                return CameraFrameContract.invalid(channel, timestampNs);
            }
            double dxPx = Pi5PayloadDecoder.dxPx(decoded.x, frameWidth);
            return new CameraFrameContract(
                    channel,
                    timestampNs,
                    true,
                    dxPx,
                    decoded.x,
                    decoded.y,
                    decoded.leftCode,
                    blockType,
                    true,
                    payload,
                    heartbeat
            );
        }
        if (channel == CameraChannel.WEBCAM2) {
            if (!rightFound || !Pi5PayloadDecoder.isValidBlockCode(decoded.rightCode)) {
                return CameraFrameContract.invalid(channel, timestampNs);
            }
            String blockType = Pi5PayloadDecoder.blockTypeForCode(decoded.rightCode, blockTypes);
            if (blockType == null) {
                return CameraFrameContract.invalid(channel, timestampNs);
            }
            return new CameraFrameContract(
                    channel,
                    timestampNs,
                    true,
                    Double.NaN,
                    decoded.x,
                    decoded.y,
                    decoded.rightCode,
                    blockType,
                    true,
                    payload,
                    heartbeat
            );
        }
        return CameraFrameContract.invalid(channel, timestampNs);
    }
}
