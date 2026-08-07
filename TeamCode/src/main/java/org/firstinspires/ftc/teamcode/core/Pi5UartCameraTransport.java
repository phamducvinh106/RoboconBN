package org.firstinspires.ftc.teamcode.core;

import java.nio.charset.StandardCharsets;

public final class Pi5UartCameraTransport implements HardwareContracts.CameraTransport {
    private final HardwareContracts.Clock clock;
    private final Pi5UartLineReader reader;
    private final long maxAgeNs;
    private final int frameWidth;
    private final String[] blockTypes;
    private final StringBuilder lineBuffer = new StringBuilder();
    private Pi5CameraSnapshot snapshot = Pi5CameraSnapshot.incomplete(0);
    private int lastHeartbeat = -1;
    private long lastHeartbeatChangeNs = -1;
    private int bytesReceived;
    private int framesOk;
    private int decodeErrors;
    private String lastLine = "";

    public Pi5UartCameraTransport(
            HardwareContracts.Clock clock,
            Pi5UartLineReader reader,
            long maxAgeNs,
            int frameWidth,
            String[] blockTypes) {
        if (clock == null || reader == null || maxAgeNs < 0 || frameWidth <= 0 || blockTypes == null || blockTypes.length != 4) {
            throw new IllegalArgumentException("invalid pi5 uart transport configuration");
        }
        this.clock = clock;
        this.reader = reader;
        this.maxAgeNs = maxAgeNs;
        this.frameWidth = frameWidth;
        this.blockTypes = blockTypes;
    }

    @Override
    public CameraFrameContract read(CameraChannel channel) {
        refresh();
        CameraFrameContract frame = snapshot.toFrame(channel, frameWidth, blockTypes);
        if (!heartbeatFresh(clock.nowNs())) {
            return CameraFrameContract.invalid(channel, frame.timestampNs);
        }
        return frame;
    }

    public Diagnostics diagnostics() {
        return new Diagnostics(bytesReceived, framesOk, decodeErrors, lastLine);
    }

    private void refresh() {
        long now = clock.nowNs();
        byte[] chunk = reader.pollBytes();
        if (chunk.length > 0) {
            bytesReceived += chunk.length;
            lineBuffer.append(new String(chunk, StandardCharsets.US_ASCII));
            int newline;
            while ((newline = lineBuffer.indexOf("\n")) >= 0) {
                String line = lineBuffer.substring(0, newline);
                lineBuffer.delete(0, newline + 1);
                applyLine(line, now);
            }
        }
    }

    private void applyLine(String line, long nowNs) {
        lastLine = line;
        try {
            Pi5UartFrameCodec.Pi5UartFrame frame = Pi5UartFrameCodec.decode(line);
            byte[] registers = Pi5UartFrameCodec.toRegisters(frame);
            Pi5CameraSnapshot next = Pi5PayloadDecoder.fromRegisters(registers, nowNs);
            if (next.readComplete && next.frameValid) {
                framesOk++;
                if (next.heartbeat != lastHeartbeat) {
                    lastHeartbeat = next.heartbeat;
                    lastHeartbeatChangeNs = nowNs;
                }
            }
            snapshot = next;
        } catch (IllegalArgumentException ignored) {
            decodeErrors++;
            snapshot = Pi5CameraSnapshot.incomplete(nowNs);
        }
    }

    public static final class Diagnostics {
        public final int bytesReceived;
        public final int framesOk;
        public final int decodeErrors;
        public final String lastLine;

        Diagnostics(int bytesReceived, int framesOk, int decodeErrors, String lastLine) {
            this.bytesReceived = bytesReceived;
            this.framesOk = framesOk;
            this.decodeErrors = decodeErrors;
            this.lastLine = lastLine == null ? "" : lastLine;
        }
    }

    private boolean heartbeatFresh(long nowNs) {
        if (lastHeartbeat < 0 || lastHeartbeatChangeNs < 0) {
            return false;
        }
        if (nowNs < lastHeartbeatChangeNs) {
            return false;
        }
        return nowNs - lastHeartbeatChangeNs <= maxAgeNs;
    }
}
