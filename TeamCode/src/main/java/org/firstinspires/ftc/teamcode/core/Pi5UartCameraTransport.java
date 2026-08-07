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
    private long lastGoodFrameNs = -1;
    private int lastStatus = -1;
    private Runnable hubIdleCallback;
    private int hubSamplesPerRefresh;

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

    public void enableHubPolling(Runnable idleCallback, int samplesPerRefresh) {
        if (idleCallback == null || samplesPerRefresh < 1) {
            throw new IllegalArgumentException("invalid hub polling configuration");
        }
        this.hubIdleCallback = idleCallback;
        this.hubSamplesPerRefresh = samplesPerRefresh;
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
        return new Diagnostics(bytesReceived, framesOk, decodeErrors, lastLine, lineBuffer.length(), lastStatus);
    }

    public Pi5UartLineReader uartReader() {
        return reader;
    }

    private void refresh() {
        pollHub();
        long now = clock.nowNs();
        byte[] chunk = reader.pollBytes();
        if (chunk.length > 0) {
            bytesReceived += chunk.length;
            lineBuffer.append(new String(chunk, StandardCharsets.US_ASCII));
            syncLineBuffer();
            int newline;
            while ((newline = lineBuffer.indexOf("\n")) >= 0) {
                String line = lineBuffer.substring(0, newline).trim();
                lineBuffer.delete(0, newline + 1);
                if (!line.isEmpty()) {
                    applyLine(line, now);
                }
            }
        }
    }

    private void syncLineBuffer() {
        int start = lineBuffer.indexOf(Pi5UartFrameCodec.PREFIX);
        if (start > 0) {
            lineBuffer.delete(0, start);
        } else if (start < 0 && lineBuffer.length() > 64) {
            lineBuffer.delete(0, lineBuffer.length() - 16);
        }
    }

    private void pollHub() {
        if (hubIdleCallback == null) {
            return;
        }
        for (int i = 0; i < hubSamplesPerRefresh; i++) {
            hubIdleCallback.run();
            reader.tickAfterHubIdle(clock.nowNs());
        }
    }

    private void applyLine(String line, long nowNs) {
        lastLine = line;
        try {
            Pi5UartFrameCodec.Pi5UartFrame frame = Pi5UartFrameCodec.decode(line);
            byte[] registers = Pi5UartFrameCodec.toRegisters(frame);
            Pi5CameraSnapshot next = Pi5PayloadDecoder.fromRegisters(registers, nowNs);
            lastStatus = frame.status;
            if (next.readComplete) {
                lastGoodFrameNs = nowNs;
                if (next.frameValid) {
                    framesOk++;
                }
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
        public final int lineBufferLen;
        public final int lastStatus;

        Diagnostics(int bytesReceived, int framesOk, int decodeErrors, String lastLine, int lineBufferLen, int lastStatus) {
            this.bytesReceived = bytesReceived;
            this.framesOk = framesOk;
            this.decodeErrors = decodeErrors;
            this.lastLine = lastLine == null ? "" : lastLine;
            this.lineBufferLen = lineBufferLen;
            this.lastStatus = lastStatus;
        }
    }

    private boolean heartbeatFresh(long nowNs) {
        if (lastGoodFrameNs < 0) {
            return false;
        }
        if (nowNs < lastGoodFrameNs) {
            return false;
        }
        return nowNs - lastGoodFrameNs <= maxAgeNs;
    }
}
