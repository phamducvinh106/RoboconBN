package org.firstinspires.ftc.teamcode.core;

public final class Pi5I2cCameraTransport implements HardwareContracts.CameraTransport {
    private final HardwareContracts.Clock clock;
    private final Pi5I2cBurstReader reader;
    private final long maxAgeNs;
    private final int frameWidth;
    private final String[] blockTypes;
    private Pi5CameraSnapshot snapshot = Pi5CameraSnapshot.incomplete(0);
    private int lastHeartbeat = -1;
    private long lastHeartbeatChangeNs = -1;

    public Pi5I2cCameraTransport(
            HardwareContracts.Clock clock,
            Pi5I2cBurstReader reader,
            long maxAgeNs,
            int frameWidth,
            String[] blockTypes) {
        if (clock == null || reader == null || maxAgeNs < 0 || frameWidth <= 0 || blockTypes == null || blockTypes.length != 4) {
            throw new IllegalArgumentException("invalid pi5 transport configuration");
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

    private void refresh() {
        long now = clock.nowNs();
        try {
            byte[] data = reader.readBurst(0, Pi5PayloadDecoder.REGISTER_COUNT);
            Pi5CameraSnapshot next = Pi5PayloadDecoder.fromRegisters(data, now);
            if (next.readComplete && next.frameValid && next.heartbeat != lastHeartbeat) {
                lastHeartbeat = next.heartbeat;
                lastHeartbeatChangeNs = now;
            }
            snapshot = next;
        } catch (InterruptedException interrupted) {
            Thread.currentThread().interrupt();
            snapshot = Pi5CameraSnapshot.incomplete(now);
        } catch (RuntimeException error) {
            snapshot = Pi5CameraSnapshot.incomplete(now);
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
