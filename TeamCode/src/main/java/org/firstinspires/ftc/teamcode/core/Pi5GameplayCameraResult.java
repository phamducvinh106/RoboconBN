package org.firstinspires.ftc.teamcode.core;

/** Dual-channel Pi5 CDC readings for gameplay state machine. */
public final class Pi5GameplayCameraResult implements LiftingSequenceStateMachine.CameraResult {
    private final CameraAdapterManager cameras;
    private final LiftingSequenceConfig config;
    private CameraFrameContract left = CameraFrameContract.invalid(CameraChannel.WEBCAM1, 0);
    private CameraFrameContract right = CameraFrameContract.invalid(CameraChannel.WEBCAM2, 0);

    public Pi5GameplayCameraResult(CameraAdapterManager cameras, LiftingSequenceConfig config) {
        if (cameras == null || config == null) throw new IllegalArgumentException("missing camera dependencies");
        this.cameras = cameras;
        this.config = config;
    }

    public void update(long nowNs) {
        left = cameras.reading(CameraChannel.WEBCAM1);
        right = cameras.reading(CameraChannel.WEBCAM2);
    }

    @Override
    public boolean leftFresh(long nowNs) {
        return left.fresh(nowNs, config.sensorStaleNs);
    }

    @Override
    public boolean rightFresh(long nowNs) {
        return right.fresh(nowNs, config.sensorStaleNs);
    }

    @Override
    public boolean leftValid() {
        return left.valid;
    }

    @Override
    public boolean rightValid() {
        return right.valid;
    }

    @Override
    public String leftBlockType() {
        return left.blockType;
    }

    @Override
    public String rightBlockType() {
        return right.blockType;
    }

    public CameraFrameContract leftReading() { return left; }
    public CameraFrameContract rightReading() { return right; }
}
