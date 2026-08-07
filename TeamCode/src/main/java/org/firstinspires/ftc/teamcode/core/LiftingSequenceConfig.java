package org.firstinspires.ftc.teamcode.core;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

public final class LiftingSequenceConfig {
    public static final int SCHEMA_VERSION = 1;
    public static final long IR_DEBOUNCE_NS=100_000_000L, STATE_TIMEOUT_NS=10_000_000_000L;
    public static final int MAX_RETRIES=2;
    public static final double RELEASE_BACK_OUT_CM=20.0, POSITION_TOLERANCE_CM=1.0, HEADING_TOLERANCE_DEG=3.0;
    public enum ElevatorTarget { HOME(0), READY1(844), LIFT1(1781), READY2(4688), LIFT2(5625); public final int steps; ElevatorTarget(int steps){this.steps=steps;} }
    public enum ForkPose { PLACE, HOLD }
    public static final class Pose { public final double x, y, heading; public Pose(double x, double y, double heading) { this.x=x; this.y=y; this.heading=heading; } }
    public static final class Factory { public final String type; public final Pose near, placement; public Factory(String type, Pose near, Pose placement) { this.type=type; this.near=near; this.placement=placement; } }

    public final int version;
    public final double placeLeft, placeRight, holdLeft, holdRight;
    public final long stepHighNs, stepLowNs, irDebounceNs, sensorStaleNs, elevatorTimeoutNs, stateTimeoutNs;
    public final int homeSteps, ready1Steps, lift1Steps, ready2Steps, lift2Steps, maxRetries, settleCycles;
    public final double releaseBackOutCm, positionToleranceCm, headingToleranceDeg, encoderFreshnessNs, noProgressCm;
    public final boolean endstopActiveLow, irActiveLow;
    public final String webcam1Identity, webcam2Identity;
    public final double scanSpeed, centerSpeed, approachSpeed;
    public final int cameraFrameWidth, pi5UartBaud;
    public final String pi5UartDeviceName;
    public final String[] blockTypesByCode;
    public final Map<String, Factory> factories;
    public final String fingerprint;
    public static final long STEP_HIGH_NS=1_000_000L, STEP_LOW_NS=1_000_000L;
    public static final double PLACE_LEFT=.25, PLACE_RIGHT=.75, HOLD_LEFT=.50, HOLD_RIGHT=.50;

    private LiftingSequenceConfig(int version, Map<String, Double> n, Map<String, Boolean> b, Map<String, Factory> factories, String fingerprint, String pi5UartDeviceName) {
        this.version=version; this.placeLeft=n.get("placeLeft"); this.placeRight=n.get("placeRight"); this.holdLeft=n.get("holdLeft"); this.holdRight=n.get("holdRight");
        stepHighNs=n.get("stepHighNs").longValue(); stepLowNs=n.get("stepLowNs").longValue(); irDebounceNs=n.get("irDebounceNs").longValue(); sensorStaleNs=n.get("sensorStaleNs").longValue(); elevatorTimeoutNs=n.get("elevatorTimeoutNs").longValue(); stateTimeoutNs=n.get("stateTimeoutNs").longValue();
        homeSteps=n.get("homeSteps").intValue(); ready1Steps=n.get("ready1Steps").intValue(); lift1Steps=n.get("lift1Steps").intValue(); ready2Steps=n.get("ready2Steps").intValue(); lift2Steps=n.get("lift2Steps").intValue(); maxRetries=n.get("maxRetries").intValue(); settleCycles=n.get("settleCycles").intValue();
        releaseBackOutCm=n.get("releaseBackOutCm"); positionToleranceCm=n.get("positionToleranceCm"); headingToleranceDeg=n.get("headingToleranceDeg"); encoderFreshnessNs=n.get("encoderFreshnessNs"); noProgressCm=n.get("noProgressCm");
        endstopActiveLow=b.get("endstopActiveLow"); irActiveLow=b.get("irActiveLow"); webcam1Identity="webcam1"; webcam2Identity="webcam2"; scanSpeed=n.get("scanSpeed"); centerSpeed=n.get("centerSpeed"); approachSpeed=n.get("approachSpeed"); cameraFrameWidth=n.get("cameraFrameWidth").intValue(); pi5UartBaud=n.get("pi5UartBaud").intValue(); this.pi5UartDeviceName=pi5UartDeviceName; blockTypesByCode=new String[]{"01","02","03","04"}; this.factories=Collections.unmodifiableMap(factories); this.fingerprint=fingerprint;
    }
    public Factory factoryFor(String type) { Factory f=factories.get(type); if(f==null) throw new IllegalArgumentException("unknown block type: "+type); return f; }
    public static Factory factory(String type) { return defaultFactory(type); }
    private static Factory defaultFactory(String type) { if(type==null||type.isEmpty()) throw new IllegalArgumentException("unknown block type: "+type); return new Factory(type,new Pose(0,0,0),new Pose(0,0,0)); }
    public static void validate() { if (SCHEMA_VERSION != 1) throw new IllegalStateException("invalid config schema"); }
    static LiftingSequenceConfig create(int v, Map<String,Double> n, Map<String,Boolean> b, Map<String,Factory> f, String fp, String uartDevice) { return new LiftingSequenceConfig(v,n,b,f,fp,uartDevice); }
}
