package org.firstinspires.ftc.teamcode.core;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

public final class LiftingSequenceConfig {
    public static final int SCHEMA_VERSION = 1;
    public static final long IR_DEBOUNCE_NS = 100_000_000L;
    public static final long STATE_TIMEOUT_NS = 10_000_000_000L;
    public static final int MAX_RETRIES = 2;
    public static final double RELEASE_BACK_OUT_CM = 20.0;
    public static final double POSITION_TOLERANCE_CM = 1.0;
    public static final double HEADING_TOLERANCE_DEG = 3.0;
    public static final int TOTAL_CYCLES = 6;

    public enum ElevatorTarget { HOME, READY1, LIFT1, READY2, LIFT2 }
    public enum ForkPose { PLACE, HOLD }

    public static final class Pose {
        public final double x, y, heading;
        public Pose(double x, double y, double heading) {
            this.x = x;
            this.y = y;
            this.heading = heading;
        }
    }

    public static final class Factory {
        public final String type;
        public final Pose near, placement;
        public Factory(String type, Pose near, Pose placement) {
            this.type = type;
            this.near = near;
            this.placement = placement;
        }
    }

    public final int version;
    public final double placeLeft, placeRight, holdLeft, holdRight;
    public final long stepHighNs, stepLowNs, irDebounceNs, sensorStaleNs, elevatorTimeoutNs, stateTimeoutNs;
    public final int homeSteps, ready1Steps, lift1Steps, ready2Steps, lift2Steps, maxRetries, settleCycles, totalCycles;
    public final double releaseBackOutCm, positionToleranceCm, headingToleranceDeg, encoderFreshnessNs, noProgressCm;
    public final boolean endstopActiveLow, irActiveLow;
    public final String webcam1Identity, webcam2Identity;
    public final double approachSpeed;
    public final int cameraFrameWidth, cameraFrameHeight;
    public final Pose shelfApproach, retreat, placeAtFactory;
    public final String[] blockTypesByCode;
    public final Map<String, Factory> factories;
    public final String fingerprint;

    private LiftingSequenceConfig(int version, Map<String, Double> n, Map<String, Boolean> b,
                                  Pose placeAtFactory, Pose shelfApproach, Pose retreat,
                                  Map<String, Factory> factories, String fingerprint) {
        this.version = version;
        placeLeft = n.get("placeLeft");
        placeRight = n.get("placeRight");
        holdLeft = n.get("holdLeft");
        holdRight = n.get("holdRight");
        stepHighNs = n.get("stepHighNs").longValue();
        stepLowNs = n.get("stepLowNs").longValue();
        irDebounceNs = n.get("irDebounceNs").longValue();
        sensorStaleNs = n.get("sensorStaleNs").longValue();
        elevatorTimeoutNs = n.get("elevatorTimeoutNs").longValue();
        stateTimeoutNs = n.get("stateTimeoutNs").longValue();
        homeSteps = n.get("homeSteps").intValue();
        ready1Steps = n.get("ready1Steps").intValue();
        lift1Steps = n.get("lift1Steps").intValue();
        ready2Steps = n.get("ready2Steps").intValue();
        lift2Steps = n.get("lift2Steps").intValue();
        maxRetries = n.get("maxRetries").intValue();
        settleCycles = n.get("settleCycles").intValue();
        totalCycles = n.get("totalCycles").intValue();
        releaseBackOutCm = n.get("releaseBackOutCm");
        positionToleranceCm = n.get("positionToleranceCm");
        headingToleranceDeg = n.get("headingToleranceDeg");
        encoderFreshnessNs = n.get("encoderFreshnessNs");
        noProgressCm = n.get("noProgressCm");
        endstopActiveLow = b.get("endstopActiveLow");
        irActiveLow = b.get("irActiveLow");
        webcam1Identity = "webcam1";
        webcam2Identity = "webcam2";
        approachSpeed = n.get("approachSpeed");
        cameraFrameWidth = n.get("frameWidth").intValue();
        cameraFrameHeight = n.get("frameHeight").intValue();
        this.placeAtFactory = placeAtFactory;
        this.shelfApproach = shelfApproach;
        this.retreat = retreat;
        blockTypesByCode = new String[]{"01", "02", "03", "04"};
        this.factories = Collections.unmodifiableMap(factories);
        this.fingerprint = fingerprint;
    }

    public int elevatorSteps(ElevatorTarget target) {
        switch (target) {
            case HOME: return homeSteps;
            case READY1: return ready1Steps;
            case LIFT1: return lift1Steps;
            case READY2: return ready2Steps;
            case LIFT2: return lift2Steps;
            default: throw new IllegalArgumentException("unknown elevator target");
        }
    }

    public Factory factoryFor(String type) {
        Factory factory = factories.get(type);
        if (factory == null) throw new IllegalArgumentException("unknown block type: " + type);
        return factory;
    }

    public static Factory factory(String type) { return defaultFactory(type); }

    private static Factory defaultFactory(String type) {
        if (type == null || type.isEmpty()) throw new IllegalArgumentException("unknown block type: " + type);
        return new Factory(type, new Pose(0, 0, 0), new Pose(0, 0, 0));
    }

    public static void validate() {
        if (SCHEMA_VERSION != 1) throw new IllegalStateException("invalid config schema");
    }

    static LiftingSequenceConfig create(int v, Map<String, Double> n, Map<String, Boolean> b,
                                        Pose placeAtFactory, Pose shelfApproach, Pose retreat,
                                        Map<String, Factory> f, String fp) {
        return new LiftingSequenceConfig(v, n, b, placeAtFactory, shelfApproach, retreat, f, fp);
    }

    public LiftingSequenceConfig withField(FieldBlueConfig field, Alliance alliance) {
        return create(version, snapshotNumbers(), snapshotBooleans(),
                field.placeAtFactory, field.shelfApproach, field.retreat,
                field.factoriesFor(alliance), fingerprint);
    }

    private Map<String, Double> snapshotNumbers() {
        Map<String, Double> n = new LinkedHashMap<>();
        n.put("placeLeft", placeLeft);
        n.put("placeRight", placeRight);
        n.put("holdLeft", holdLeft);
        n.put("holdRight", holdRight);
        n.put("stepHighNs", (double) stepHighNs);
        n.put("stepLowNs", (double) stepLowNs);
        n.put("irDebounceNs", (double) irDebounceNs);
        n.put("sensorStaleNs", (double) sensorStaleNs);
        n.put("elevatorTimeoutNs", (double) elevatorTimeoutNs);
        n.put("stateTimeoutNs", (double) stateTimeoutNs);
        n.put("homeSteps", (double) homeSteps);
        n.put("ready1Steps", (double) ready1Steps);
        n.put("lift1Steps", (double) lift1Steps);
        n.put("ready2Steps", (double) ready2Steps);
        n.put("lift2Steps", (double) lift2Steps);
        n.put("maxRetries", (double) maxRetries);
        n.put("settleCycles", (double) settleCycles);
        n.put("totalCycles", (double) totalCycles);
        n.put("releaseBackOutCm", releaseBackOutCm);
        n.put("positionToleranceCm", positionToleranceCm);
        n.put("headingToleranceDeg", headingToleranceDeg);
        n.put("encoderFreshnessNs", encoderFreshnessNs);
        n.put("noProgressCm", noProgressCm);
        n.put("approachSpeed", approachSpeed);
        n.put("frameWidth", (double) cameraFrameWidth);
        n.put("frameHeight", (double) cameraFrameHeight);
        return n;
    }

    private Map<String, Boolean> snapshotBooleans() {
        Map<String, Boolean> b = new LinkedHashMap<>();
        b.put("endstopActiveLow", endstopActiveLow);
        b.put("irActiveLow", irActiveLow);
        return b;
    }
}
