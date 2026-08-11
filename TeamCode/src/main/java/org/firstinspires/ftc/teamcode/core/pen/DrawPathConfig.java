package org.firstinspires.ftc.teamcode.core.pen;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Cấu hình đường vẽ — tọa độ trong JSON là mm; các getter cm dùng cho MecanumDrive/Localizer.
 */
public final class DrawPathConfig {

    public static final int SCHEMA_VERSION = 1;

    public final String name;
    public final Pose startPose;
    public final double drawingHeadingDeg;
    public final List<Waypoint> waypoints;

    public DrawPathConfig(String name, Pose startPose, double drawingHeadingDeg, List<Waypoint> waypoints) {
        if (name == null || name.isEmpty()) throw new IllegalArgumentException("name");
        if (startPose == null) throw new IllegalArgumentException("startPose");
        if (waypoints == null || waypoints.isEmpty()) {
            throw new IllegalArgumentException("waypoints");
        }
        if (!Double.isFinite(drawingHeadingDeg)) {
            throw new IllegalArgumentException("drawingHeadingDeg");
        }
        this.name = name;
        this.startPose = startPose;
        this.drawingHeadingDeg = drawingHeadingDeg;
        this.waypoints = Collections.unmodifiableList(new ArrayList<>(waypoints));
    }

    public static final class Pose {
        public final double xMm;
        public final double yMm;
        public final double headingDeg;

        public Pose(double xMm, double yMm, double headingDeg) {
            if (!Double.isFinite(xMm) || !Double.isFinite(yMm) || !Double.isFinite(headingDeg)) {
                throw new IllegalArgumentException("pose must be finite");
            }
            this.xMm = xMm;
            this.yMm = yMm;
            this.headingDeg = headingDeg;
        }

        public double xCm() { return xMm / 10.0; }
        public double yCm() { return yMm / 10.0; }
    }

    public static final class Waypoint {
        public final double xMm;
        public final double yMm;
        public final boolean penDown;

        public Waypoint(double xMm, double yMm, boolean penDown) {
            if (!Double.isFinite(xMm) || !Double.isFinite(yMm)) {
                throw new IllegalArgumentException("waypoint must be finite");
            }
            this.xMm = xMm;
            this.yMm = yMm;
            this.penDown = penDown;
        }

        public double xCm() { return xMm / 10.0; }
        public double yCm() { return yMm / 10.0; }
    }
}
