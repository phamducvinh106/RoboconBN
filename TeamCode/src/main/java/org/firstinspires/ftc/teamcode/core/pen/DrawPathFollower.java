package org.firstinspires.ftc.teamcode.core.pen;

import org.firstinspires.ftc.teamcode.core.drive.MecanumDrive;

/**
 * Theo waypoint JSON — heading cố định suốt bài vẽ.
 *
 * Luồng mỗi waypoint:
 * 1. Nếu penDown đổi so với waypoint trước → nhấc/hạ bút trước khi di chuyển
 * 2. {@code drive.goToPosition(xCm, yCm, drawingHeadingDeg)}
 * 3. Chờ {@link MecanumDrive#atTarget()} liên tiếp {@code settleCycles} vòng
 * 4. Sang waypoint tiếp theo
 */
public final class DrawPathFollower {

    public enum State { RUNNING, DONE }

    private final MecanumDrive drive;
    private final PenServoManager pen;
    private final DrawPathConfig path;
    private final int settleCycles;

    private int waypointIndex;
    private int settleCount;
    private Boolean lastPenDown;
    private boolean waypointPrimed;
    private State state = State.RUNNING;

    public DrawPathFollower(MecanumDrive drive, PenServoManager pen,
                            DrawPathConfig path, int settleCycles) {
        if (drive == null || pen == null || path == null) {
            throw new IllegalArgumentException("missing follower dependency");
        }
        if (settleCycles < 1) throw new IllegalArgumentException("settleCycles");
        this.drive = drive;
        this.pen = pen;
        this.path = path;
        this.settleCycles = settleCycles;
        this.waypointIndex = 0;
        this.settleCount = 0;
        this.waypointPrimed = false;
    }

    public void tick() {
        if (state == State.DONE) return;
        if (waypointIndex >= path.waypoints.size()) {
            state = State.DONE;
            return;
        }

        DrawPathConfig.Waypoint waypoint = path.waypoints.get(waypointIndex);

        if (!waypointPrimed) {
            // Bước 1: chuyển trạng thái bút trước khi travel (tránh kéo vết)
            if (lastPenDown == null || lastPenDown != waypoint.penDown) {
                if (waypoint.penDown) {
                    pen.penDown();
                } else {
                    pen.penUp();
                }
                lastPenDown = waypoint.penDown;
            }
            // Bước 2: ra lệnh tới đích với heading vẽ cố định
            drive.goToPosition(waypoint.xCm(), waypoint.yCm(), path.drawingHeadingDeg);
            waypointPrimed = true;
            settleCount = 0;
        }

        // Bước 3: settle — phải atTarget liên tiếp đủ settleCycles vòng
        if (drive.atTarget()) {
            settleCount++;
            if (settleCount >= settleCycles) {
                waypointIndex++;
                waypointPrimed = false;
                settleCount = 0;
                if (waypointIndex >= path.waypoints.size()) {
                    state = State.DONE;
                }
            }
        } else {
            settleCount = 0;
        }
    }

    public State getState() {
        return state;
    }

    public boolean isDone() {
        return state == State.DONE;
    }

    public int getWaypointIndex() {
        return waypointIndex;
    }

    public int getWaypointCount() {
        return path.waypoints.size();
    }

    public DrawPathConfig.Waypoint getCurrentWaypoint() {
        if (waypointIndex >= path.waypoints.size()) return null;
        return path.waypoints.get(waypointIndex);
    }
}
