package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.core.MecanumDrive;

/**
 * Offline checks for MecanumDrive math + simulated goToPosition.
 * Run: java org.firstinspires.ftc.teamcode.test.MecanumDriveTest
 */
public final class MecanumDriveTest {

    static final class FakeOdo implements MecanumDrive.OdometryProvider {
        double xCm, yCm, headingDeg;

        @Override public double getX() { return xCm; }
        @Override public double getY() { return yCm; }
        @Override public double getHeadingDeg() { return headingDeg; }

        void step(double fl, double fr, double bl, double br, double dtSec) {
            double forward = (fl + fr + bl + br) / 4.0;
            double strafe = (fl - fr - bl + br) / 4.0;
            double rotate = (fl - fr + bl - br) / 4.0;
            double rad = Math.toRadians(headingDeg);
            double cos = Math.cos(rad);
            double sin = Math.sin(rad);
            double gx = strafe * cos - forward * sin;
            double gy = strafe * sin + forward * cos;
            xCm += 80.0 * gx * dtSec;
            yCm += 80.0 * gy * dtSec;
            headingDeg += 120.0 * rotate * dtSec;
            if (headingDeg > 180) headingDeg -= 360;
            if (headingDeg < -180) headingDeg += 360;
        }
    }

    static int passed, failed;

    static void check(String name, boolean cond) {
        if (cond) { passed++; System.out.println("  PASS " + name); }
        else { failed++; System.out.println("  FAIL " + name); }
    }

    static void checkEq(String name, double actual, double expected, double tol) {
        check(name, Math.abs(actual - expected) <= tol);
    }

    static void testMixForward() {
        double[] p = MecanumDrive.mixMecanum(1, 0, 0);
        checkEq("forward fl", p[0], 1, 1e-9);
        checkEq("forward fr", p[1], 1, 1e-9);
        checkEq("forward bl", p[2], 1, 1e-9);
        checkEq("forward br", p[3], 1, 1e-9);
    }

    static void testMixStrafeLeft() {
        double[] p = MecanumDrive.mixMecanum(0, 1, 0);
        checkEq("strafe fl", p[0], 1, 1e-9);
        checkEq("strafe fr", p[1], -1, 1e-9);
        checkEq("strafe bl", p[2], -1, 1e-9);
        checkEq("strafe br", p[3], 1, 1e-9);
    }

    static void testMixRotate() {
        double[] p = MecanumDrive.mixMecanum(0, 0, 1);
        checkEq("rotate fl", p[0], 1, 1e-9);
        checkEq("rotate fr", p[1], -1, 1e-9);
        checkEq("rotate bl", p[2], 1, 1e-9);
        checkEq("rotate br", p[3], -1, 1e-9);
    }

    static void testFieldTransform0() {
        double[] r = MecanumDrive.fieldToRobot(1, 0, 0);
        checkEq("h0 forward", r[0], 1, 1e-9);
        checkEq("h0 strafe", r[1], 0, 1e-9);
    }

    static void testFieldTransform90() {
        double[] r = MecanumDrive.fieldToRobot(1, 0, 90);
        checkEq("h90 forward", r[0], 0, 1e-9);
        checkEq("h90 strafe", r[1], 1, 1e-9);
    }

    static void testFieldTransform180() {
        double[] r = MecanumDrive.fieldToRobot(0, 1, 180);
        checkEq("h180 forward", r[0], 0, 1e-9);
        checkEq("h180 strafe", r[1], -1, 1e-9);
    }

    static void testHeadingWrap() {
        checkEq("wrap +20", MecanumDrive.wrapHeadingError(20), 20, 1e-9);
        checkEq("wrap -340", MecanumDrive.wrapHeadingError(-340), 20, 1e-9);
        checkEq("wrap +350", MecanumDrive.wrapHeadingError(350), -10, 1e-9);
    }

    static void testPowerLimit() {
        double[] p = MecanumDrive.normalizePowers(2, 2, 2, 2);
        checkEq("limit fl", p[0], 1, 1e-9);
        checkEq("limit br", p[3], 1, 1e-9);
    }

    static void testZeroErrorIdleMotors() {
        FakeOdo odo = new FakeOdo();
        MecanumDrive drive = MecanumDrive.forTest(odo);
        drive.goToPosition(0, 0, 0);
        drive.update();
        checkEq("zero err fl", drive.getLastFlPower(), 0, 0.05);
        check("zero err state", drive.getState() == MecanumDrive.DriveState.HOLDING);
    }

    static void testFieldCentricForward() {
        FakeOdo odo = new FakeOdo();
        MecanumDrive drive = MecanumDrive.forTest(odo);
        drive.driveFieldCentric(1, 0, 0);
        odo.step(drive.getLastFlPower(), drive.getLastFrPower(),
                drive.getLastBlPower(), drive.getLastBrPower(), 0.02);
        checkEq("field fwd X", odo.xCm, 0, 0.01);
        check("field fwd Y+", odo.yCm > 0);
    }

    static void testFieldCentricStrafeRight() {
        FakeOdo odo = new FakeOdo();
        MecanumDrive drive = MecanumDrive.forTest(odo);
        drive.driveFieldCentric(0, 1, 0);
        odo.step(drive.getLastFlPower(), drive.getLastFrPower(),
                drive.getLastBlPower(), drive.getLastBrPower(), 0.02);
        checkEq("field strafe Y", odo.yCm, 0, 0.01);
        check("field strafe X+", odo.xCm > 0);
    }

    static void testGoToConverge() {
        FakeOdo odo = new FakeOdo();
        MecanumDrive drive = MecanumDrive.forTest(odo);
        drive.setSlewPerLoop(1.0);
        drive.goToPosition(50, 0, 0);
        for (int i = 0; i < 600; i++) {
            drive.update();
            odo.step(drive.getLastFlPower(), drive.getLastFrPower(),
                    drive.getLastBlPower(), drive.getLastBrPower(), 0.02);
            if (drive.atTarget()) break;
        }
        checkEq("goto X", odo.xCm, 50, 1.5);
        checkEq("goto Y", odo.yCm, 0, 1.5);
        check("goto HOLDING", drive.getState() == MecanumDrive.DriveState.HOLDING);
    }

    static void testTimeout() throws InterruptedException {
        FakeOdo odo = new FakeOdo();
        MecanumDrive drive = MecanumDrive.forTest(odo);
        drive.setSlewPerLoop(1.0);
        drive.goToPosition(500, 0, 0);
        drive.update();
        check("goto remains moving", drive.getState() == MecanumDrive.DriveState.MOVING);
        check("goto still powered", drive.getLastFlPower() != 0);
    }

    public static void main(String[] args) throws InterruptedException {
        System.out.println("MecanumDrive offline tests");
        passed = failed = 0;
        testMixForward();
        testMixStrafeLeft();
        testMixRotate();
        testFieldTransform0();
        testFieldTransform90();
        testFieldTransform180();
        testHeadingWrap();
        testPowerLimit();
        testZeroErrorIdleMotors();
        testFieldCentricForward();
        testFieldCentricStrafeRight();
        testGoToConverge();
        testTimeout();
        System.out.printf("%nRESULT: %d passed, %d failed%n", passed, failed);
        if (failed > 0) System.exit(1);
    }
}
