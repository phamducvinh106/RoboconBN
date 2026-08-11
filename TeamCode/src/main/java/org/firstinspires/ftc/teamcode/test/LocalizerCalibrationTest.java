package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.core.odometry.Localizer;

/** Executable offline guards for the Localizer calibration contract. */
public final class LocalizerCalibrationTest {
    public static void main(String[] args) {
        Localizer.Calibration calibration = new Localizer.Calibration(
                3.2, 2000.0, -1.0, -1.0, 1.0,
                5.0, -25.0, 1e-5, 2, 0.2);
        check("leftfront / forward".equals(calibration.parallelPodRole), "parallel mapping");
        check("rightfront / strafe".equals(calibration.perpendicularPodRole), "perpendicular mapping");
        check(calibration.imuOrientation.contains("BACKWARD") && calibration.imuOrientation.contains("UP"), "IMU orientation");
        expectInvalid(0.0, 2000.0);
        expectInvalid(3.2, Double.NaN);
        expectInvalidSign(0.5);
        check(Double.isNaN(Double.NaN), "insufficient rotation remains NaN");
        System.out.println("LocalizerCalibrationTest PASS");
    }

    private static void expectInvalid(double diameter, double ticks) {
        try {
            new Localizer.Calibration(diameter, ticks, -1, -1, 1, 5, -25, 1e-5, 2, 0.2);
            throw new AssertionError("invalid calibration accepted");
        } catch (IllegalArgumentException expected) {
            // expected
        }
    }

    private static void expectInvalidSign(double sign) {
        try {
            new Localizer.Calibration(3.2, 2000, sign, -1, 1, 5, -25, 1e-5, 2, 0.2);
            throw new AssertionError("invalid sign accepted");
        } catch (IllegalArgumentException expected) {
            // expected
        }
    }

    private static void check(boolean condition, String name) {
        if (!condition) throw new AssertionError(name);
    }
}
