package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.core.*;

public final class LiftingHardwareManagerTest {
    static int passed;
    static void check(String n, boolean ok) { if (!ok) throw new AssertionError(n); passed++; }

    static final class B implements HardwareContracts.BinaryChannel {
        boolean value;
        final StringBuilder log = new StringBuilder();
        public boolean high() { return value; }
        public void setHigh(boolean v) { value = v; log.append(v ? '1' : '0'); }
    }

    static final class S implements HardwareContracts.BooleanSensor {
        boolean value;
        public boolean active() { return value; }
    }

    public static void main(String[] args) {
        S end = new S();
        B step = new B(), dir = new B();
        StepperElevatorManager lift = new StepperElevatorManager(step, dir, new EndstopManager(end), 10, 10, 20);
        end.value = true;
        check("home", lift.moveToward(0, 0));
        end.value = false;
        check("unknown gate", !lift.moveToward(2, 0));
        end.value = true;
        check("home reset", lift.moveToward(0, 1));
        end.value = false;
        check("pulse", !lift.moveToward(1, 20));
        check("step low before dir", step.log.indexOf("01") >= 0);

        S left = new S(), right = new S();
        IrSensorManager ir = new IrSensorManager(left, right);
        left.value = true;
        check("partial IR", !ir.bothActive());
        right.value = true;
        check("dual IR", ir.bothActive());

        CameraFrameContract invalid = CameraFrameContract.invalid(CameraChannel.WEBCAM1, 10);
        check("invalid camera stop", !invalid.authorizesMovement(10, 100));
        check("explicit channels", CameraChannel.WEBCAM2.identity.equals("webcam2"));

        int payload = Pi5PayloadDecoder.composePayload(128, 64, 1, 2);
        Pi5PayloadDecoder.Decoded decoded = Pi5PayloadDecoder.decode(payload);
        check("decode x", decoded.x == 128);
        check("decode y", decoded.y == 64);
        check("decode left", decoded.leftCode == 1);
        check("decode right", decoded.rightCode == 2);

        CameraAdapterManager cameras = new CameraAdapterManager(new PlaceholderCameraTransport(), 100);
        check("placeholder invalid", !cameras.movementAuthorized(CameraChannel.WEBCAM1, 50));

        ForkServoManager fork = new ForkServoManager((v) -> {}, (v) -> {}, .2, .8, .5, .5);
        fork.setPose(ForkServoManager.Pose.HOLD);
        check("fork hold", fork.pose() == ForkServoManager.Pose.HOLD);
        check("invalid pose", !new HardwareContracts.PoseReading(Double.NaN, 1, 0, 1).valid);

        ReleaseBackoutSensorManager release = new ReleaseBackoutSensorManager(left::active, () -> new HardwareContracts.PoseReading(1, 2, 3, 4));
        check("release reading", release.released() && release.reading().valid);

        System.out.println("RESULT: " + passed + " passed, 0 failed");
    }
}
