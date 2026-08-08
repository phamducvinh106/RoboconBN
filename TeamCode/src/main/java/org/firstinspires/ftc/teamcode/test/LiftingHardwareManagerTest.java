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

    static final class StubTransport implements HardwareContracts.CameraTransport {
        public CameraFrameContract read(CameraChannel channel) {
            return CameraFrameContract.invalid(channel, 0);
        }
    }

    public static void main(String[] args) {
        S endU = new S();
        StepperElevatorManager unhomed = new StepperElevatorManager(new B(), new B(), new EndstopManager(endU), 10, 10, 20);
        endU.value = false;
        check("unknown gate", !unhomed.moveToward(2, 0));

        S end = new S();
        B step = new B(), dir = new B();
        StepperElevatorManager lift = new StepperElevatorManager(step, dir, new EndstopManager(end), 10, 10, 20);
        end.value = false;
        check("must home before done", !lift.moveToward(0, 0));
        end.value = true;
        check("home", lift.moveToward(0, 0));
        end.value = false;
        check("move blocking", lift.moveToward(2, 0) && lift.position() == 2);
        end.value = true;
        check("home reset", lift.moveToward(0, 1));
        end.value = false;
        check("pulse", lift.moveToward(1, 20));
        check("position", lift.position() == 1);
        check("step low before dir", step.log.indexOf("01") >= 0);

        B stepInv = new B(), dirInv = new B();
        StepperElevatorManager inverted = new StepperElevatorManager(
                stepInv, dirInv, new EndstopManager(end), 10, 10, 20, true);
        end.value = true;
        inverted.moveToward(0, 0);
        end.value = false;
        inverted.moveToward(1, 0);
        check("inverted up dir low", !dirInv.value);

        S left = new S(), right = new S();
        IrSensorManager ir = new IrSensorManager(left, right);
        left.value = true;
        check("partial IR", !ir.bothActive());
        right.value = true;
        check("dual IR", ir.bothActive());

        CameraFrameContract invalid = CameraFrameContract.invalid(CameraChannel.WEBCAM1, 10);
        check("invalid camera stop", !invalid.authorizesMovement(10, 100));
        check("explicit channels", CameraChannel.WEBCAM2.identity.equals("webcam2"));

        long receivedNs = 1_000_000L;
        PiCdcPacket.Frame frame = PiCdcPacket.parse(
                "{\"v\":1,\"hb\":1,\"frame_valid\":true,"
                        + "\"left\":{\"camera\":\"left\",\"found\":true,\"block_type\":1,\"class_name\":\"02\","
                        + "\"confidence\":0.9,\"x\":0.4,\"y\":0.5},"
                        + "\"right\":{\"camera\":\"right\",\"found\":false,\"block_type\":-1,\"class_name\":\"\","
                        + "\"confidence\":0.0,\"x\":0.0,\"y\":0.0}}",
                receivedNs);
        check("cdc frame valid", frame.valid);
        check("cdc dxPx", Math.abs(PiCdcPacket.dxPx(0.4, 640) + 64.0) < 0.01);

        CameraAdapterManager cameras = new CameraAdapterManager(new StubTransport(), 100);
        check("stub invalid", !cameras.movementAuthorized(CameraChannel.WEBCAM1, 50));

        ForkServoManager fork = new ForkServoManager((v) -> {}, (v) -> {}, .2, .8, .5, .5);
        fork.setPose(ForkServoManager.Pose.HOLD);
        check("fork hold", fork.pose() == ForkServoManager.Pose.HOLD);
        check("invalid pose", !new HardwareContracts.PoseReading(Double.NaN, 1, 0, 1).valid);

        ReleaseBackoutSensorManager release = new ReleaseBackoutSensorManager(left::active, () -> new HardwareContracts.PoseReading(1, 2, 3, 4));
        check("release reading", release.released() && release.reading().valid);

        System.out.println("RESULT: " + passed + " passed, 0 failed");
    }
}
