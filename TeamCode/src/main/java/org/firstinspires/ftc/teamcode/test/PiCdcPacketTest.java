package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.core.CameraChannel;
import org.firstinspires.ftc.teamcode.core.CameraFrameContract;
import org.firstinspires.ftc.teamcode.core.PiCdcPacket;

public final class PiCdcPacketTest {
    private static int checks;

    private static void check(boolean ok, String name) {
        if (!ok) throw new AssertionError(name);
        checks++;
    }

    private static String validLine(int hb) {
        return "{\"v\":1,\"hb\":" + hb + ",\"frame_valid\":true,"
                + "\"left\":{\"camera\":\"left\",\"found\":true,\"block_type\":1,\"class_name\":\"02\","
                + "\"confidence\":0.9,\"x\":0.4,\"y\":0.5},"
                + "\"right\":{\"camera\":\"right\",\"found\":true,\"block_type\":3,\"class_name\":\"04\","
                + "\"confidence\":0.8,\"x\":0.6,\"y\":0.5}}";
    }

    public static void main(String[] args) {
        long receivedNs = 1_000_000L;
        PiCdcPacket.Frame frame = PiCdcPacket.parse(validLine(1), receivedNs);
        check(frame.valid, "valid packet");
        check(frame.heartbeat == 1, "heartbeat");
        check(frame.receivedNs == receivedNs, "hub receive stamp");
        check(frame.left.found && frame.right.found, "both found");
        check(Math.abs(PiCdcPacket.dxPx(0.4, 640) - (-64.0)) < 0.01, "dxPx conversion");
        check(PiCdcPacket.centerPx(0.4, 640) == 256, "centerPx x");
        check(PiCdcPacket.centerPx(0.5, 480) == 240, "centerPx y");

        PiCdcPacket.Frame badVersion = PiCdcPacket.parse(validLine(1).replace("\"v\":1", "\"v\":2"), receivedNs);
        check(!badVersion.valid, "reject version");

        PiCdcPacket.Frame noHb = PiCdcPacket.parse(
                "{\"v\":1,\"frame_valid\":true,\"left\":{\"camera\":\"left\",\"found\":false,"
                        + "\"block_type\":-1,\"class_name\":\"\",\"confidence\":0,\"x\":0,\"y\":0},"
                        + "\"right\":{\"camera\":\"right\",\"found\":false,\"block_type\":-1,"
                        + "\"class_name\":\"\",\"confidence\":0,\"x\":0,\"y\":0}}",
                receivedNs);
        check(!noHb.valid, "reject missing heartbeat");

        PiCdcPacket.Frame badX = PiCdcPacket.parse(
                validLine(2).replace("\"x\":0.4", "\"x\":1.5"),
                receivedNs);
        check(!badX.valid, "reject out of range x");

        PiCdcPacket.Frame frameInvalid = PiCdcPacket.parse(
                validLine(3).replace("\"frame_valid\":true", "\"frame_valid\":false"),
                receivedNs);
        check(!frameInvalid.valid, "reject frame_valid false");

        CameraFrameContract contract = new CameraFrameContract(
                CameraChannel.WEBCAM1, receivedNs, true, PiCdcPacket.dxPx(0.4, 640),
                PiCdcPacket.centerPx(0.4, 640), PiCdcPacket.centerPx(0.5, 480),
                1, "02", true, 0, 1);
        check(contract.valid, "contract valid");
        check(contract.fresh(receivedNs + 10_000_000L, 50_000_000L), "fresh within budget");
        check(!contract.fresh(receivedNs + 200_000_000L, 50_000_000L), "stale rejects movement");

        check(PiCdcPacket.parse(null, receivedNs).valid == false, "null line");
        check(PiCdcPacket.parse("not json", receivedNs).valid == false, "malformed json");
        check(!PiCdcPacket.isValidBlockCode(4), "block code upper bound");
        check(PiCdcPacket.isValidBlockCode(0), "block code lower bound");

        System.out.println("PiCdcPacketTest passed " + checks + " checks");
    }
}
