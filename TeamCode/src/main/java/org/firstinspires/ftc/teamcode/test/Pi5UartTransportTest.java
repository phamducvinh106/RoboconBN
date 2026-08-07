package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.core.BufferPi5UartLineReader;
import org.firstinspires.ftc.teamcode.core.CameraAdapterManager;
import org.firstinspires.ftc.teamcode.core.CameraChannel;
import org.firstinspires.ftc.teamcode.core.HardwareContracts;
import org.firstinspires.ftc.teamcode.core.Pi5PayloadDecoder;
import org.firstinspires.ftc.teamcode.core.Pi5UartCameraTransport;
import org.firstinspires.ftc.teamcode.core.Pi5UartFrameCodec;

public final class Pi5UartTransportTest {
    static int passed;
    static void check(String name, boolean ok) { if (!ok) throw new AssertionError(name); passed++; }

    static final class Clock implements HardwareContracts.Clock {
        long n;
        public long nowNs() { return n; }
    }

    public static void main(String[] args) {
        int payload = Pi5PayloadDecoder.composePayload(128, 64, 1, 2);
        int status = Pi5PayloadDecoder.STATUS_PROTO_OK
                | Pi5PayloadDecoder.STATUS_FRAME_VALID
                | Pi5PayloadDecoder.STATUS_LEFT_FOUND
                | Pi5PayloadDecoder.STATUS_RIGHT_FOUND;
        String line = Pi5UartFrameCodec.encode(3, status, payload);
        Pi5UartFrameCodec.Pi5UartFrame frame = Pi5UartFrameCodec.decode(line);
        check("decode heartbeat", frame.heartbeat == 3);
        check("decode payload", frame.payload == payload);

        BufferPi5UartLineReader reader = BufferPi5UartLineReader.fromLines(line);
        Clock clock = new Clock();
        Pi5UartCameraTransport transport = new Pi5UartCameraTransport(
                clock,
                reader,
                100,
                320,
                new String[] {"01", "02", "03", "04"}
        );
        CameraAdapterManager cameras = new CameraAdapterManager(transport, 100);
        clock.n = 50;
        check("uart webcam1 valid", cameras.reading(CameraChannel.WEBCAM1).valid);
        check("uart movement authorized", cameras.movementAuthorized(CameraChannel.WEBCAM1, 50));
        check("uart webcam2 type", "03".equals(cameras.reading(CameraChannel.WEBCAM2).blockType));

        reader.clear();
        reader.offerLine("$V1,1,80,00000,00\n");
        clock.n = 60;
        check("bad crc blocked", !cameras.reading(CameraChannel.WEBCAM1).valid);

        System.out.println("RESULT: " + passed + " passed, 0 failed");
    }
}
