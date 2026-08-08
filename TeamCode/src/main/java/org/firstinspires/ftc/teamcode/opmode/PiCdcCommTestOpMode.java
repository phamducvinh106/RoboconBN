package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.PiBlockReceiver;
import org.firstinspires.ftc.teamcode.core.PiCdcPacket;

@TeleOp(name = "Pi USB CDC Communication Test", group = "Test")
public final class PiCdcCommTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PiBlockReceiver receiver = new PiBlockReceiver(hardwareMap.appContext);
        boolean connected = receiver.start();
        telemetry.addData("USB CDC", connected ? "CONNECTED" : "NO DEVICE/PERMISSION");
        telemetry.addData("last error", receiver.getLastError());
        telemetry.addLine("Pi: python3 main.py --cdc-device /dev/ttyGS0");
        telemetry.update();
        waitForStart();

        try {
            while (opModeIsActive()) {
                PiCdcPacket.Frame frame = receiver.getLatestFrame();
                telemetry.addData("running", receiver.isRunning());
                telemetry.addData("parsed", receiver.getParsedCount());
                telemetry.addData("malformed", receiver.getMalformedCount());
                telemetry.addData("last hb", receiver.getLastHeartbeat());
                telemetry.addData("last error", receiver.getLastError());
                if (frame == null) {
                    telemetry.addData("frame", "WAITING");
                } else {
                    long ageMs = (System.nanoTime() - frame.receivedNs) / 1_000_000L;
                    telemetry.addData("frame", "valid=%s age=%dms hb=%d", frame.valid, ageMs, frame.heartbeat);
                    showSide("LEFT", frame.left, ageMs);
                    showSide("RIGHT", frame.right, ageMs);
                }
                telemetry.update();
                sleep(50);
            }
        } finally {
            receiver.close();
        }
    }

    private void showSide(String label, PiCdcPacket.ChannelDetection detection, long ageMs) {
        if (detection == null) {
            telemetry.addData(label, "missing");
            return;
        }
        telemetry.addData(label, "found=%s type=%d name=%s conf=%.2f age=%dms",
                detection.found, detection.blockType, detection.className, detection.confidence, ageMs);
        telemetry.addData(label + " pos", "x=%.3f y=%.3f dx=%.1f",
                detection.x, detection.y, PiCdcPacket.dxPx(detection.x, PiCdcPacket.DEFAULT_FRAME_WIDTH));
    }
}
