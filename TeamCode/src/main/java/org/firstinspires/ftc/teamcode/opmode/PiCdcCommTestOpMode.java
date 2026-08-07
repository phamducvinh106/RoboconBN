package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.PiBlockReceiver;

@TeleOp(name = "Pi USB CDC Communication Test", group = "Test")
public final class PiCdcCommTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PiBlockReceiver receiver = new PiBlockReceiver(hardwareMap.appContext);
        boolean connected = receiver.start();
        telemetry.addData("USB CDC", connected ? "CONNECTED" : "NO DEVICE/PERMISSION");
        telemetry.addLine("Pi must run: python3 main.py --cdc-device /dev/ttyGS0");
        telemetry.update();
        waitForStart();

        try {
            while (opModeIsActive()) {
                show("LEFT", receiver.getLatest("left"));
                show("RIGHT", receiver.getLatest("right"));
                telemetry.update();
                sleep(50);
            }
        } finally {
            receiver.close();
        }
    }

    private void show(String label, PiBlockReceiver.BlockDetection detection) {
        if (detection == null) {
            telemetry.addData(label, "WAITING");
            return;
        }
        long ageMs = System.currentTimeMillis() - detection.timestampMs;
        telemetry.addData(label, "found=%s type=%d name=%s conf=%.2f",
                detection.found, detection.blockType, detection.className, detection.confidence);
        telemetry.addData(label + " position", "x=%.3f y=%.3f age=%dms",
                detection.x, detection.y, ageMs);
    }
}
