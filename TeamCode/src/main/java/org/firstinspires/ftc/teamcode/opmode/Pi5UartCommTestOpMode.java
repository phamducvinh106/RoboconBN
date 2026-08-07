package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.core.CameraAdapterManager;
import org.firstinspires.ftc.teamcode.core.CameraChannel;
import org.firstinspires.ftc.teamcode.core.CameraFrameContract;
import org.firstinspires.ftc.teamcode.core.LiftingSequenceConfig;
import org.firstinspires.ftc.teamcode.core.LiftingSequenceConfigLoader;
import org.firstinspires.ftc.teamcode.core.Pi5CameraTransportFactory;
import org.firstinspires.ftc.teamcode.core.Pi5UartCameraTransport;

import java.io.InputStream;
import java.nio.charset.StandardCharsets;

/**
 * Minimal Pi5 UART link test — only requires digital input {@code pi5UartRx}.
 * Run {@code python tools/pi5_bench.py --loop} on the Pi while this OpMode is active.
 */
@TeleOp(name = "Pi5 UART Communication Test", group = "Test")
public final class Pi5UartCommTestOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        LiftingSequenceConfig config;
        try (InputStream input = hardwareMap.appContext.getAssets().open("phase2-lifting-config.json")) {
            java.io.ByteArrayOutputStream output = new java.io.ByteArrayOutputStream();
            byte[] buffer = new byte[1024];
            int count;
            while ((count = input.read(buffer)) != -1) {
                output.write(buffer, 0, count);
            }
            config = LiftingSequenceConfigLoader.load(new String(output.toByteArray(), StandardCharsets.UTF_8));
        } catch (Exception error) {
            telemetry.addData("LINK", "config error: %s", error.getMessage());
            telemetry.update();
            return;
        }

        Pi5UartCameraTransport transport = (Pi5UartCameraTransport) Pi5CameraTransportFactory.create(
                hardwareMap, config, System::nanoTime);
        CameraAdapterManager cameras = new CameraAdapterManager(transport, config.sensorStaleNs);

        telemetry.addLine("PI5 UART COMMUNICATION TEST");
        telemetry.addLine("Requires: digital input pi5UartRx (Pi GPIO14 TX -> this pin)");
        telemetry.addLine("Pi: python tools/pi5_bench.py --loop --uart-port /dev/serial0");
        telemetry.addData("config", "baud=%d device=%s staleMs=%.0f",
                config.pi5UartBaud, config.pi5UartDeviceName, config.sensorStaleNs / 1e6);
        telemetry.update();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            long now = System.nanoTime();
            CameraFrameContract left = cameras.reading(CameraChannel.WEBCAM1);
            CameraFrameContract right = cameras.reading(CameraChannel.WEBCAM2);
            Pi5UartCameraTransport.Diagnostics diag = transport.diagnostics();
            boolean linkOk = left.valid && cameras.movementAuthorized(CameraChannel.WEBCAM1, now);

            telemetry.addData("LINK", linkOk ? "OK" : "WAITING");
            telemetry.addData("rx", "bytes=%d framesOk=%d decodeErr=%d", diag.bytesReceived, diag.framesOk, diag.decodeErrors);
            telemetry.addData("lastLine", diag.lastLine.isEmpty() ? "(none)" : diag.lastLine);
            telemetry.addData("webcam1", "valid=%s fresh=%s hb=%d type=%s payload=0x%05X dx=%.1f",
                    left.valid, cameras.movementAuthorized(CameraChannel.WEBCAM1, now),
                    left.heartbeat, left.blockType, left.rawPayload, left.dxPx);
            telemetry.addData("webcam2", "valid=%s fresh=%s hb=%d type=%s payload=0x%05X",
                    right.valid, cameras.movementAuthorized(CameraChannel.WEBCAM2, now),
                    right.heartbeat, right.blockType, right.rawPayload);
            telemetry.update();
            idle();
        }
    }
}
