package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.core.CameraAdapterManager;
import org.firstinspires.ftc.teamcode.core.CameraChannel;
import org.firstinspires.ftc.teamcode.core.CameraFrameContract;
import org.firstinspires.ftc.teamcode.core.FtcPi5SoftUartLineReader;
import org.firstinspires.ftc.teamcode.core.LiftingSequenceConfig;
import org.firstinspires.ftc.teamcode.core.LiftingSequenceConfigLoader;
import org.firstinspires.ftc.teamcode.core.Pi5CameraTransportFactory;
import org.firstinspires.ftc.teamcode.core.Pi5UartCameraTransport;
import org.firstinspires.ftc.teamcode.core.Pi5UartLineReader;

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

        Pi5UartCameraTransport transport = Pi5CameraTransportFactory.createWithHubPolling(
                hardwareMap, config, System::nanoTime, this::idle);
        CameraAdapterManager cameras = new CameraAdapterManager(transport, config.sensorStaleNs);
        DigitalChannel rxPin = hardwareMap.get(DigitalChannel.class, config.pi5UartDeviceName);
        Pi5UartLineReader uartReader = transport.uartReader();
        FtcPi5SoftUartLineReader softUart = uartReader instanceof FtcPi5SoftUartLineReader
                ? (FtcPi5SoftUartLineReader) uartReader
                : null;

        telemetry.addLine("PI5 UART COMMUNICATION TEST");
        telemetry.addLine("Requires: digital input pi5UartRx (Pi GPIO14 TX -> this pin)");
        telemetry.addLine("Manual test: short pi5UartRx to GND -> pin=LOW, pinEdges/s>0 when removed");
        telemetry.addLine("Pi: python tools/pi5_bench.py --loop --port /dev/serial0 --baud 9600");
        telemetry.addData("config", "baud=%d device=%s staleMs=%.0f",
                config.pi5UartBaud, config.pi5UartDeviceName, config.sensorStaleNs / 1e6);
        telemetry.update();
        waitForStart();

        boolean lastPinState = rxPin.getState();
        int pinEdgesThisSec = 0;
        int pinEdgesPerSec = 0;
        long edgeWindowStartNs = System.nanoTime();

        while (opModeIsActive() && !isStopRequested()) {
            long now = System.nanoTime();
            CameraFrameContract left = cameras.reading(CameraChannel.WEBCAM1);
            CameraFrameContract right = cameras.reading(CameraChannel.WEBCAM2);
            Pi5UartCameraTransport.Diagnostics diag = transport.diagnostics();
            boolean linkOk = left.valid && cameras.movementAuthorized(CameraChannel.WEBCAM1, now);

            idle();
            boolean pinState = rxPin.getState();
            int samplerEdges = softUart == null ? 0 : softUart.pinEdgesPerSecond();
            if (pinState != lastPinState) {
                pinEdgesThisSec++;
                lastPinState = pinState;
            }
            if (now - edgeWindowStartNs >= 1_000_000_000L) {
                pinEdgesPerSec = pinEdgesThisSec;
                pinEdgesThisSec = 0;
                edgeWindowStartNs = now;
            }

            String uartMode = softUart == null ? "N/A" : "MAIN+IDLE";
            String hint = classifyHint(diag, linkOk, Math.max(pinEdgesPerSec, samplerEdges));

            telemetry.addData("LINK", linkOk ? "OK" : "WAITING");
            telemetry.addData("hint", hint);
            telemetry.addData("pin", pinState ? "HIGH" : "LOW");
            telemetry.addData("pinEdges/s", pinEdgesPerSec);
            telemetry.addData("samplerEdges/s", samplerEdges);
            telemetry.addData("uartMode", uartMode);
            telemetry.addData("uartRx", softUart == null ? "N/A"
                    : String.format("%s bits=%d queued=%d", softUart.receiverState(),
                    softUart.bitsLatchedCount(), softUart.bytesQueuedTotal()));
            telemetry.addData("rx", "bytes=%d framesOk=%d decodeErr=%d buf=%d",
                    diag.bytesReceived, diag.framesOk, diag.decodeErrors, diag.lineBufferLen);
            telemetry.addData("lastLine", diag.lastLine.isEmpty() ? "(none)" : diag.lastLine);
            telemetry.addData("lastStatus", diag.lastStatus < 0 ? "n/a" : String.format("0x%02X", diag.lastStatus));
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

    private static String classifyHint(
            Pi5UartCameraTransport.Diagnostics diag,
            boolean linkOk,
            int pinEdgesPerSec) {
        if (linkOk) {
            return "OK";
        }
        if (diag.bytesReceived == 0 && pinEdgesPerSec == 0) {
            return "NO_SIGNAL (check wire/port/GND)";
        }
        if (diag.bytesReceived == 0 && pinEdgesPerSec > 0) {
            return "SIGNAL_NO_BYTES (soft-UART/baud)";
        }
        if (diag.bytesReceived > 0 && diag.framesOk == 0 && diag.decodeErrors > 0) {
            return "BYTES_NO_DECODE (baud/noise, check lastLine)";
        }
        if (diag.bytesReceived > 0 && diag.framesOk == 0 && diag.decodeErrors == 0) {
            return "WAIT_LINE (bytes ok, no full line yet)";
        }
        if (diag.framesOk > 0) {
            return "WAIT_HB (frames ok, heartbeat stale)";
        }
        return "DECODING";
    }
}
