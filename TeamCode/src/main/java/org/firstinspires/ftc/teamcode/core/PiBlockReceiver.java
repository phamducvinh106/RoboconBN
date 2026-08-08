package org.firstinspires.ftc.teamcode.core;

import android.content.Context;
import android.hardware.usb.UsbConstants;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbEndpoint;
import android.hardware.usb.UsbInterface;
import android.hardware.usb.UsbManager;

import org.json.JSONObject;

import java.nio.charset.StandardCharsets;
import java.util.concurrent.atomic.AtomicReference;

/** Reads newline-delimited JSON from Pi USB CDC ACM gadget. */
public final class PiBlockReceiver implements AutoCloseable {
    public static final class BlockDetection {
        public final String camera;
        public final int blockType;
        public final double confidence;
        public final double x;
        public final double y;
        public final long timestampMs;
        public final boolean found;
        public final String className;

        private BlockDetection(JSONObject json) {
            camera = json.optString("camera", "unknown");
            blockType = json.optInt("block_type", -1);
            confidence = json.optDouble("confidence", 0.0);
            x = json.optDouble("x", 0.0);
            y = json.optDouble("y", 0.0);
            timestampMs = json.optLong("ts_ms", 0L);
            found = json.optBoolean("found", blockType >= 0);
            className = json.optString("class_name", "");
        }
    }

    private final UsbManager usbManager;
    private final AtomicReference<BlockDetection> latestLeft = new AtomicReference<>();
    private final AtomicReference<BlockDetection> latestRight = new AtomicReference<>();
    private volatile boolean running;
    private Thread readerThread;
    private UsbDeviceConnection connection;
    private UsbInterface usbInterface;
    private UsbEndpoint inEndpoint;

    public PiBlockReceiver(Context context) {
        usbManager = (UsbManager) context.getSystemService(Context.USB_SERVICE);
    }

    public synchronized boolean start() {
        if (running) return true;
        UsbDevice device = findCdcAcmDevice();
        if (device == null || !usbManager.hasPermission(device)) return false;
        for (int i = 0; i < device.getInterfaceCount(); i++) {
            UsbInterface candidate = device.getInterface(i);
            if (candidate.getInterfaceClass() == UsbConstants.USB_CLASS_CDC_DATA) {
                usbInterface = candidate;
                break;
            }
        }
        if (usbInterface == null) return false;
        for (int i = 0; i < usbInterface.getEndpointCount(); i++) {
            UsbEndpoint endpoint = usbInterface.getEndpoint(i);
            if (endpoint.getDirection() == UsbConstants.USB_DIR_IN
                    && endpoint.getType() == UsbConstants.USB_ENDPOINT_XFER_BULK) {
                inEndpoint = endpoint;
                break;
            }
        }
        connection = usbManager.openDevice(device);
        if (connection == null || inEndpoint == null || !connection.claimInterface(usbInterface, true)) {
            close();
            return false;
        }
        running = true;
        readerThread = new Thread(this::readLoop, "pi-cdc-reader");
        readerThread.start();
        return true;
    }

    public BlockDetection getLatest() {
        BlockDetection left = latestLeft.get();
        return left != null ? left : latestRight.get();
    }

    public BlockDetection getLatest(String camera) {
        return "left".equals(camera) ? latestLeft.get() : "right".equals(camera) ? latestRight.get() : null;
    }

    private void readLoop() {
        byte[] buffer = new byte[512];
        StringBuilder lines = new StringBuilder();
        while (running) {
            int count = connection.bulkTransfer(inEndpoint, buffer, buffer.length, 100);
            if (count <= 0) continue;
            lines.append(new String(buffer, 0, count, StandardCharsets.US_ASCII));
            int newline;
            while ((newline = lines.indexOf("\n")) >= 0) {
                String line = lines.substring(0, newline).trim();
                lines.delete(0, newline + 1);
                if (line.isEmpty()) continue;
                try {
                    JSONObject packet = new JSONObject(line);
                    latestLeft.set(new BlockDetection(packet.getJSONObject("left")));
                    latestRight.set(new BlockDetection(packet.getJSONObject("right")));
                } catch (Exception ignored) {
                    // Ignore incomplete or malformed frames; next frame remains usable.
                }
            }
        }
    }

    private UsbDevice findCdcAcmDevice() {
        for (UsbDevice device : usbManager.getDeviceList().values()) {
            for (int i = 0; i < device.getInterfaceCount(); i++) {
                int type = device.getInterface(i).getInterfaceClass();
                if (type == UsbConstants.USB_CLASS_COMM || type == UsbConstants.USB_CLASS_CDC_DATA) {
                    return device;
                }
            }
        }
        return null;
    }

    @Override
    public synchronized void close() {
        running = false;
        if (readerThread != null) readerThread.interrupt();
        if (connection != null && usbInterface != null) connection.releaseInterface(usbInterface);
        if (connection != null) connection.close();
        readerThread = null;
        connection = null;
        usbInterface = null;
        inEndpoint = null;
    }
}
