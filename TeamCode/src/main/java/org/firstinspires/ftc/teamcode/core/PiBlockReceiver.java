package org.firstinspires.ftc.teamcode.core;

import android.app.PendingIntent;
import android.content.Context;
import android.content.Intent;
import android.hardware.usb.UsbConstants;
import android.hardware.usb.UsbDevice;
import android.hardware.usb.UsbDeviceConnection;
import android.hardware.usb.UsbEndpoint;
import android.hardware.usb.UsbInterface;
import android.hardware.usb.UsbManager;

import java.nio.charset.StandardCharsets;
import java.util.concurrent.atomic.AtomicReference;

/** Reads validated newline-delimited JSON from Pi USB CDC ACM gadget. */
public final class PiBlockReceiver implements AutoCloseable {
    public static final String ACTION_USB_PERMISSION = "org.firstinspires.ftc.teamcode.USB_PERMISSION";
    private static final int PREFERRED_VENDOR_ID = 0x0525;
    private static final int PREFERRED_PRODUCT_ID = 0xA4A7;

    private final Context context;
    private final UsbManager usbManager;
    private final AtomicReference<PiCdcPacket.Frame> latestFrame = new AtomicReference<>();
    private volatile boolean running;
    private volatile int lastHeartbeat = -1;
    private volatile int malformedCount;
    private volatile int parsedCount;
    private volatile String lastError = "";
    private volatile int disconnectCount;
    private Thread readerThread;
    private UsbDeviceConnection connection;
    private UsbInterface usbInterface;
    private UsbEndpoint inEndpoint;

    public PiBlockReceiver(Context context) {
        this.context = context.getApplicationContext();
        usbManager = (UsbManager) this.context.getSystemService(Context.USB_SERVICE);
    }

    public void requestPermission() {
        UsbDevice device = findCdcAcmDevice();
        if (device == null || usbManager.hasPermission(device)) return;
        Intent intent = new Intent(ACTION_USB_PERMISSION);
        intent.setPackage(context.getPackageName());
        PendingIntent pending = PendingIntent.getBroadcast(
                context, 0, intent, PendingIntent.FLAG_IMMUTABLE);
        usbManager.requestPermission(device, pending);
    }

    public synchronized boolean restart() {
        close();
        return start();
    }

    public synchronized boolean start() {
        if (running) return true;
        UsbDevice device = findCdcAcmDevice();
        if (device == null) {
            lastError = "no_cdc_device";
            return false;
        }
        if (!usbManager.hasPermission(device)) {
            requestPermission();
            lastError = "no_usb_permission";
            return false;
        }
        for (int i = 0; i < device.getInterfaceCount(); i++) {
            UsbInterface candidate = device.getInterface(i);
            if (candidate.getInterfaceClass() == UsbConstants.USB_CLASS_CDC_DATA) {
                usbInterface = candidate;
                break;
            }
        }
        if (usbInterface == null) {
            lastError = "no_cdc_data_interface";
            return false;
        }
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
            lastError = "open_or_claim_failed";
            close();
            return false;
        }
        running = true;
        lastError = "";
        readerThread = new Thread(this::readLoop, "pi-cdc-reader");
        readerThread.start();
        return true;
    }

    public PiCdcPacket.Frame getLatestFrame() {
        return latestFrame.get();
    }

    public PiCdcPacket.ChannelDetection getLatest(String camera) {
        PiCdcPacket.Frame frame = latestFrame.get();
        if (frame == null || !frame.valid) return null;
        return "left".equals(camera) ? frame.left : "right".equals(camera) ? frame.right : null;
    }

    public int getLastHeartbeat() { return lastHeartbeat; }
    public int getMalformedCount() { return malformedCount; }
    public int getParsedCount() { return parsedCount; }
    public String getLastError() { return lastError; }
    public boolean isRunning() { return running; }

    private void readLoop() {
        byte[] buffer = new byte[512];
        StringBuilder lines = new StringBuilder();
        while (running) {
            int count = connection.bulkTransfer(inEndpoint, buffer, buffer.length, 100);
            if (count < 0) {
                disconnectCount++;
                if (disconnectCount > 50) {
                    running = false;
                    lastError = "usb_disconnect";
                }
                continue;
            }
            disconnectCount = 0;
            if (count == 0) continue;
            lines.append(new String(buffer, 0, count, StandardCharsets.US_ASCII));
            if (lines.length() > PiCdcPacket.MAX_LINE_BYTES) {
                lines.setLength(0);
                malformedCount++;
                lastError = "line_overflow";
                continue;
            }
            int newline;
            while ((newline = lines.indexOf("\n")) >= 0) {
                String line = lines.substring(0, newline).trim();
                lines.delete(0, newline + 1);
                if (line.isEmpty()) continue;
                long receivedNs = System.nanoTime();
                PiCdcPacket.Frame frame = PiCdcPacket.parse(line, receivedNs);
                if (!frame.valid) {
                    malformedCount++;
                    lastError = "parse_reject";
                    continue;
                }
                if (lastHeartbeat >= 0 && frame.heartbeat == lastHeartbeat) {
                    malformedCount++;
                    lastError = "duplicate_heartbeat";
                    continue;
                }
                lastHeartbeat = frame.heartbeat;
                parsedCount++;
                lastError = "";
                latestFrame.set(frame);
            }
        }
    }

    private UsbDevice findCdcAcmDevice() {
        UsbDevice preferred = null;
        UsbDevice fallback = null;
        for (UsbDevice device : usbManager.getDeviceList().values()) {
            boolean cdc = false;
            for (int i = 0; i < device.getInterfaceCount(); i++) {
                int type = device.getInterface(i).getInterfaceClass();
                if (type == UsbConstants.USB_CLASS_COMM || type == UsbConstants.USB_CLASS_CDC_DATA) {
                    cdc = true;
                    break;
                }
            }
            if (!cdc) continue;
            if (device.getVendorId() == PREFERRED_VENDOR_ID
                    && device.getProductId() == PREFERRED_PRODUCT_ID) {
                preferred = device;
                break;
            }
            if (fallback == null) fallback = device;
        }
        return preferred != null ? preferred : fallback;
    }

    @Override
    public synchronized void close() {
        running = false;
        if (readerThread != null) readerThread.interrupt();
        if (connection != null && usbInterface != null) {
            try { connection.releaseInterface(usbInterface); } catch (RuntimeException ignored) { }
        }
        if (connection != null) {
            try { connection.close(); } catch (RuntimeException ignored) { }
        }
        if (readerThread != null) {
            try { readerThread.join(200); } catch (InterruptedException ignored) { }
        }
        readerThread = null;
        connection = null;
        usbInterface = null;
        inEndpoint = null;
        latestFrame.set(null);
        lastHeartbeat = -1;
        disconnectCount = 0;
    }
}
