package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Software UART receiver on a Hub digital input (Pi TX -> digital IN).
 * Samples on the main OpMode thread after {@code idle()} so Lynx refreshes the pin cache.
 */
public final class FtcPi5SoftUartLineReader implements Pi5UartLineReader {
    private static final int STATE_IDLE = 0;
    private static final int STATE_RECEIVING = 1;

    private final DigitalChannel rx;
    private final long bitNs;
    private final ArrayRing buffer = new ArrayRing(512);
    private int state = STATE_IDLE;
    private long startNs;
    private int result;
    private int bitsLatched;
    private boolean lastPinHigh = true;
    private int pinEdgesThisSec;
    private int pinEdgesPerSec;
    private int bytesQueuedTotal;
    private long edgeWindowStartNs = System.nanoTime();

    public FtcPi5SoftUartLineReader(DigitalChannel rx, int baud) {
        if (rx == null || baud < 300 || baud > 19200) {
            throw new IllegalArgumentException("invalid soft uart configuration");
        }
        this.rx = rx;
        this.bitNs = Math.round(1_000_000_000.0 / baud);
        rx.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public byte[] pollBytes() {
        return buffer.drain();
    }

    @Override
    public void tickAfterHubIdle(long nowNs) {
        tickAfterIdle(nowNs);
    }

    public void tickAfterIdle(long nowNs) {
        boolean pinHigh = rx.getState();
        if (pinHigh != lastPinHigh) {
            pinEdgesThisSec++;
        }
        if (nowNs - edgeWindowStartNs >= 1_000_000_000L) {
            pinEdgesPerSec = pinEdgesThisSec;
            pinEdgesThisSec = 0;
            edgeWindowStartNs = nowNs;
        }

        if (state == STATE_IDLE) {
            if (!pinHigh && lastPinHigh) {
                beginFrame(nowNs);
            }
        } else {
            long elapsed = nowNs - startNs;
            for (int bit = 0; bit < 8; bit++) {
                long sampleNs = bitNs * (bit + 1) + bitNs / 2;
                int mask = 1 << bit;
                if ((bitsLatched & mask) == 0 && elapsed >= sampleNs) {
                    if (pinHigh) {
                        result |= mask;
                    }
                    bitsLatched |= mask;
                }
            }
            if (elapsed >= bitNs * 9 + bitNs / 2) {
                if (bitsLatched == 0xFF && pinHigh) {
                    buffer.offer((byte) result);
                    bytesQueuedTotal++;
                }
                state = STATE_IDLE;
            } else if (elapsed > bitNs * 12) {
                state = STATE_IDLE;
            }
        }
        lastPinHigh = pinHigh;
    }

    private void beginFrame(long nowNs) {
        state = STATE_RECEIVING;
        startNs = nowNs;
        result = 0;
        bitsLatched = 0;
    }

    public boolean pinState() {
        return rx.getState();
    }

    public int pinEdgesPerSecond() {
        return pinEdgesPerSec;
    }

    public int bytesQueuedTotal() {
        return bytesQueuedTotal;
    }

    public String receiverState() {
        return state == STATE_IDLE ? "IDLE" : "RECV";
    }

    public int bitsLatchedCount() {
        return Integer.bitCount(bitsLatched);
    }

    public static int recommendedHubSamples(int baud) {
        long charNs = Math.round(10_000_000_000.0 / baud);
        int samples = (int) Math.ceil(charNs / 800_000.0);
        return Math.max(16, Math.min(samples, 40));
    }

    private static final class ArrayRing {
        private final byte[] data;
        private int head;
        private int tail;
        private int size;

        ArrayRing(int capacity) {
            data = new byte[capacity];
        }

        synchronized void offer(byte value) {
            if (size >= data.length) {
                head = (head + 1) % data.length;
                size--;
            }
            data[tail] = value;
            tail = (tail + 1) % data.length;
            size++;
        }

        synchronized byte[] drain() {
            if (size == 0) {
                return new byte[0];
            }
            byte[] out = new byte[size];
            for (int i = 0; i < out.length; i++) {
                out[i] = data[head];
                head = (head + 1) % data.length;
            }
            size = 0;
            return out;
        }
    }
}
