package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Software UART receiver on a Hub digital input (Pi TX -> digital IN).
 * Runs a background sampler at low baud for reliability.
 */
public final class FtcPi5SoftUartLineReader implements Pi5UartLineReader {
    private final DigitalChannel rx;
    private final int baud;
    private final long bitNs;
    private final ArrayRing buffer = new ArrayRing(512);
    private final Thread thread;
    private volatile boolean running = true;

    public FtcPi5SoftUartLineReader(DigitalChannel rx, int baud) {
        if (rx == null || baud < 300 || baud > 19200) {
            throw new IllegalArgumentException("invalid soft uart configuration");
        }
        this.rx = rx;
        this.baud = baud;
        this.bitNs = 1_000_000_000L / baud;
        rx.setMode(DigitalChannel.Mode.INPUT);
        this.thread = new Thread(this::sampleLoop, "pi5-soft-uart-rx");
        this.thread.setDaemon(true);
        this.thread.setPriority(Thread.MAX_PRIORITY);
        this.thread.start();
    }

    @Override
    public byte[] pollBytes() {
        return buffer.drain();
    }

    public boolean pinState() {
        return rx.getState();
    }

    public boolean isRunning() {
        return running;
    }

    public boolean isThreadAlive() {
        return thread.isAlive();
    }

    public void close() {
        running = false;
        thread.interrupt();
    }

    private void sampleLoop() {
        while (running) {
            int value = readByte();
            if (value >= 0) {
                buffer.offer((byte) value);
            }
        }
    }

    private int readByte() {
        waitForLow();
        if (!running) {
            return -1;
        }
        sleepHalfBit();
        int result = 0;
        for (int bit = 0; bit < 8; bit++) {
            sleepBit();
            if (rx.getState()) {
                result |= (1 << bit);
            }
        }
        sleepBit();
        return result;
    }

    private void waitForLow() {
        while (running && rx.getState()) {
            // Tight spin — Thread.sleep cannot reliably sample 104us start bits @ 9600 on Android.
        }
    }

    private void sleepHalfBit() {
        busyWait(bitNs / 2);
    }

    private void sleepBit() {
        busyWait(bitNs);
    }

    private void busyWait(long nanos) {
        long deadline = System.nanoTime() + nanos;
        while (running && System.nanoTime() < deadline) {
            // spin
        }
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
