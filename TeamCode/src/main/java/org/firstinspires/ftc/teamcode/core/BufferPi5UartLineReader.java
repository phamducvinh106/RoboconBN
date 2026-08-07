package org.firstinspires.ftc.teamcode.core;

import java.util.ArrayDeque;
import java.util.Arrays;

public final class BufferPi5UartLineReader implements Pi5UartLineReader {
    private final ArrayDeque<Byte> buffer = new ArrayDeque<>();

    public void offer(byte[] data) {
        if (data == null || data.length == 0) {
            return;
        }
        synchronized (buffer) {
            for (byte value : data) {
                buffer.add(value);
            }
        }
    }

    public void offerLine(String line) {
        offer(line.getBytes());
    }

    @Override
    public byte[] pollBytes() {
        synchronized (buffer) {
            if (buffer.isEmpty()) {
                return new byte[0];
            }
            byte[] out = new byte[buffer.size()];
            for (int i = 0; i < out.length; i++) {
                Byte value = buffer.pollFirst();
                out[i] = value == null ? 0 : value;
            }
            return out;
        }
    }

    public int size() {
        synchronized (buffer) {
            return buffer.size();
        }
    }

    public void clear() {
        synchronized (buffer) {
            buffer.clear();
        }
    }

    public static BufferPi5UartLineReader fromLines(String... lines) {
        BufferPi5UartLineReader reader = new BufferPi5UartLineReader();
        for (String line : lines) {
            reader.offerLine(line);
        }
        return reader;
    }

    public static byte[] concat(byte[] first, byte[] second) {
        if (first == null || first.length == 0) {
            return second == null ? new byte[0] : Arrays.copyOf(second, second.length);
        }
        if (second == null || second.length == 0) {
            return Arrays.copyOf(first, first.length);
        }
        byte[] out = Arrays.copyOf(first, first.length + second.length);
        System.arraycopy(second, 0, out, first.length, second.length);
        return out;
    }
}
