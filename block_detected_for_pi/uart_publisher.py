from __future__ import annotations

import threading
from typing import Protocol

from .payload import build_registers
from .uart_protocol import format_frame


class UartPort(Protocol):
    def write(self, data: bytes) -> int: ...

    def flush(self) -> None: ...


class SerialUartPort:
    """Wrap pyserial port for publishing."""

    def __init__(self, port: str, baud: int) -> None:
        import serial  # type: ignore[import-untyped]

        self._serial = serial.Serial(port=port, baudrate=baud, timeout=0)

    def write(self, data: bytes) -> int:
        return self._serial.write(data)

    def flush(self) -> None:
        self._serial.flush()

    def close(self) -> None:
        self._serial.close()


class MockUartPort:
    """Capture frames for tests and development machines."""

    def __init__(self) -> None:
        self.frames: list[str] = []
        self._lock = threading.Lock()

    def write(self, data: bytes) -> int:
        with self._lock:
            self.frames.append(data.decode("ascii"))
        return len(data)

    def flush(self) -> None:
        return

    def close(self) -> None:
        return


class UartPublisher:
    def __init__(self, port: UartPort) -> None:
        self._port = port

    def publish(self, frame: bytes) -> None:
        if len(frame) != 6:
            raise ValueError("expected 6-byte register frame")
        status = frame[0]
        payload = (frame[1] & 0xFF) | ((frame[2] & 0xFF) << 8) | ((frame[3] & 0x0F) << 16)
        heartbeat = frame[4]
        line = format_frame(heartbeat=heartbeat, status=status, payload=payload)
        self._port.write(line.encode("ascii"))
        self._port.flush()

    def close(self) -> None:
        self._port.close()


def frame_to_uart_line(frame: bytes) -> str:
    if len(frame) != 6:
        raise ValueError("expected 6-byte register frame")
    status = frame[0]
    payload = (frame[1] & 0xFF) | ((frame[2] & 0xFF) << 8) | ((frame[3] & 0x0F) << 16)
    heartbeat = frame[4]
    return format_frame(heartbeat=heartbeat, status=status, payload=payload)


def registers_to_uart_line(
    *,
    payload: int,
    frame_valid: bool,
    left_found: bool,
    right_found: bool,
    heartbeat: int,
    proto_version: int = 1,
) -> str:
    frame = build_registers(
        payload=payload,
        frame_valid=frame_valid,
        left_found=left_found,
        right_found=right_found,
        heartbeat=heartbeat,
        proto_version=proto_version,
    )
    return frame_to_uart_line(frame)


def create_uart_publisher(port: str, baud: int, *, mock: bool = False) -> UartPublisher:
    if mock:
        return UartPublisher(MockUartPort())
    return UartPublisher(SerialUartPort(port, baud))
