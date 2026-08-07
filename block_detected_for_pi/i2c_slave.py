from __future__ import annotations

import fcntl
import os
import threading
from dataclasses import dataclass

from .config import REGISTER_COUNT

I2C_SLAVE = 0x0703
I2C_SLAVE_FORCE = 0x0706


@dataclass
class RegisterState:
    registers: bytearray
    heartbeat: int = 0

    def update(self, frame: bytes) -> None:
        if len(frame) != REGISTER_COUNT:
            raise ValueError(f"expected {REGISTER_COUNT} register bytes")
        self.registers[:] = frame
        if frame[0] & 0x01:
            self.heartbeat = (self.heartbeat + 1) & 0xFF
            self.registers[4] = self.heartbeat


class RegisterFile:
    """Thread-safe 6-byte register map exposed to the Control Hub."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._state = RegisterState(bytearray(REGISTER_COUNT))

    @property
    def heartbeat(self) -> int:
        with self._lock:
            return self._state.heartbeat

    def publish(self, frame: bytes) -> int:
        with self._lock:
            self._state.update(frame)
            return self._state.heartbeat

    def snapshot(self) -> bytes:
        with self._lock:
            return bytes(self._state.registers)


class LinuxI2cSlave:
    """Serve register reads on /dev/i2c-* as a Linux I2C slave device."""

    def __init__(self, bus: int, address: int, register_file: RegisterFile) -> None:
        self._register_file = register_file
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._fd = os.open(f"/dev/i2c-{bus}", os.O_RDWR)
        fcntl.ioctl(self._fd, I2C_SLAVE_FORCE, address)
        self._pointer = 0

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._serve, name="pi5-i2c-slave", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        os.close(self._fd)

    def _serve(self) -> None:
        while not self._stop.is_set():
            try:
                request = os.read(self._fd, 32)
            except OSError:
                continue
            if not request:
                continue
            if len(request) == 1:
                self._pointer = request[0] % REGISTER_COUNT
            data = self._register_file.snapshot()
            try:
                os.write(self._fd, bytes([data[self._pointer]]))
                self._pointer = (self._pointer + 1) % REGISTER_COUNT
            except OSError:
                continue


class MockI2cSlave:
    """No-op backend for development machines without I2C slave support."""

    def __init__(self, register_file: RegisterFile) -> None:
        self.register_file = register_file

    def start(self) -> None:
        return

    def stop(self) -> None:
        return


def create_slave(bus: int, address: int, register_file: RegisterFile, *, mock: bool = False):
    if mock:
        return MockI2cSlave(register_file)
    return LinuxI2cSlave(bus, address, register_file)
