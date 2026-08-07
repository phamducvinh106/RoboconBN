from __future__ import annotations

import threading
from dataclasses import dataclass

from .config import REGISTER_COUNT


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
    """Thread-safe 6-byte logical frame used for CDC publish + heartbeat tracking."""

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
