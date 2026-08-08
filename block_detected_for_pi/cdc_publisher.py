from __future__ import annotations

import json
import math
import queue
import threading
from typing import Any

PROTOCOL_VERSION = 1
MAX_LINE_BYTES = 2048


class CdcPublisher:
    """Publish newline-delimited JSON through Pi USB CDC gadget."""

    def __init__(self, device: str = "/dev/ttyGS0") -> None:
        import serial

        self._serial_timeout = serial.SerialTimeoutException
        self._serial = serial.Serial(device, 115200, timeout=0, write_timeout=0.01)
        self._pending: queue.Queue[bytes | None] = queue.Queue(maxsize=1)
        self._worker = threading.Thread(target=self._write_loop, name="cdc-writer", daemon=True)
        self._heartbeat = 0
        self.dropped = 0
        self.written = 0
        self.last_error = ""
        self._worker.start()

    @property
    def heartbeat(self) -> int:
        return self._heartbeat

    def _write_loop(self) -> None:
        while (data := self._pending.get()) is not None:
            try:
                written = self._serial.write(data)
                if written == len(data):
                    self.written += 1
                    self.last_error = ""
                else:
                    self.dropped += 1
                    self.last_error = f"short write {written}/{len(data)}"
            except (self._serial_timeout, OSError) as error:
                self.dropped += 1
                self.last_error = type(error).__name__

    @staticmethod
    def _side_packet(camera: str, target: Any) -> dict[str, object]:
        found = bool(target.found)
        confidence = float(target.confidence) if found else 0.0
        x = float(target.x) if found else 0.0
        y = float(target.y) if found else 0.0
        if found and (
            not math.isfinite(confidence)
            or confidence < 0.0
            or confidence > 1.0
            or not math.isfinite(x)
            or not math.isfinite(y)
            or x < 0.0
            or x > 1.0
            or y < 0.0
            or y > 1.0
        ):
            found = False
            confidence = 0.0
            x = 0.0
            y = 0.0
        return {
            "camera": camera,
            "found": found,
            "block_type": int(target.class_id) if found else -1,
            "class_name": target.class_name if found else "",
            "confidence": confidence,
            "x": x,
            "y": y,
        }

    def publish(self, *, left: Any, right: Any, frame_valid: bool) -> bool:
        self._heartbeat = (self._heartbeat + 1) & 0xFFFFFFFF
        payload = {
            "v": PROTOCOL_VERSION,
            "hb": self._heartbeat,
            "frame_valid": bool(frame_valid),
            "left": self._side_packet("left", left),
            "right": self._side_packet("right", right),
        }
        data = (json.dumps(payload, separators=(",", ":")) + "\n").encode("ascii")
        if len(data) > MAX_LINE_BYTES:
            self.dropped += 1
            self.last_error = "line_overflow"
            return False
        try:
            self._pending.put_nowait(data)
            return True
        except queue.Full:
            try:
                self._pending.get_nowait()
            except queue.Empty:
                pass
            self.dropped += 1
            try:
                self._pending.put_nowait(data)
            except queue.Full:
                return False
            return True

    def close(self) -> None:
        try:
            self._pending.put_nowait(None)
        except queue.Full:
            pass
        self._worker.join(timeout=0.1)
        self._serial.close()
