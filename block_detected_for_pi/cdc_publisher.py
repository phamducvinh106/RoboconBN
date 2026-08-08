from __future__ import annotations

import json
import queue
import threading
import time
from typing import Any


class CdcPublisher:
    """Publish newline-delimited JSON through Pi USB CDC gadget."""

    def __init__(self, device: str = "/dev/ttyGS0") -> None:
        import serial

        self._serial_timeout = serial.SerialTimeoutException
        self._serial = serial.Serial(device, 115200, timeout=0, write_timeout=0.01)
        self._pending: queue.Queue[bytes | None] = queue.Queue(maxsize=1)
        self._worker = threading.Thread(target=self._write_loop, name="cdc-writer", daemon=True)
        self.dropped = 0
        self.written = 0
        self.last_error = ""
        self._worker.start()

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

    def publish(self, *, left: Any, right: Any, frame_valid: bool) -> bool:
        timestamp_ms = int(time.time() * 1000)

        def packet(camera: str, target: Any) -> dict[str, object]:
            return {
                "v": 1,
                "camera": camera,
                "found": bool(target.found),
                "block_type": int(target.class_id) if target.found else -1,
                "class_name": target.class_name if target.found else "",
                "confidence": float(target.confidence) if target.found else 0.0,
                "x": float(target.x) if target.found else 0.0,
                "y": float(target.y) if target.found else 0.0,
                "ts_ms": timestamp_ms,
            }

        payload = {
            "v": 1,
            "frame_valid": frame_valid,
            "left": packet("left", left),
            "right": packet("right", right),
        }
        data = (json.dumps(payload, separators=(",", ":")) + "\n").encode("ascii")
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
