from __future__ import annotations

import json
import time
from typing import Any


class CdcPublisher:
    """Publish newline-delimited JSON through Pi USB CDC gadget."""

    def __init__(self, device: str = "/dev/ttyGS0") -> None:
        import serial
        self._serial = serial.Serial(device, 115200, timeout=0)

    def publish(self, *, left: Any, right: Any, frame_valid: bool) -> None:
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
                "ts_ms": int(time.time() * 1000),
            }

        payload = {"v": 1, "frame_valid": frame_valid,
                   "left": packet("left", left), "right": packet("right", right)}
        self._serial.write((json.dumps(payload, separators=(",", ":")) + "\n").encode("ascii"))
        self._serial.flush()

    def close(self) -> None:
        self._serial.close()
