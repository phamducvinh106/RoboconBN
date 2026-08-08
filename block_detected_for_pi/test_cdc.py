"""CDC JSON packet contract — must match PiCdcPacket on Hub."""

from __future__ import annotations

import json
import math
import queue
import threading
import time
import unittest
from types import SimpleNamespace

from block_detected_for_pi.cdc_publisher import CdcPublisher, MAX_LINE_BYTES, PROTOCOL_VERSION


class _MockSerial:
    def __init__(self) -> None:
        self.lines: list[str] = []

    def write(self, data: bytes) -> int:
        self.lines.append(data.decode("ascii"))
        return len(data)

    def close(self) -> None:
        return


def _drain(publisher: CdcPublisher, port: _MockSerial, timeout_s: float = 0.5) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        if publisher._pending.empty() and port.lines:  # type: ignore[attr-defined]
            return
        time.sleep(0.01)


def _publisher(port: _MockSerial) -> CdcPublisher:
    publisher = CdcPublisher.__new__(CdcPublisher)
    publisher._serial_timeout = Exception  # type: ignore[attr-defined]
    publisher._serial = port  # type: ignore[attr-defined]
    publisher._pending = queue.Queue(maxsize=1)  # type: ignore[attr-defined]
    publisher._heartbeat = 0  # type: ignore[attr-defined]
    publisher.dropped = 0  # type: ignore[attr-defined]
    publisher.written = 0  # type: ignore[attr-defined]
    publisher.last_error = ""  # type: ignore[attr-defined]
    publisher._worker = threading.Thread(target=publisher._write_loop, daemon=True)  # type: ignore[attr-defined]
    publisher._worker.start()  # type: ignore[attr-defined]
    return publisher


def valid_packet(
    *,
    hb: int = 1,
    frame_valid: bool = True,
    left_found: bool = True,
    right_found: bool = True,
) -> dict[str, object]:
    return {
        "v": PROTOCOL_VERSION,
        "hb": hb,
        "frame_valid": frame_valid,
        "left": {
            "camera": "left",
            "found": left_found,
            "block_type": 1 if left_found else -1,
            "class_name": "02" if left_found else "",
            "confidence": 0.9 if left_found else 0.0,
            "x": 0.4 if left_found else 0.0,
            "y": 0.5 if left_found else 0.0,
        },
        "right": {
            "camera": "right",
            "found": right_found,
            "block_type": 3 if right_found else -1,
            "class_name": "04" if right_found else "",
            "confidence": 0.8 if right_found else 0.0,
            "x": 0.6 if right_found else 0.0,
            "y": 0.5 if right_found else 0.0,
        },
    }


class CdcPublisherTest(unittest.TestCase):
    def test_dual_camera_packet_shape(self) -> None:
        port = _MockSerial()
        publisher = _publisher(port)
        left = SimpleNamespace(found=True, class_id=1, class_name="02", confidence=0.9, x=0.4, y=0.5)
        right = SimpleNamespace(found=True, class_id=3, class_name="04", confidence=0.8, x=0.6, y=0.5)
        self.assertTrue(publisher.publish(left=left, right=right, frame_valid=True))
        _drain(publisher, port)
        publisher.close()
        packet = json.loads(port.lines[0].strip())
        self.assertEqual(packet["v"], PROTOCOL_VERSION)
        self.assertEqual(packet["hb"], 1)
        self.assertTrue(packet["frame_valid"])
        self.assertEqual(packet["left"]["camera"], "left")
        self.assertEqual(packet["right"]["camera"], "right")

    def test_heartbeat_increments(self) -> None:
        port = _MockSerial()
        publisher = _publisher(port)
        target = SimpleNamespace(found=False, class_id=-1, class_name="", confidence=0.0, x=0.0, y=0.0)
        publisher.publish(left=target, right=target, frame_valid=False)
        publisher.publish(left=target, right=target, frame_valid=False)
        self.assertEqual(publisher.heartbeat, 2)
        _drain(publisher, port)
        publisher.close()
        self.assertGreaterEqual(len(port.lines), 1)
        self.assertEqual(json.loads(port.lines[-1].strip())["hb"], 2)

    def test_latest_frame_drop(self) -> None:
        port = _MockSerial()
        publisher = _publisher(port)
        target = SimpleNamespace(found=False, class_id=-1, class_name="", confidence=0.0, x=0.0, y=0.0)
        for _ in range(3):
            publisher.publish(left=target, right=target, frame_valid=True)
        self.assertEqual(publisher.heartbeat, 3)
        _drain(publisher, port)
        publisher.close()
        self.assertGreaterEqual(len(port.lines), 1)
        self.assertEqual(json.loads(port.lines[-1].strip())["hb"], 3)
        self.assertGreaterEqual(publisher.dropped, 1)

    def test_rejects_non_finite_input(self) -> None:
        port = _MockSerial()
        publisher = _publisher(port)
        bad = SimpleNamespace(found=True, class_id=1, class_name="02", confidence=float("nan"), x=0.4, y=0.5)
        good = SimpleNamespace(found=False, class_id=-1, class_name="", confidence=0.0, x=0.0, y=0.0)
        self.assertTrue(publisher.publish(left=bad, right=good, frame_valid=True))
        _drain(publisher, port)
        publisher.close()
        packet = json.loads(port.lines[0].strip())
        self.assertFalse(packet["left"]["found"])

    def test_line_length_bound(self) -> None:
        self.assertGreater(MAX_LINE_BYTES, 0)


if __name__ == "__main__":
    unittest.main()
