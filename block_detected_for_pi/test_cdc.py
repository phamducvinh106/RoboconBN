"""CDC JSON packet shape — must match PiBlockReceiver on Hub."""

from __future__ import annotations

import json
import queue
import threading
import unittest
from types import SimpleNamespace

from block_detected_for_pi.cdc_publisher import CdcPublisher


class _MockSerial:
    def __init__(self) -> None:
        self.lines: list[str] = []

    def write(self, data: bytes) -> int:
        self.lines.append(data.decode("ascii"))
        return len(data)

    def close(self) -> None:
        return


class CdcPublisherTest(unittest.TestCase):
    def test_dual_camera_packet_shape(self) -> None:
        port = _MockSerial()
        publisher = CdcPublisher.__new__(CdcPublisher)
        publisher._serial_timeout = Exception  # type: ignore[attr-defined]
        publisher._serial = port  # type: ignore[attr-defined]
        publisher._pending = queue.Queue(maxsize=1)  # type: ignore[attr-defined]
        publisher.dropped = 0  # type: ignore[attr-defined]
        publisher.written = 0  # type: ignore[attr-defined]
        publisher.last_error = ""  # type: ignore[attr-defined]
        publisher._worker = threading.Thread(target=publisher._write_loop, daemon=True)  # type: ignore[attr-defined]
        publisher._worker.start()  # type: ignore[attr-defined]

        left = SimpleNamespace(
            found=True, class_id=1, class_name="02", confidence=0.9, x=0.4, y=0.5
        )
        right = SimpleNamespace(
            found=True, class_id=3, class_name="04", confidence=0.8, x=0.6, y=0.5
        )
        self.assertTrue(publisher.publish(left=left, right=right, frame_valid=True))
        publisher.close()

        self.assertEqual(len(port.lines), 1)
        packet = json.loads(port.lines[0].strip())
        self.assertEqual(packet["v"], 1)
        self.assertTrue(packet["frame_valid"])
        self.assertEqual(packet["left"]["camera"], "left")
        self.assertEqual(packet["right"]["camera"], "right")
        self.assertEqual(packet["left"]["block_type"], 1)
        self.assertEqual(packet["right"]["block_type"], 3)
        self.assertIn("ts_ms", packet["left"])
        self.assertIn("ts_ms", packet["right"])


if __name__ == "__main__":
    unittest.main()
