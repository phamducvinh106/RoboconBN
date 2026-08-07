from __future__ import annotations

import unittest

from block_detected_for_pi.i2c_slave import RegisterFile
from block_detected_for_pi.payload import build_registers, pack_payload


class I2cRegisterTests(unittest.TestCase):
    def test_publish_valid_frame_increments_heartbeat(self) -> None:
        register_file = RegisterFile()
        frame = build_registers(
            payload=pack_payload(10, 20, 1, 2),
            frame_valid=True,
            left_found=True,
            right_found=True,
            heartbeat=register_file.heartbeat,
        )
        first = register_file.publish(frame)
        second = register_file.publish(frame)
        self.assertEqual(first, 1)
        self.assertEqual(second, 2)

    def test_invalid_frame_does_not_increment_heartbeat(self) -> None:
        register_file = RegisterFile()
        frame = build_registers(
            payload=0,
            frame_valid=False,
            left_found=False,
            right_found=False,
            heartbeat=register_file.heartbeat,
        )
        heartbeat = register_file.publish(frame)
        self.assertEqual(heartbeat, 0)

    def test_snapshot_is_six_bytes(self) -> None:
        register_file = RegisterFile()
        frame = build_registers(
            payload=pack_payload(1, 2, 0, 3),
            frame_valid=True,
            left_found=True,
            right_found=True,
            heartbeat=register_file.heartbeat,
        )
        register_file.publish(frame)
        self.assertEqual(len(register_file.snapshot()), 6)


if __name__ == "__main__":
    unittest.main()
