from __future__ import annotations

import unittest

from block_detected_for_pi.config import BlockCodeConfig
from block_detected_for_pi.types import Target
from block_detected_for_pi.payload import (
    build_registers,
    frame_from_targets,
    pack_payload,
    unpack_payload,
)


class PayloadTests(unittest.TestCase):
    def test_round_trip(self) -> None:
        payload = pack_payload(128, 64, 2, 1)
        decoded = unpack_payload(payload)
        self.assertEqual(decoded.x, 128)
        self.assertEqual(decoded.y, 64)
        self.assertEqual(decoded.left_code, 2)
        self.assertEqual(decoded.right_code, 1)

    def test_masks_match_spec(self) -> None:
        payload = pack_payload(0xAB, 0xCD, 0x3, 0x2)
        decoded = unpack_payload(payload)
        self.assertEqual(decoded.x, 0xAB)
        self.assertEqual(decoded.y, 0xCD)
        self.assertEqual(decoded.left_code, 3)
        self.assertEqual(decoded.right_code, 2)

    def test_registers_length(self) -> None:
        frame = build_registers(
            payload=pack_payload(10, 20, 1, 0),
            frame_valid=True,
            left_found=True,
            right_found=True,
            heartbeat=7,
        )
        self.assertEqual(len(frame), 6)
        self.assertEqual(frame[4], 7)
        self.assertEqual(frame[5], 1)
        self.assertTrue(frame[0] & 0x80)

    def test_invalid_frame_not_valid(self) -> None:
        left = Target(found=False)
        right = Target(found=True, class_id=0, class_name="block", confidence=0.9, x=0.5, y=0.5)
        payload, frame_valid, left_found, right_found = frame_from_targets(
            left,
            right,
            left_code=None,
            right_code=0,
            min_confidence=0.25,
        )
        self.assertFalse(frame_valid)
        self.assertFalse(left_found)
        self.assertTrue(right_found)
        self.assertEqual(payload, 0)


class ConfigTests(unittest.TestCase):
    def test_default_class_mapping(self) -> None:
        config = BlockCodeConfig()
        self.assertEqual(config.code_for(0, "block"), 0)
        self.assertEqual(config.block_type_for_code(2), "03")


if __name__ == "__main__":
    unittest.main()
