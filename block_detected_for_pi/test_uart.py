from __future__ import annotations

import unittest

from block_detected_for_pi.payload import (
    STATUS_FRAME_VALID,
    STATUS_LEFT_FOUND,
    STATUS_PROTO_OK,
    STATUS_RIGHT_FOUND,
    build_registers,
    pack_payload,
)
from block_detected_for_pi.uart_protocol import format_frame, parse_frame
from block_detected_for_pi.uart_publisher import MockUartPort, UartPublisher, frame_to_uart_line, registers_to_uart_line


class UartProtocolTests(unittest.TestCase):
    def test_round_trip_frame(self) -> None:
        payload = pack_payload(128, 64, 1, 2)
        status = STATUS_PROTO_OK | STATUS_FRAME_VALID | STATUS_LEFT_FOUND | STATUS_RIGHT_FOUND
        line = format_frame(heartbeat=7, status=status, payload=payload)
        heartbeat, parsed_status, parsed_payload, _ = parse_frame(line)
        self.assertEqual(heartbeat, 7)
        self.assertEqual(parsed_status, status)
        self.assertEqual(parsed_payload, payload)

    def test_crc_rejects_tampered_frame(self) -> None:
        line = format_frame(heartbeat=1, status=STATUS_PROTO_OK, payload=0)
        bad = line.replace("00", "01", 1)
        with self.assertRaises(ValueError):
            parse_frame(bad)

    def test_registers_to_uart_line_matches_register_bytes(self) -> None:
        payload = pack_payload(10, 20, 1, 2)
        frame = build_registers(
            payload=payload,
            frame_valid=True,
            left_found=True,
            right_found=True,
            heartbeat=3,
        )
        self.assertEqual(frame_to_uart_line(frame), registers_to_uart_line(
            payload=payload,
            frame_valid=True,
            left_found=True,
            right_found=True,
            heartbeat=3,
        ))

    def test_publisher_writes_line(self) -> None:
        port = MockUartPort()
        publisher = UartPublisher(port)
        frame = build_registers(
            payload=pack_payload(1, 2, 0, 1),
            frame_valid=True,
            left_found=True,
            right_found=True,
            heartbeat=0,
        )
        publisher.publish(frame)
        self.assertEqual(len(port.frames), 1)
        heartbeat, status, payload, _ = parse_frame(port.frames[0])
        self.assertEqual(heartbeat, 0)
        self.assertTrue(status & STATUS_FRAME_VALID)


if __name__ == "__main__":
    unittest.main()
