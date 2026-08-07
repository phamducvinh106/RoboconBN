#!/usr/bin/env python3
"""Cross-check Pi5 UART frames between Python publisher and Java codec logic."""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from block_detected_for_pi.payload import (
    STATUS_FRAME_VALID,
    STATUS_LEFT_FOUND,
    STATUS_PROTO_OK,
    STATUS_RIGHT_FOUND,
    build_registers,
    pack_payload,
)
from block_detected_for_pi.uart_protocol import format_frame, parse_frame
from block_detected_for_pi.uart_publisher import frame_to_uart_line


def main() -> int:
    payload = pack_payload(128, 64, 1, 2)
    status = STATUS_PROTO_OK | STATUS_FRAME_VALID | STATUS_LEFT_FOUND | STATUS_RIGHT_FOUND
    frame = build_registers(
        payload=payload,
        frame_valid=True,
        left_found=True,
        right_found=True,
        heartbeat=2,
    )
    line = frame_to_uart_line(frame)
    parsed = parse_frame(line)
    if parsed[2] != payload:
        print("Python round-trip failed", file=sys.stderr)
        return 1

    bench = subprocess.run(
        [sys.executable, str(ROOT / "tools" / "pi5_bench.py"), "--mode", "frame-layout"],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
    )
    if bench.returncode != 0:
        print(bench.stdout)
        print(bench.stderr, file=sys.stderr)
        return bench.returncode

    print("E2E protocol check OK")
    print(f"sample frame: {line.strip()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
