from __future__ import annotations

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from block_detected_for_pi.payload import (
    build_registers,
    pack_payload,
)


def verify_frame_layout() -> int:
    payload = pack_payload(100, 50, 2, 3)
    frame = build_registers(
        payload=payload,
        frame_valid=True,
        left_found=True,
        right_found=True,
        heartbeat=0,
    )
    assert len(frame) == 6
    assert frame[5] == 1
    print("Logical frame layout OK:", frame.hex())
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Pi5 transport bench utilities")
    parser.add_argument("--mode", choices=("frame-layout",), default="frame-layout")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.mode == "frame-layout":
        return verify_frame_layout()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
