from __future__ import annotations

import argparse
import sys
import time
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
from block_detected_for_pi.uart_publisher import create_uart_publisher


def emit_uart_frames(
    count: int,
    *,
    mock: bool,
    port: str,
    baud: int,
    loop: bool,
    interval_s: float,
) -> int:
    publisher = create_uart_publisher(port, baud, mock=mock)
    payload = pack_payload(128, 64, 1, 2)
    status = STATUS_PROTO_OK | STATUS_FRAME_VALID | STATUS_LEFT_FOUND | STATUS_RIGHT_FOUND
    heartbeat = 0
    try:
        while True:
            frame = build_registers(
                payload=payload,
                frame_valid=True,
                left_found=True,
                right_found=True,
                heartbeat=heartbeat & 0xFF,
            )
            publisher.publish(frame)
            line = format_frame(heartbeat=heartbeat & 0xFF, status=status, payload=payload)
            parsed = parse_frame(line)
            print(f"TX heartbeat={parsed[0]} status=0x{parsed[1]:02X} payload=0x{parsed[2]:05X}")
            heartbeat = (heartbeat + 1) & 0xFF
            if not loop and heartbeat >= count:
                break
            time.sleep(interval_s)
    except KeyboardInterrupt:
        print("stopped")
    finally:
        publisher.close()
    return 0


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


def listen_uart(port: str, baud: int, seconds: float) -> int:
    import serial  # type: ignore[import-untyped]

    with serial.Serial(port=port, baudrate=baud, timeout=0.2) as device:
        end = time.time() + seconds
        buffer = ""
        while time.time() < end:
            chunk = device.read(256)
            if not chunk:
                continue
            buffer += chunk.decode("ascii", errors="replace")
            while "\n" in buffer:
                line, buffer = buffer.split("\n", 1)
                if not line.strip():
                    continue
                heartbeat, status, payload, _ = parse_frame(line + "\n")
                print(f"RX heartbeat={heartbeat} status=0x{status:02X} payload=0x{payload:05X}")
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Pi5 transport bench utilities")
    parser.add_argument("--mode", choices=("uart", "uart-listen", "frame-layout"), default="uart")
    parser.add_argument("--count", type=int, default=10, help="frames to send (ignored with --loop)")
    parser.add_argument("--loop", action="store_true", help="send frames until Ctrl+C")
    parser.add_argument("--interval", type=float, default=0.05, help="seconds between frames")
    parser.add_argument("--port", default="/dev/serial0")
    parser.add_argument("--baud", type=int, default=9600)
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--mock", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.mode == "uart":
        if args.count < 0:
            print("--count must be >= 0", file=sys.stderr)
            return 2
        if args.interval <= 0:
            print("--interval must be > 0", file=sys.stderr)
            return 2
        return emit_uart_frames(
            args.count,
            mock=args.mock,
            port=args.port,
            baud=args.baud,
            loop=args.loop,
            interval_s=args.interval,
        )
    if args.mode == "uart-listen":
        return listen_uart(args.port, args.baud, args.seconds)
    if args.mode == "frame-layout":
        return verify_frame_layout()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
