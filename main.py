from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2

from block_detected_for_pi.config import BlockCodeConfig, DEFAULT_UART_BAUD
from block_detected_for_pi.core import TargetingCore
from block_detected_for_pi.frame_state import RegisterFile
from block_detected_for_pi.payload import build_registers, frame_from_targets, unpack_payload
from block_detected_for_pi.uart_publisher import create_uart_publisher


DEFAULT_MODEL = "block_detected_for_pi/models/pose11-fp16.onnx"
DEFAULT_UART_PORT = "/dev/serial0"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Pi 5 dual-camera block detection with UART output")
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--camera-left", type=int, default=0,
                        help="V4L2 index for left cam (often /dev/video0)")
    parser.add_argument("--camera-right", type=int, default=2,
                        help="V4L2 index for right cam (often /dev/video2; not video1 metadata)")
    parser.add_argument("--size", type=int, default=320)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--no-uart", action="store_true", help="skip UART publish (stdout JSON only)")
    parser.add_argument("--uart-port", default=DEFAULT_UART_PORT)
    parser.add_argument("--uart-baud", type=int, default=DEFAULT_UART_BAUD)
    parser.add_argument("--mock-uart", action="store_true", help="skip real UART port open")
    return parser


def resolve_model(path: str) -> Path:
    model_path = Path(path).expanduser()
    if model_path.is_file():
        return model_path
    repo_root = Path(__file__).resolve().parent
    fallback = repo_root / path
    if fallback.is_file():
        return fallback
    legacy = repo_root / "block_detected_for_pi" / "models" / "pose11-fp16.onnx"
    if legacy.is_file():
        return legacy
    raise FileNotFoundError(f"model not found: {path}")


def run(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if args.frames < 0 or args.size < 32 or not 0 <= args.conf <= 1:
        parser.error("frames >= 0, size >= 32, conf in [0, 1]")

    model_path = resolve_model(args.model)
    code_config = BlockCodeConfig()
    register_file = RegisterFile()
    uart = None
    if not args.no_uart:
        uart = create_uart_publisher(args.uart_port, args.uart_baud, mock=args.mock_uart)

    left_cam = cv2.VideoCapture(args.camera_left)
    right_cam = cv2.VideoCapture(args.camera_right)
    if not left_cam.isOpened() or not right_cam.isOpened():
        print(json.dumps({"error": "camera open failed"}), file=sys.stderr)
        return 1

    try:
        with TargetingCore(model_path, image_size=args.size, confidence=args.conf) as core:
            count = 0
            while args.frames == 0 or count < args.frames:
                ok_left, left_frame = left_cam.read()
                ok_right, right_frame = right_cam.read()
                if not ok_left or left_frame is None or not ok_right or right_frame is None:
                    print(json.dumps({"error": "camera read failed"}), file=sys.stderr)
                    return 1

                left = core.process(left_frame)
                right = core.process(right_frame)
                left_code = code_config.code_for(left.class_id, left.class_name) if left.found else None
                right_code = code_config.code_for(right.class_id, right.class_name) if right.found else None
                payload, frame_valid, left_found, right_found = frame_from_targets(
                    left,
                    right,
                    left_code=left_code,
                    right_code=right_code,
                    min_confidence=args.conf,
                )
                frame = build_registers(
                    payload=payload,
                    frame_valid=frame_valid,
                    left_found=left_found,
                    right_found=right_found,
                    heartbeat=register_file.heartbeat,
                )
                heartbeat = register_file.publish(frame)
                if uart is not None:
                    uart.publish(frame)
                decoded = unpack_payload(payload)
                record = {
                    "frame_valid": frame_valid,
                    "left": left.packet(),
                    "right": right.packet(),
                    "payload": payload,
                    "decoded": {
                        "x": decoded.x,
                        "y": decoded.y,
                        "left_code": decoded.left_code,
                        "right_code": decoded.right_code,
                    },
                    "heartbeat": heartbeat,
                    "uart": not args.no_uart,
                }
                print(json.dumps(record, separators=(",", ":")), flush=True)
                count += 1
    finally:
        left_cam.release()
        right_cam.release()
        if uart is not None:
            uart.close()
    return 0


def main() -> int:
    return run()


if __name__ == "__main__":
    raise SystemExit(main())
