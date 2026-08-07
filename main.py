from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2

from block_detected_for_pi.config import BlockCodeConfig
from block_detected_for_pi.cdc_publisher import CdcPublisher
from block_detected_for_pi.core import TargetingCore
from block_detected_for_pi.frame_state import RegisterFile
from block_detected_for_pi.payload import build_registers, frame_from_targets, unpack_payload

DEFAULT_MODEL = "block_detected_for_pi/models/pose11-fp16.onnx"
DEFAULT_CDC_DEVICE = "/dev/ttyGS0"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Pi 5 dual-camera block detection with USB CDC output")
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--camera-left", type=int, default=0,
                        help="V4L2 index for left cam (often /dev/video0)")
    parser.add_argument("--camera-right", type=int, default=2,
                        help="V4L2 index for right cam (often /dev/video2; not video1 metadata)")
    parser.add_argument("--size", type=int, default=320)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--no-cdc", action="store_true", help="skip USB CDC publish")
    parser.add_argument("--cdc-device", default=DEFAULT_CDC_DEVICE)
    return parser


def camera_details(index: int, camera: cv2.VideoCapture) -> dict[str, object]:
    opened = camera.isOpened()
    details: dict[str, object] = {"index": index, "path": f"/dev/video{index}", "opened": opened}
    if opened:
        details.update({
            "backend": camera.getBackendName(),
            "width": int(camera.get(cv2.CAP_PROP_FRAME_WIDTH)),
            "height": int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            "fps": camera.get(cv2.CAP_PROP_FPS),
        })
    return details


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
    cdc = None
    if not args.no_cdc:
        cdc = CdcPublisher(args.cdc_device)

    left_cam = cv2.VideoCapture(args.camera_left)
    right_cam = cv2.VideoCapture(args.camera_right)
    cameras = (
        camera_details(args.camera_left, left_cam),
        camera_details(args.camera_right, right_cam),
    )
    unavailable = [camera["index"] for camera in cameras if not camera["opened"]]
    if unavailable:
        left_cam.release()
        right_cam.release()
        print(json.dumps({
            "error": "camera open failed",
            "unavailable": unavailable,
            "cameras": cameras,
            "hint": "dual-camera mode needs two V4L2 Video Capture devices; metadata nodes are not cameras",
        }), file=sys.stderr)
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
                if cdc is not None:
                    cdc.publish(left=left, right=right, frame_valid=frame_valid)
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
                    "cdc": not args.no_cdc,
                }
                print(json.dumps(record, separators=(",", ":")), flush=True)
                count += 1
    finally:
        left_cam.release()
        right_cam.release()
        if cdc is not None:
            cdc.close()
    return 0


def main() -> int:
    return run()


if __name__ == "__main__":
    raise SystemExit(main())
