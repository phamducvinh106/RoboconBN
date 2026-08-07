from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2

from block_detected_for_pi.config import BlockCodeConfig
from block_detected_for_pi.cdc_publisher import CdcPublisher
from block_detected_for_pi.core import TargetingCore
from block_detected_for_pi.frame_state import RegisterFile
from block_detected_for_pi.payload import build_registers, frame_from_targets, unpack_payload

DEFAULT_MODEL = "block_detected_for_pi/models/pose11-fp16.onnx"
DEFAULT_CDC_DEVICE = "/dev/ttyGS0"


def usb_gadget_state() -> tuple[str, str]:
    states = Path("/sys/class/udc").glob("*/state")
    state_path = next(states, None)
    if state_path is None:
        return "missing", "UNKNOWN"
    state = state_path.read_text(encoding="ascii").strip()
    speed_path = state_path.with_name("current_speed")
    speed = speed_path.read_text(encoding="ascii").strip() if speed_path.is_file() else "UNKNOWN"
    return state, speed


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Pi 5 dual-camera block detection with USB CDC output")
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--camera-left", type=int, default=0,
                        help="V4L2 index for left cam (often /dev/video0)")
    parser.add_argument("--camera-right", type=int, default=2,
                        help="V4L2 index for right cam (often /dev/video2; not video1 metadata)")
    parser.add_argument("--size", type=int, default=320)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--threads", type=int, default=4, help="ONNX CPU threads")
    parser.add_argument("--camera-width", type=int, default=640)
    parser.add_argument("--camera-height", type=int, default=480)
    parser.add_argument("--camera-fps", type=float, default=30.0)
    parser.add_argument("--frames", type=int, default=0)
    parser.add_argument("--verbose", action="store_true", help="print every detection frame")
    parser.add_argument("--no-cdc", action="store_true", help="skip USB CDC publish")
    parser.add_argument("--cdc-device", default=DEFAULT_CDC_DEVICE)
    return parser


def configure_camera(camera: cv2.VideoCapture, *, width: int, height: int, fps: float) -> None:
    camera.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    camera.set(cv2.CAP_PROP_FPS, fps)
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 1)


def camera_details(index: int, camera: cv2.VideoCapture) -> dict[str, object]:
    opened = camera.isOpened()
    details: dict[str, object] = {"index": index, "path": f"/dev/video{index}", "opened": opened}
    if opened:
        fourcc = int(camera.get(cv2.CAP_PROP_FOURCC))
        details.update({
            "backend": camera.getBackendName(),
            "width": int(camera.get(cv2.CAP_PROP_FRAME_WIDTH)),
            "height": int(camera.get(cv2.CAP_PROP_FRAME_HEIGHT)),
            "fps": camera.get(cv2.CAP_PROP_FPS),
            "fourcc": "".join(chr((fourcc >> (8 * offset)) & 0xFF) for offset in range(4)),
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
    if (args.frames < 0 or args.size < 32 or not 0 <= args.conf <= 1 or args.threads < 1
            or args.camera_width < 1 or args.camera_height < 1 or args.camera_fps <= 0):
        parser.error("frames >= 0, size >= 32, conf in [0, 1], threads/camera dimensions/fps > 0")

    model_path = resolve_model(args.model)
    code_config = BlockCodeConfig()
    register_file = RegisterFile()
    cdc = None
    if not args.no_cdc:
        cdc = CdcPublisher(args.cdc_device)

    left_cam = cv2.VideoCapture(args.camera_left, cv2.CAP_V4L2)
    right_cam = cv2.VideoCapture(args.camera_right, cv2.CAP_V4L2)
    for camera in (left_cam, right_cam):
        configure_camera(
            camera,
            width=args.camera_width,
            height=args.camera_height,
            fps=args.camera_fps,
        )
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

    print(json.dumps({"model": str(model_path), "threads": args.threads, "cameras": cameras}), file=sys.stderr)
    try:
        with TargetingCore(
            model_path,
            image_size=args.size,
            confidence=args.conf,
            threads=args.threads,
        ) as core:
            count = 0
            last_metric = time.monotonic()
            metric_frames = 0
            while args.frames == 0 or count < args.frames:
                capture_start = time.perf_counter()
                ok_left, left_frame = left_cam.read()
                left_ms = (time.perf_counter() - capture_start) * 1000
                right_start = time.perf_counter()
                ok_right, right_frame = right_cam.read()
                right_ms = (time.perf_counter() - right_start) * 1000
                if not ok_left or left_frame is None or not ok_right or right_frame is None:
                    print(json.dumps({"error": "camera read failed"}), file=sys.stderr)
                    return 1

                inference_start = time.perf_counter()
                left, right = core.process_batch((left_frame, right_frame))
                inference_ms = (time.perf_counter() - inference_start) * 1000
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
                metric_frames += 1
                now = time.monotonic()
                if count == 0 or now - last_metric >= 2.0:
                    metric_elapsed = now - last_metric
                    gadget_state, usb_speed = usb_gadget_state()
                    cdc_written = None if cdc is None else cdc.written
                    cdc_dropped = None if cdc is None else cdc.dropped
                    connected = gadget_state in {"configured", "suspended"} and bool(cdc_written)
                    status = {
                        "running": True,
                        "pair_fps": round(metric_frames / metric_elapsed, 1),
                        "frame": count + 1,
                        "left_capture_ms": round(left_ms, 1),
                        "right_capture_ms": round(right_ms, 1),
                        "inference_ms": round(inference_ms, 1),
                        "usb_state": gadget_state,
                        "usb_speed": usb_speed,
                        "cdc_connected": connected,
                        "cdc_written": cdc_written,
                        "cdc_dropped": cdc_dropped,
                        "cdc_error": None if cdc is None else cdc.last_error,
                        "left_found": left.found,
                        "right_found": right.found,
                    }
                    print(json.dumps(status, separators=(",", ":")), file=sys.stderr, flush=True)
                    last_metric = now
                    metric_frames = 0
                if args.verbose:
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
                    print(json.dumps(record, separators=(",", ":")))
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
