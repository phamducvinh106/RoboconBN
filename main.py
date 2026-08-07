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
from block_detected_for_pi.types import Target
from cli.monitor import CliMonitor
from cli.state import CameraState, DetectionState, MonitorState, UsbCdcState

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
    parser.add_argument("--verbose", action="store_true",
                        help="show dashboard plus latest raw frame JSON")
    output_mode = parser.add_mutually_exclusive_group()
    output_mode.add_argument("--json", action="store_true",
                             help="machine-readable NDJSON per frame on stdout; disables dashboard")
    output_mode.add_argument("--no-ui", action="store_true",
                             help="headless periodic status JSON on stderr; disables dashboard")
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


def detection_state(target: Target, *, age_ms: float) -> DetectionState:
    return DetectionState(
        found=target.found,
        class_name=target.class_name if target.found else "",
        block_type=int(target.class_id) if target.found else -1,
        confidence=float(target.confidence) if target.found else 0.0,
        x=float(target.x) if target.found else 0.0,
        y=float(target.y) if target.found else 0.0,
        age_ms=age_ms,
    )


def build_frame_record(
    *,
    frame_valid: bool,
    left: Target,
    right: Target,
    payload: int,
    heartbeat: int,
    no_cdc: bool,
) -> dict[str, object]:
    decoded = unpack_payload(payload)
    return {
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
        "cdc": not no_cdc,
    }


def build_monitor_state(
    *,
    start_time: float,
    frame: int,
    pair_fps: float,
    model_path: Path,
    threads: int,
    inference_ms: float,
    cameras: tuple[dict[str, object], dict[str, object]],
    left_capture_ms: float,
    right_capture_ms: float,
    left: Target,
    right: Target,
    cdc: CdcPublisher | None,
    cdc_device: str,
    last_error: str,
    age_ms: float,
    raw_verbose: str | None,
) -> MonitorState:
    gadget_state, usb_speed = usb_gadget_state()
    cdc_written = None if cdc is None else cdc.written
    cdc_dropped = None if cdc is None else cdc.dropped
    connected = (
        cdc is not None
        and gadget_state in {"configured", "suspended"}
        and bool(cdc_written)
    )
    camera_states = (
        CameraState(
            label="LEFT",
            path=str(cameras[0]["path"]),
            opened=bool(cameras[0]["opened"]),
            width=int(cameras[0].get("width", 0) or 0),
            height=int(cameras[0].get("height", 0) or 0),
            capture_ms=left_capture_ms,
        ),
        CameraState(
            label="RIGHT",
            path=str(cameras[1]["path"]),
            opened=bool(cameras[1]["opened"]),
            width=int(cameras[1].get("width", 0) or 0),
            height=int(cameras[1].get("height", 0) or 0),
            capture_ms=right_capture_ms,
        ),
    )
    return MonitorState(
        runtime_s=time.monotonic() - start_time,
        fps=float(pair_fps),
        frame=frame,
        model=model_path.name,
        threads=threads,
        inference_ms=inference_ms,
        cameras=camera_states,
        left=detection_state(left, age_ms=age_ms),
        right=detection_state(right, age_ms=age_ms),
        usb=UsbCdcState(
            enabled=cdc is not None,
            device=cdc_device,
            connected=connected,
            usb_state=gadget_state,
            usb_speed=usb_speed,
            written=cdc_written,
            dropped=cdc_dropped,
            last_error="" if cdc is None else cdc.last_error,
        ),
        last_error=last_error,
        raw_verbose=raw_verbose,
    )


def run(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    if (args.frames < 0 or args.size < 32 or not 0 <= args.conf <= 1 or args.threads < 1
            or args.camera_width < 1 or args.camera_height < 1 or args.camera_fps <= 0):
        parser.error("frames >= 0, size >= 32, conf in [0, 1], threads/camera dimensions/fps > 0")

    use_dashboard = not args.json and not args.no_ui
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

    startup = {"model": str(model_path), "threads": args.threads, "cameras": cameras}
    if args.no_ui:
        print(json.dumps(startup), file=sys.stderr)

    monitor: CliMonitor | None = None
    if use_dashboard:
        monitor = CliMonitor(verbose=args.verbose)
        monitor.start()

    start_time = time.monotonic()
    last_error = ""
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
            pair_fps = 0.0
            while args.frames == 0 or count < args.frames:
                capture_start = time.perf_counter()
                ok_left, left_frame = left_cam.read()
                left_ms = (time.perf_counter() - capture_start) * 1000
                right_start = time.perf_counter()
                ok_right, right_frame = right_cam.read()
                right_ms = (time.perf_counter() - right_start) * 1000
                if not ok_left or left_frame is None or not ok_right or right_frame is None:
                    last_error = "camera read failed"
                    print(json.dumps({"error": last_error}), file=sys.stderr)
                    return 1

                inference_start = time.perf_counter()
                left, right = core.process_batch((left_frame, right_frame))
                inference_ms = (time.perf_counter() - inference_start) * 1000
                inference_end = time.perf_counter()
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
                    pair_fps = round(metric_frames / metric_elapsed, 1) if metric_elapsed > 0 else 0.0
                    if args.no_ui:
                        gadget_state, usb_speed = usb_gadget_state()
                        cdc_written = None if cdc is None else cdc.written
                        cdc_dropped = None if cdc is None else cdc.dropped
                        connected = gadget_state in {"configured", "suspended"} and bool(cdc_written)
                        status = {
                            "running": True,
                            "pair_fps": pair_fps,
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

                age_ms = max(0.0, (time.perf_counter() - inference_end) * 1000)
                record = build_frame_record(
                    frame_valid=frame_valid,
                    left=left,
                    right=right,
                    payload=payload,
                    heartbeat=heartbeat,
                    no_cdc=args.no_cdc,
                )
                raw_verbose = json.dumps(record, separators=(",", ":")) if args.verbose else None

                if args.json:
                    print(json.dumps(record, separators=(",", ":")))

                if monitor is not None:
                    snapshot = build_monitor_state(
                        start_time=start_time,
                        frame=count + 1,
                        pair_fps=pair_fps,
                        model_path=model_path,
                        threads=args.threads,
                        inference_ms=inference_ms,
                        cameras=cameras,
                        left_capture_ms=left_ms,
                        right_capture_ms=right_ms,
                        left=left,
                        right=right,
                        cdc=cdc,
                        cdc_device=args.cdc_device,
                        last_error=last_error,
                        age_ms=age_ms,
                        raw_verbose=raw_verbose,
                    )
                    monitor.update(snapshot)

                count += 1
    finally:
        if monitor is not None:
            monitor.close()
        left_cam.release()
        right_cam.release()
        if cdc is not None:
            cdc.close()
    return 0


def main() -> int:
    return run()


if __name__ == "__main__":
    raise SystemExit(main())
