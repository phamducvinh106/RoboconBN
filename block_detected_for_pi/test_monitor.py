from __future__ import annotations

import io
import json
import unittest
from dataclasses import replace

from rich.console import Console

from cli.monitor import render_dashboard
from cli.state import (
    CAPTURE_WARN_MS,
    CameraState,
    DetectionState,
    MonitorState,
    UsbCdcState,
    format_runtime,
)
from main import build_parser, build_frame_record
from block_detected_for_pi.types import Target


def sample_state(**overrides: object) -> MonitorState:
    base = MonitorState(
        runtime_s=261.0,
        fps=7.6,
        frame=140,
        model="pose11-fp16.onnx",
        threads=4,
        inference_ms=52.1,
        cameras=(
            CameraState("LEFT", "/dev/video0", True, 640, 480, 1.5),
            CameraState("RIGHT", "/dev/video2", True, 640, 480, 79.2),
        ),
        left=DetectionState(False),
        right=DetectionState(False),
        usb=UsbCdcState(
            enabled=True,
            device="/dev/ttyGS0",
            connected=True,
            usb_state="configured",
            usb_speed="high-speed",
            written=140,
            dropped=0,
            last_error="",
        ),
        last_error="",
        raw_verbose=None,
    )
    if not overrides:
        return base
    return replace(base, **overrides)


class MonitorStateTests(unittest.TestCase):
    def test_capture_warn_threshold(self) -> None:
        camera = CameraState("LEFT", "/dev/video0", True, 640, 480, CAPTURE_WARN_MS + 0.1)
        self.assertTrue(camera.degraded)
        self.assertFalse(MonitorState(
            runtime_s=0,
            fps=0,
            frame=1,
            model="m.onnx",
            threads=1,
            inference_ms=10,
            cameras=(camera, camera),
            left=DetectionState(False),
            right=DetectionState(False),
            usb=UsbCdcState(False, "", False, "", "", None, None, ""),
            last_error="",
        ).performance_ok)

    def test_cdc_issue_when_dropped(self) -> None:
        usb = UsbCdcState(True, "/dev/ttyGS0", True, "configured", "high", 10, 2, "")
        self.assertTrue(usb.has_issue)

    def test_format_runtime(self) -> None:
        self.assertEqual(format_runtime(261), "00:04:21")


class MonitorRenderTests(unittest.TestCase):
    def test_render_contains_sections(self) -> None:
        buffer = io.StringIO()
        console = Console(file=buffer, force_terminal=True, width=120)
        console.print(render_dashboard(sample_state()))
        output = buffer.getvalue()
        for token in ("ROBOCON VISION", "CAMERAS", "INFERENCE", "DETECTION", "USB CDC", "HEALTH"):
            self.assertIn(token, output)

    def test_render_shows_detection_when_found(self) -> None:
        state = sample_state(
            left=DetectionState(True, "red_block", 1, 0.91, 0.42, 0.58, 18.0),
        )
        buffer = io.StringIO()
        console = Console(file=buffer, force_terminal=True, width=120)
        console.print(render_dashboard(state))
        output = buffer.getvalue()
        self.assertIn("red_block", output)
        self.assertIn("0.91", output)


class ParserTests(unittest.TestCase):
    def test_json_and_no_ui_exclusive(self) -> None:
        parser = build_parser()
        with self.assertRaises(SystemExit):
            parser.parse_args(["--json", "--no-ui"])

    def test_default_dashboard_flags(self) -> None:
        args = build_parser().parse_args([])
        self.assertFalse(args.json)
        self.assertFalse(args.no_ui)

    def test_json_mode_flag(self) -> None:
        args = build_parser().parse_args(["--json"])
        self.assertTrue(args.json)
        self.assertFalse(args.no_ui)


class FrameRecordTests(unittest.TestCase):
    def test_build_frame_record_shape(self) -> None:
        left = Target(True, 1, "red_block", 0.9, 0.1, 0.2)
        right = Target(False)
        record = build_frame_record(
            frame_valid=False,
            left=left,
            right=right,
            payload=0,
            heartbeat=0,
            no_cdc=True,
        )
        payload = json.dumps(record)
        self.assertIn('"left"', payload)
        self.assertIn('"right"', payload)
        self.assertIn('"cdc": false', payload)


if __name__ == "__main__":
    unittest.main()
