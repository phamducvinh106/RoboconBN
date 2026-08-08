from __future__ import annotations

import unittest

import cv2
import numpy as np

from block_detected_for_pi.mjpeg_stream import BOUNDARY, CONTENT_TYPE, MjpegStreamer
from block_detected_for_pi.overlay import annotate_frame, compose_stream_frame
from block_detected_for_pi.types import Target


class OverlayTests(unittest.TestCase):
    def test_annotate_found(self) -> None:
        frame = np.zeros((120, 160, 3), dtype=np.uint8)
        target = Target(True, 2, "3", 0.91, 0.5, 0.5)
        annotated = annotate_frame(frame, target, label="LEFT", fps=7.5)
        self.assertEqual(annotated.shape, frame.shape)
        self.assertFalse(np.array_equal(annotated, frame))

    def test_compose_both(self) -> None:
        left = np.zeros((100, 80, 3), dtype=np.uint8)
        right = np.ones((100, 80, 3), dtype=np.uint8) * 255
        composed = compose_stream_frame(left, right, mode="both")
        self.assertEqual(composed.shape[1], 160)


class StreamTests(unittest.TestCase):
    def test_boundary_constants(self) -> None:
        self.assertIn(b"frame", BOUNDARY)
        self.assertIn("multipart/x-mixed-replace", CONTENT_TYPE)

    def test_streamer_start_close(self) -> None:
        streamer = MjpegStreamer(port=0)
        streamer.start()
        frame = np.zeros((48, 64, 3), dtype=np.uint8)
        streamer.publish(frame)
        streamer.close()


if __name__ == "__main__":
    unittest.main()
