from __future__ import annotations

import cv2
import numpy as np

from block_detected_for_pi.types import Target

GREEN = (0, 255, 0)
YELLOW = (0, 255, 255)
WHITE = (255, 255, 255)
RED = (0, 0, 255)


def annotate_frame(
    frame: np.ndarray,
    target: Target,
    *,
    label: str,
    fps: float,
) -> np.ndarray:
    """Draw center point, class, coordinates and FPS on a BGR frame."""
    annotated = frame.copy()
    height, width = annotated.shape[:2]

    header = f"{label}  FPS {fps:.1f}"
    cv2.putText(
        annotated,
        header,
        (8, 22),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        GREEN,
        2,
        cv2.LINE_AA,
    )

    if target.found:
        cx = int(target.x * width)
        cy = int(target.y * height)
        cv2.drawMarker(
            annotated,
            (cx, cy),
            RED,
            markerType=cv2.MARKER_CROSS,
            markerSize=18,
            thickness=2,
        )
        cv2.circle(annotated, (cx, cy), 6, YELLOW, 2, cv2.LINE_AA)
        class_label = target.class_name or str(target.class_id)
        text = (
            f"class={class_label} conf={target.confidence:.2f} "
            f"center=({target.x:.3f},{target.y:.3f})"
        )
        cv2.putText(
            annotated,
            text,
            (8, height - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            WHITE,
            1,
            cv2.LINE_AA,
        )
    else:
        cv2.putText(
            annotated,
            "NO BLOCK",
            (8, height - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            YELLOW,
            2,
            cv2.LINE_AA,
        )
    return annotated


def compose_stream_frame(
    left: np.ndarray,
    right: np.ndarray,
    *,
    mode: str = "both",
) -> np.ndarray:
    """Build one preview frame for MJPEG stream."""
    if mode == "left":
        return left
    if mode == "right":
        return right
    if left.shape[0] != right.shape[0]:
        target_h = min(left.shape[0], right.shape[0])
        left = cv2.resize(left, (int(left.shape[1] * target_h / left.shape[0]), target_h))
        right = cv2.resize(right, (int(right.shape[1] * target_h / right.shape[0]), target_h))
    return np.hstack((left, right))
