"""ONNX Runtime inference for Ultralytics YOLO pose exports (no PyTorch/CUDA)."""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort


@dataclass(frozen=True, slots=True)
class PoseDetection:
    confidence: float
    class_id: int
    class_name: str
    xyxy: tuple[float, float, float, float]


def _letterbox(
    image: np.ndarray,
    new_shape: tuple[int, int],
    *,
    stride: int = 32,
    color: int = 114,
) -> tuple[np.ndarray, tuple[float, float], tuple[int, int]]:
    """Resize with aspect ratio and center padding (Ultralytics LetterBox defaults)."""
    shape = image.shape[:2]
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    new_unpad = (round(shape[1] * r), round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]
    dw = (dw % stride) / 2
    dh = (dh % stride) / 2
    if shape[::-1] != new_unpad:
        image = cv2.resize(image, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = round(dh - 0.1), round(dh + 0.1)
    left, right = round(dw - 0.1), round(dw + 0.1)
    image = cv2.copyMakeBorder(image, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(color, color, color))
    return image, (r, r), (left, top)


def _xywh2xyxy(boxes: np.ndarray) -> np.ndarray:
    xy = boxes[:, :2]
    wh = boxes[:, 2:4] / 2.0
    out = np.empty_like(boxes)
    out[:, 0:2] = xy - wh
    out[:, 2:4] = xy + wh
    return out


def _scale_boxes(
    img_shape: tuple[int, int],
    boxes: np.ndarray,
    orig_shape: tuple[int, int],
    ratio_pad: tuple[tuple[float, float], tuple[int, int]],
) -> np.ndarray:
    gain = ratio_pad[0][0]
    pad_x, pad_y = ratio_pad[1]
    boxes = boxes.copy()
    boxes[:, 0] -= pad_x
    boxes[:, 1] -= pad_y
    boxes[:, 2] -= pad_x
    boxes[:, 3] -= pad_y
    boxes[:, :4] /= gain
    boxes[:, 0] = np.clip(boxes[:, 0], 0, orig_shape[1])
    boxes[:, 1] = np.clip(boxes[:, 1], 0, orig_shape[0])
    boxes[:, 2] = np.clip(boxes[:, 2], 0, orig_shape[1])
    boxes[:, 3] = np.clip(boxes[:, 3], 0, orig_shape[0])
    return boxes


def _parse_names(metadata: dict[str, str]) -> dict[int, str]:
    raw = metadata.get("names", "")
    if not raw:
        return {}
    try:
        names = json.loads(raw.replace("'", '"'))
    except json.JSONDecodeError:
        return {}
    if isinstance(names, list):
        return {index: str(name) for index, name in enumerate(names)}
    if isinstance(names, dict):
        return {int(key): str(value) for key, value in names.items()}
    return {}


class OnnxPoseModel:
    """Run YOLO11 pose ONNX models with CPUExecutionProvider only."""

    def __init__(self, model_path: Path) -> None:
        self.model_path = model_path
        self.session = ort.InferenceSession(
            str(model_path),
            providers=["CPUExecutionProvider"],
        )
        metadata = self.session.get_modelmeta().custom_metadata_map
        self.names = _parse_names(dict(metadata))
        self.stride = int(metadata.get("stride", "32"))
        self.input_name = self.session.get_inputs()[0].name
        self.output_names = [output.name for output in self.session.get_outputs()]

    def predict(
        self,
        frame: np.ndarray,
        *,
        image_size: int,
        confidence: float,
        class_filter: str | int | None = None,
    ) -> list[PoseDetection]:
        orig_shape = frame.shape[:2]
        letterboxed, ratio, pad = _letterbox(
            frame,
            (image_size, image_size),
            stride=self.stride,
        )
        blob = letterboxed[:, :, ::-1].astype(np.float32) / 255.0
        blob = np.transpose(blob, (2, 0, 1))
        blob = np.expand_dims(blob, 0)
        output = self.session.run(self.output_names, {self.input_name: blob})[0]
        if output.ndim != 3:
            return []
        prediction = np.transpose(output[0], (1, 0))
        if prediction.shape[1] < 8:
            return []

        nc = 4
        extra = prediction.shape[1] - nc - 4
        if extra < 0:
            return []

        boxes = _xywh2xyxy(prediction[:, :4])
        class_scores = prediction[:, 4:4 + nc]
        class_ids = np.argmax(class_scores, axis=1)
        confidences = class_scores[np.arange(class_scores.shape[0]), class_ids]
        keep = confidences >= confidence
        if not np.any(keep):
            return []

        boxes = boxes[keep]
        class_ids = class_ids[keep]
        confidences = confidences[keep]
        boxes = _scale_boxes((image_size, image_size), boxes, orig_shape, (ratio, pad))

        detections: list[PoseDetection] = []
        for box, class_id, score in zip(boxes, class_ids, confidences, strict=True):
            class_name = self.names.get(int(class_id), str(int(class_id)))
            if isinstance(class_filter, int) and int(class_id) != class_filter:
                continue
            if isinstance(class_filter, str) and class_name.casefold() != class_filter.casefold():
                continue
            detections.append(
                PoseDetection(
                    confidence=float(score),
                    class_id=int(class_id),
                    class_name=class_name,
                    xyxy=(float(box[0]), float(box[1]), float(box[2]), float(box[3])),
                )
            )
        return detections
