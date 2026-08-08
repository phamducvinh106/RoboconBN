"""ONNX Runtime inference for Ultralytics YOLO exports (no PyTorch/CUDA)."""

from __future__ import annotations

import ast
import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import onnxruntime as ort

from block_detected_for_pi.model_benchmark.compat import ModelInputSpec, inspect_model_input

EXPECTED_CLASS_COUNT = 4


@dataclass(frozen=True, slots=True)
class PoseDetection:
    confidence: float
    class_id: int
    class_name: str
    xyxy: tuple[float, float, float, float]


def _resize_exact(
    image: np.ndarray,
    new_shape: tuple[int, int],
) -> tuple[np.ndarray, tuple[float, float], tuple[int, int]]:
    height, width = image.shape[:2]
    target_h, target_w = new_shape
    resized = cv2.resize(image, (target_w, target_h), interpolation=cv2.INTER_LINEAR)
    gain_y = target_h / height
    gain_x = target_w / width
    return resized, (gain_y, gain_x), (0, 0)


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
    if isinstance(ratio_pad[0], tuple) and ratio_pad[0][0] != ratio_pad[0][1]:
        boxes[:, 0] /= ratio_pad[0][1]
        boxes[:, 1] /= ratio_pad[0][0]
        boxes[:, 2] /= ratio_pad[0][1]
        boxes[:, 3] /= ratio_pad[0][0]
    else:
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
        names = ast.literal_eval(raw)
    except (SyntaxError, ValueError):
        try:
            names = json.loads(raw.replace("'", '"'))
        except json.JSONDecodeError:
            return {}
    if isinstance(names, list):
        return {index: str(name) for index, name in enumerate(names)}
    if isinstance(names, dict):
        return {int(key): str(value) for key, value in names.items()}
    return {}


def validate_model_schema(names: dict[int, str], output_channels: int | None = None) -> None:
    if len(names) != EXPECTED_CLASS_COUNT:
        raise ValueError(f"expected {EXPECTED_CLASS_COUNT} classes, found {len(names)}")
    if output_channels is not None and output_channels < 4 + len(names):
        raise ValueError(
            f"output channels {output_channels} too small for {len(names)} classes"
        )


class OnnxPoseModel:
    """Run YOLO ONNX models with CPUExecutionProvider only."""

    def __init__(self, model_path: Path, *, threads: int = 4) -> None:
        self.model_path = model_path
        options = ort.SessionOptions()
        options.log_severity_level = 3
        options.intra_op_num_threads = threads
        options.inter_op_num_threads = 1
        options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
        self.session = ort.InferenceSession(
            str(model_path),
            sess_options=options,
            providers=["CPUExecutionProvider"],
        )
        metadata = self.session.get_modelmeta().custom_metadata_map
        self.metadata = dict(metadata)
        self.names = _parse_names(self.metadata)
        self.stride = int(self.metadata.get("stride", "32"))
        self.task = str(self.metadata.get("task", "detect"))
        self.input_name = self.session.get_inputs()[0].name
        self.output_names = [output.name for output in self.session.get_outputs()]
        output_shape = self.session.get_outputs()[0].shape
        output_channels = int(output_shape[1]) if len(output_shape) > 1 and isinstance(output_shape[1], int) else None
        validate_model_schema(self.names, output_channels)
        self.input_spec = inspect_model_input(model_path)

    def effective_image_size(self, image_size: int) -> int:
        return self.input_spec.spatial_size or image_size

    def _prepare_frames(
        self,
        frames: list[np.ndarray] | tuple[np.ndarray, ...],
        *,
        image_size: int,
    ) -> tuple[list[np.ndarray], list[tuple[tuple[int, int], tuple[tuple[float, float], tuple[int, int]]]], tuple[int, int]]:
        prepared: list[np.ndarray] = []
        contexts: list[tuple[tuple[int, int], tuple[tuple[float, float], tuple[int, int]]]] = []
        effective_size = self.effective_image_size(image_size)
        input_shape: tuple[int, int] | None = None
        for frame in frames:
            orig_shape = frame.shape[:2]
            if not self.input_spec.dynamic_spatial and self.input_spec.spatial_size is not None:
                resized, ratio, pad = _resize_exact(frame, (effective_size, effective_size))
            else:
                resized, ratio, pad = _letterbox(
                    frame,
                    (effective_size, effective_size),
                    stride=self.stride,
                )
            letterboxed = resized
            if input_shape is None:
                input_shape = letterboxed.shape[:2]
            elif letterboxed.shape[:2] != input_shape:
                raise ValueError("batch frames must produce the same ONNX input shape")
            prepared.append(np.transpose(letterboxed[:, :, ::-1], (2, 0, 1)))
            contexts.append((orig_shape, (ratio, pad)))
        if not prepared or input_shape is None:
            return [], [], (0, 0)
        return prepared, contexts, input_shape

    def _run_prepared_batch(
        self,
        prepared: list[np.ndarray],
        contexts: list[tuple[tuple[int, int], tuple[tuple[float, float], tuple[int, int]]]],
        input_shape: tuple[int, int],
        *,
        confidence: float,
        class_filter: str | int | None,
    ) -> list[list[PoseDetection]]:
        blob = np.ascontiguousarray(np.stack(prepared), dtype=np.float32)
        blob /= 255.0
        output = self.session.run(self.output_names, {self.input_name: blob})[0]
        if output.ndim != 3 or output.shape[0] != len(prepared):
            return [[] for _ in prepared]
        return [
            self._decode(
                prediction,
                input_shape=input_shape,
                orig_shape=orig_shape,
                ratio_pad=ratio_pad,
                confidence=confidence,
                class_filter=class_filter,
            )
            for prediction, (orig_shape, ratio_pad) in zip(output, contexts, strict=True)
        ]

    def _decode(
        self,
        prediction: np.ndarray,
        *,
        input_shape: tuple[int, int],
        orig_shape: tuple[int, int],
        ratio_pad: tuple[tuple[float, float], tuple[int, int]],
        confidence: float,
        class_filter: str | int | None,
    ) -> list[PoseDetection]:
        prediction = np.transpose(prediction, (1, 0))
        nc = len(self.names)
        if nc == 0 or prediction.shape[1] < 4 + nc:
            return []

        boxes = _xywh2xyxy(prediction[:, :4])
        class_scores = prediction[:, 4:4 + nc]
        class_ids = np.argmax(class_scores, axis=1)
        confidences = class_scores[np.arange(class_scores.shape[0]), class_ids]
        keep = confidences >= confidence
        if not np.any(keep):
            return []

        boxes = _scale_boxes(input_shape, boxes[keep], orig_shape, ratio_pad)
        class_ids = class_ids[keep]
        confidences = confidences[keep]
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

    def predict_batch(
        self,
        frames: list[np.ndarray] | tuple[np.ndarray, ...],
        *,
        image_size: int,
        confidence: float,
        class_filter: str | int | None = None,
    ) -> list[list[PoseDetection]]:
        if self.input_spec.batch_fixed == 1 and len(frames) > 1:
            results: list[list[PoseDetection]] = []
            for frame in frames:
                results.extend(
                    self.predict_batch(
                        (frame,),
                        image_size=image_size,
                        confidence=confidence,
                        class_filter=class_filter,
                    )
                )
            return results

        prepared, contexts, input_shape = self._prepare_frames(frames, image_size=image_size)
        if not prepared:
            return [[] for _ in frames]
        return self._run_prepared_batch(
            prepared,
            contexts,
            input_shape,
            confidence=confidence,
            class_filter=class_filter,
        )

    def predict(
        self,
        frame: np.ndarray,
        *,
        image_size: int,
        confidence: float,
        class_filter: str | int | None = None,
    ) -> list[PoseDetection]:
        return self.predict_batch(
            (frame,),
            image_size=image_size,
            confidence=confidence,
            class_filter=class_filter,
        )[0]
