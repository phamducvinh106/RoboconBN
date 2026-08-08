"""YOLO targeting core for one camera frame."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from .onnx_pose import OnnxPoseModel
from .types import Target


class TargetingCore:
    """Load one ONNX pose model and detect the best target in a frame."""

    def __init__(
        self,
        model: str | Path,
        *,
        image_size: int = 320,
        confidence: float = 0.25,
        class_filter: str | int | None = None,
        threads: int = 4,
    ) -> None:
        model_path = Path(model).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(f"model not found: {model}")
        if model_path.suffix.casefold() != ".onnx":
            raise ValueError(f"only ONNX models are supported on Pi: {model_path}")
        self.model_path = model_path
        self.image_size = image_size
        self.confidence = confidence
        self.class_filter = class_filter
        self._model = OnnxPoseModel(model_path, threads=threads)

    @staticmethod
    def _target(frame: Any, detections: list[Any]) -> Target:
        if not detections:
            return Target(found=False)
        height, width = frame.shape[:2]
        best = max(detections, key=lambda item: item.confidence)
        x1, y1, x2, y2 = best.xyxy
        x = ((x1 + x2) / 2.0) / width
        y = ((y1 + y2) / 2.0) / height
        return Target(
            found=True,
            class_id=best.class_id,
            class_name=best.class_name,
            confidence=best.confidence,
            x=x,
            y=y,
            error_x=2.0 * x - 1.0,
            error_y=2.0 * y - 1.0,
            angle=0.0,
        )

    def process_batch(self, frames: list[Any] | tuple[Any, ...]) -> list[Target]:
        for frame in frames:
            height, width = frame.shape[:2]
            if width <= 0 or height <= 0:
                raise ValueError("frame dimensions must be positive")
        detections = self._model.predict_batch(
            frames,
            image_size=self.image_size,
            confidence=self.confidence,
            class_filter=self.class_filter,
        )
        return [self._target(frame, items) for frame, items in zip(frames, detections, strict=True)]

    def process(self, frame: Any) -> Target:
        return self.process_batch((frame,))[0]

    def close(self) -> None:
        del self._model

    def __enter__(self) -> "TargetingCore":
        return self

    def __exit__(self, _type, _value, _traceback) -> None:
        self.close()
