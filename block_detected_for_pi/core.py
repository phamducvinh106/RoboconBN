"""YOLO targeting core for one camera frame."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from ultralytics import YOLO

from .types import Target


class TargetingCore:
    """Load one YOLO model and detect the best target in a frame."""

    def __init__(
        self,
        model: str | Path,
        *,
        image_size: int = 320,
        confidence: float = 0.25,
        class_filter: str | int | None = None,
    ) -> None:
        model_path = Path(model).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(f"model not found: {model}")
        self.model_path = model_path
        self.image_size = image_size
        self.confidence = confidence
        self.class_filter = class_filter
        self._model = YOLO(str(model_path), task="pose")

    def process(self, frame: Any) -> Target:
        height, width = frame.shape[:2]
        if width <= 0 or height <= 0:
            raise ValueError("frame dimensions must be positive")
        result = self._model(
            frame,
            conf=self.confidence,
            imgsz=self.image_size,
            max_det=8,
            verbose=False,
        )[0]
        detections = self._detections(result)
        if not detections:
            return Target(found=False)
        confidence, class_id, class_name, (x1, y1, x2, y2) = max(detections)
        x = ((x1 + x2) / 2.0) / width
        y = ((y1 + y2) / 2.0) / height
        return Target(
            found=True,
            class_id=class_id,
            class_name=class_name,
            confidence=confidence,
            x=x,
            y=y,
            error_x=2.0 * x - 1.0,
            error_y=2.0 * y - 1.0,
            angle=0.0,
        )

    def _detections(self, result) -> list[tuple[float, int, str, tuple[float, float, float, float]]]:
        if result.boxes is None:
            return []
        names = result.names
        detections: list[tuple[float, int, str, tuple[float, float, float, float]]] = []
        for box in result.boxes:
            class_id = int(box.cls.item())
            class_name = str(names[class_id])
            if isinstance(self.class_filter, int) and class_id != self.class_filter:
                continue
            if isinstance(self.class_filter, str) and class_name.casefold() != self.class_filter.casefold():
                continue
            xyxy = box.xyxy[0].tolist()
            detections.append((float(box.conf.item()), class_id, class_name, tuple(xyxy)))
        return detections

    def close(self) -> None:
        del self._model

    def __enter__(self) -> "TargetingCore":
        return self

    def __exit__(self, _type, _value, _traceback) -> None:
        self.close()
