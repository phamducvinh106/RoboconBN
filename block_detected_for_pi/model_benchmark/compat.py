from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import onnxruntime as ort


@dataclass(frozen=True, slots=True)
class ModelInputSpec:
    batch_fixed: int | None
    height: int | None
    width: int | None
    dynamic_batch: bool
    dynamic_spatial: bool

    @property
    def spatial_size(self) -> int | None:
        if self.height is not None and self.width is not None and self.height == self.width:
            return self.height
        return None

    def describe(self) -> str:
        batch = "dynamic" if self.dynamic_batch else str(self.batch_fixed)
        if self.dynamic_spatial:
            spatial = "dynamic"
        elif self.spatial_size is not None:
            spatial = str(self.spatial_size)
        else:
            spatial = f"{self.height}x{self.width}"
        return f"batch={batch}, spatial={spatial}"


def _parse_dim(value: object) -> int | None:
    if isinstance(value, int):
        return value
    if isinstance(value, str) and value.isdigit():
        return int(value)
    return None


def inspect_model_input(model_path: Path) -> ModelInputSpec:
    session = ort.InferenceSession(str(model_path), providers=["CPUExecutionProvider"])
    shape = session.get_inputs()[0].shape
    batch = _parse_dim(shape[0]) if len(shape) > 0 else None
    height = _parse_dim(shape[2]) if len(shape) > 2 else None
    width = _parse_dim(shape[3]) if len(shape) > 3 else None
    return ModelInputSpec(
        batch_fixed=batch if batch == 1 else None,
        height=height,
        width=width,
        dynamic_batch=batch is None,
        dynamic_spatial=height is None or width is None,
    )
