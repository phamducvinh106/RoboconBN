from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any


@dataclass(frozen=True, slots=True)
class Target:
    found: bool
    class_id: int = -1
    class_name: str = ""
    confidence: float = 0.0
    x: float = 0.0
    y: float = 0.0
    error_x: float = 0.0
    error_y: float = 0.0
    angle: float = 0.0

    def packet(self) -> dict[str, Any]:
        return asdict(self)
