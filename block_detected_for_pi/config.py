from __future__ import annotations

from dataclasses import dataclass, field


DEFAULT_BLOCK_TYPES = ("01", "02", "03", "04")
PROTO_VERSION = 1
REGISTER_COUNT = 6
DEFAULT_UART_BAUD = 9600


@dataclass(frozen=True, slots=True)
class BlockCodeConfig:
    """Map detector class identifiers to packed 2-bit block codes 0..3."""

    class_to_code: dict[str | int, int] = field(default_factory=dict)
    block_types: tuple[str, ...] = DEFAULT_BLOCK_TYPES

    def code_for(self, class_id: int, class_name: str = "") -> int | None:
        if class_id in self.class_to_code:
            return self._normalize_code(self.class_to_code[class_id])
        if class_name and class_name in self.class_to_code:
            return self._normalize_code(self.class_to_code[class_name])
        if 0 <= class_id < len(self.block_types):
            return class_id
        return None

    def block_type_for_code(self, code: int) -> str | None:
        if 0 <= code < len(self.block_types):
            return self.block_types[code]
        return None

    @staticmethod
    def _normalize_code(value: int) -> int | None:
        if 0 <= value <= 3:
            return value
        return None
