from __future__ import annotations

from dataclasses import dataclass

from .types import Target

X_MASK = 0x000FF
Y_MASK = 0x0FF00
LEFT_TYPE_MASK = 0x30000
RIGHT_TYPE_MASK = 0xC0000

STATUS_FRAME_VALID = 0x01
STATUS_LEFT_FOUND = 0x02
STATUS_RIGHT_FOUND = 0x04
STATUS_PROTO_OK = 0x80


@dataclass(frozen=True, slots=True)
class DecodedPayload:
    x: int
    y: int
    left_code: int
    right_code: int

    @property
    def raw(self) -> int:
        return pack_payload(self.x, self.y, self.left_code, self.right_code)


def clamp_byte(value: float) -> int:
    return max(0, min(255, int(round(value))))


def normalized_to_byte(value: float) -> int:
    return clamp_byte(value * 255.0)


def pack_payload(x: int, y: int, left_code: int, right_code: int) -> int:
    return (
        (x & 0xFF)
        | ((y & 0xFF) << 8)
        | ((left_code & 0x3) << 16)
        | ((right_code & 0x3) << 18)
    )


def unpack_payload(payload: int) -> DecodedPayload:
    return DecodedPayload(
        x=payload & X_MASK,
        y=(payload & Y_MASK) >> 8,
        left_code=(payload & LEFT_TYPE_MASK) >> 16,
        right_code=(payload & RIGHT_TYPE_MASK) >> 18,
    )


def build_status(*, frame_valid: bool, left_found: bool, right_found: bool) -> int:
    status = STATUS_PROTO_OK
    if frame_valid:
        status |= STATUS_FRAME_VALID
    if left_found:
        status |= STATUS_LEFT_FOUND
    if right_found:
        status |= STATUS_RIGHT_FOUND
    return status


def build_registers(
    *,
    payload: int,
    frame_valid: bool,
    left_found: bool,
    right_found: bool,
    heartbeat: int,
    proto_version: int = 1,
) -> bytes:
    status = build_status(
        frame_valid=frame_valid,
        left_found=left_found,
        right_found=right_found,
    )
    return bytes(
        [
            status,
            payload & 0xFF,
            (payload >> 8) & 0xFF,
            (payload >> 16) & 0x0F,
            heartbeat & 0xFF,
            proto_version & 0xFF,
        ]
    )


def frame_from_targets(
    left: Target,
    right: Target,
    *,
    left_code: int | None,
    right_code: int | None,
    min_confidence: float,
) -> tuple[int, bool, bool, bool]:
    left_found = left.found and left.confidence >= min_confidence and left_code is not None
    right_found = right.found and right.confidence >= min_confidence and right_code is not None
    frame_valid = left_found and right_found
    if not frame_valid:
        return 0, False, left_found, right_found
    payload = pack_payload(
        normalized_to_byte(left.x),
        normalized_to_byte(left.y),
        left_code,
        right_code,
    )
    return payload, True, True, True
