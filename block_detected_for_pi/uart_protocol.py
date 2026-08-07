from __future__ import annotations

FRAME_PREFIX = "$V1,"


def crc8_ascii(data: str) -> int:
    value = 0
    for byte in data.encode("ascii"):
        value ^= byte
    return value & 0xFF


def format_frame(*, heartbeat: int, status: int, payload: int) -> str:
    body = f"{FRAME_PREFIX}{heartbeat & 0xFF},{status & 0xFF},{payload & 0xFFFFF:05X}"
    checksum = crc8_ascii(body)
    return f"{body},{checksum:02X}\n"


def parse_frame(line: str) -> tuple[int, int, int, int]:
    text = line.strip()
    if not text.startswith(FRAME_PREFIX):
        raise ValueError("invalid frame prefix")
    parts = text.split(",")
    if len(parts) != 5:
        raise ValueError("invalid field count")
    heartbeat = int(parts[1], 10)
    status = int(parts[2], 10)
    payload = int(parts[3], 16)
    expected = int(parts[4], 16)
    body = ",".join(parts[:4])
    if crc8_ascii(body) != expected:
        raise ValueError("crc mismatch")
    if payload < 0 or payload > 0xFFFFF:
        raise ValueError("payload out of range")
    return heartbeat, status, payload, expected
