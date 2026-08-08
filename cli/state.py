from __future__ import annotations

from dataclasses import dataclass

CAPTURE_WARN_MS = 50.0


@dataclass(frozen=True, slots=True)
class CameraState:
    label: str
    path: str
    opened: bool
    width: int
    height: int
    capture_ms: float

    @property
    def degraded(self) -> bool:
        return self.opened and self.capture_ms > CAPTURE_WARN_MS


@dataclass(frozen=True, slots=True)
class DetectionState:
    found: bool
    class_name: str = ""
    block_type: int = -1
    confidence: float = 0.0
    x: float = 0.0
    y: float = 0.0
    age_ms: float = 0.0


@dataclass(frozen=True, slots=True)
class UsbCdcState:
    enabled: bool
    device: str
    connected: bool
    usb_state: str
    usb_speed: str
    written: int | None
    dropped: int | None
    last_error: str

    @property
    def has_issue(self) -> bool:
        if not self.enabled:
            return False
        if not self.connected:
            return True
        if self.dropped and self.dropped > 0:
            return True
        return bool(self.last_error)


@dataclass(frozen=True, slots=True)
class MonitorState:
    runtime_s: float
    fps: float
    frame: int
    model: str
    threads: int
    inference_ms: float
    cameras: tuple[CameraState, CameraState]
    left: DetectionState
    right: DetectionState
    usb: UsbCdcState
    last_error: str
    raw_verbose: str | None = None

    @property
    def cameras_ok(self) -> bool:
        return all(camera.opened for camera in self.cameras)

    @property
    def cdc_ok(self) -> bool:
        if not self.usb.enabled:
            return True
        return self.usb.connected and not self.usb.has_issue

    @property
    def performance_ok(self) -> bool:
        if self.inference_ms > CAPTURE_WARN_MS:
            return False
        return not any(camera.degraded for camera in self.cameras)

    @property
    def vision_ok(self) -> bool:
        return self.cameras_ok and not self.last_error


def format_runtime(seconds: float) -> str:
    total = max(0, int(seconds))
    hours, remainder = divmod(total, 3600)
    minutes, secs = divmod(remainder, 60)
    return f"{hours:02d}:{minutes:02d}:{secs:02d}"
