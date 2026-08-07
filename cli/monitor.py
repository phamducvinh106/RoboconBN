from __future__ import annotations

import time
from typing import TextIO

from rich.console import Console, Group, RenderableType
from rich.live import Live
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

from cli.state import DetectionState, MonitorState, format_runtime

REFRESH_HZ = 5.0


def _status_icon(ok: bool, *, warn: bool = False) -> Text:
    if ok:
        return Text("✓", style="green")
    if warn:
        return Text("⚠", style="yellow")
    return Text("✗", style="red")


def _fmt_detection(value: DetectionState, field: str) -> str:
    if not value.found:
        return "-"
    if field == "type":
        if value.class_name:
            return value.class_name
        return str(value.block_type) if value.block_type >= 0 else "-"
    if field == "xy":
        return f"{value.x:.3f}/{value.y:.3f}"
    if field == "conf":
        return f"{value.confidence:.2f}"
    if field == "age":
        return f"{value.age_ms:.0f} ms"
    return "-"


def render_dashboard(state: MonitorState, *, verbose: bool = False) -> RenderableType:
    runtime = Table.grid(expand=True)
    runtime.add_column(ratio=1)
    runtime.add_column(ratio=1)
    runtime.add_column(ratio=1)
    runtime.add_row(
        f"Runtime  {format_runtime(state.runtime_s)}",
        f"FPS {state.fps:.1f}",
        f"#{state.frame}",
    )

    camera_table = Table(show_header=False, box=None, padding=(0, 1), expand=True)
    camera_table.add_column("side", style="bold")
    camera_table.add_column("path")
    camera_table.add_column("status", justify="center")
    camera_table.add_column("size")
    camera_table.add_column("latency", justify="right")
    camera_table.add_column("flag", justify="right")
    for camera in state.cameras:
        status = _status_icon(camera.opened)
        latency = f"{camera.capture_ms:.1f}ms"
        flag = Text("⚠", style="yellow") if camera.degraded else Text("")
        size = f"{camera.width}x{camera.height}" if camera.opened else "-"
        camera_table.add_row(camera.label, camera.path, status, size, latency, flag)

    inference = Table.grid(expand=True)
    inference.add_column()
    inference.add_row(f"Model   {state.model}")
    inference.add_row(f"Time    {state.inference_ms:.1f} ms")
    inference.add_row(f"Threads {state.threads}")

    detection = Table(show_header=False, box=None, padding=(0, 1), expand=True)
    detection.add_column("left", ratio=1)
    detection.add_column("right", ratio=1)
    detection.add_row(Text("LEFT", style="bold"), Text("RIGHT", style="bold"))
    detection.add_row(
        f"Found: {'YES' if state.left.found else 'NO'}",
        f"Found: {'YES' if state.right.found else 'NO'}",
    )
    detection.add_row(
        f"Type:  {_fmt_detection(state.left, 'type')}",
        f"Type:  {_fmt_detection(state.right, 'type')}",
    )
    detection.add_row(
        f"Conf:  {_fmt_detection(state.left, 'conf')}",
        f"Conf:  {_fmt_detection(state.right, 'conf')}",
    )
    detection.add_row(
        f"X/Y:   {_fmt_detection(state.left, 'xy')}",
        f"X/Y:   {_fmt_detection(state.right, 'xy')}",
    )
    detection.add_row(
        f"Age:   {_fmt_detection(state.left, 'age')}",
        f"Age:   {_fmt_detection(state.right, 'age')}",
    )

    if state.usb.enabled:
        cdc_lines = [
            f"{state.usb.device:<16} {'CONNECTED' if state.usb.connected else 'DISCONNECTED'}",
            f"USB configured    {state.usb.usb_state} / {state.usb.usb_speed}",
            (
                f"TX {state.usb.written or 0:<10} "
                f"Drop {state.usb.dropped or 0:<6} "
                f"Errors {1 if state.usb.last_error else 0}"
            ),
        ]
        if state.usb.last_error:
            cdc_lines.append(f"Last: {state.usb.last_error}")
    else:
        cdc_lines = ["CDC disabled (--no-cdc)"]

    health = Table.grid(expand=True)
    health.add_column(ratio=1)
    health.add_column(ratio=1)
    health.add_column(ratio=1)
    health.add_column(ratio=1)
    health.add_row(
        Text.assemble("Vision ", _status_icon(state.vision_ok)),
        Text.assemble("Cameras ", _status_icon(state.cameras_ok)),
        Text.assemble(
            "CDC ",
            _status_icon(state.cdc_ok, warn=state.usb.enabled and not state.usb.connected),
        ),
        Text.assemble(
            "Performance ",
            _status_icon(state.performance_ok, warn=not state.performance_ok),
        ),
    )

    sections: list[RenderableType] = [
        Panel(runtime, title="ROBOCON VISION", border_style="cyan"),
        Panel(camera_table, title="CAMERAS", border_style="blue"),
        Panel(inference, title="INFERENCE", border_style="magenta"),
        Panel(detection, title="DETECTION", border_style="green"),
        Panel("\n".join(cdc_lines), title="USB CDC", border_style="yellow"),
        Panel(health, title="HEALTH", border_style="white"),
    ]

    if state.last_error:
        sections.append(Panel(state.last_error, title="LAST ERROR", border_style="red"))

    if verbose and state.raw_verbose:
        sections.append(
            Panel(
                state.raw_verbose,
                title="VERBOSE (latest frame)",
                border_style="dim",
            )
        )

    return Group(*sections)


class CliMonitor:
    """Rich live dashboard; throttles redraw to ~5 Hz."""

    def __init__(self, *, verbose: bool = False, file: TextIO | None = None) -> None:
        self._verbose = verbose
        self._console = Console(file=file)
        self._state: MonitorState | None = None
        self._live: Live | None = None
        self._last_draw = 0.0
        self._min_interval = 1.0 / REFRESH_HZ

    def start(self) -> None:
        if self._live is not None:
            return
        placeholder = Panel("Starting...", title="ROBOCON VISION", border_style="cyan")
        self._live = Live(
            placeholder,
            console=self._console,
            refresh_per_second=REFRESH_HZ,
            transient=False,
        )
        self._live.start()

    def update(self, state: MonitorState) -> None:
        self._state = state
        now = time.monotonic()
        if self._live is None:
            return
        if now - self._last_draw < self._min_interval:
            return
        self._last_draw = now
        self._live.update(render_dashboard(state, verbose=self._verbose))

    def close(self) -> None:
        if self._live is not None:
            if self._state is not None:
                self._live.update(render_dashboard(self._state, verbose=self._verbose))
            self._live.stop()
            self._live = None

    def __enter__(self) -> CliMonitor:
        self.start()
        return self

    def __exit__(self, *_args: object) -> None:
        self.close()
