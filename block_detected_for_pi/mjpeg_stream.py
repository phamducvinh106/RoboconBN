from __future__ import annotations

import queue
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2
import numpy as np

# Default stream settings (override via CLI in main.py).
ENABLE_STREAM = False
STREAM_PORT = 8080
JPEG_QUALITY = 80
STREAM_WIDTH = 640
STREAM_HEIGHT = 480

BOUNDARY = b"--frame"
CONTENT_TYPE = f"multipart/x-mixed-replace; boundary={BOUNDARY.decode('ascii')}"


class _StreamState:
    def __init__(self) -> None:
        self.latest_jpeg: bytes | None = None
        self.lock = threading.Lock()
        self.clients = 0


class MjpegStreamer:
    """Non-blocking MJPEG server; encode runs off the vision loop thread."""

    def __init__(
        self,
        *,
        host: str = "0.0.0.0",
        port: int = STREAM_PORT,
        jpeg_quality: int = JPEG_QUALITY,
        stream_width: int = STREAM_WIDTH,
        stream_height: int = STREAM_HEIGHT,
    ) -> None:
        self.host = host
        self.port = port
        self.jpeg_quality = max(1, min(100, jpeg_quality))
        self.stream_width = stream_width
        self.stream_height = stream_height
        self._state = _StreamState()
        self._pending: queue.Queue[np.ndarray | None] = queue.Queue(maxsize=1)
        self._encoder = threading.Thread(target=self._encode_loop, name="mjpeg-encoder", daemon=True)
        self._server: ThreadingHTTPServer | None = None
        self._server_thread: threading.Thread | None = None
        self._encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality]

    @property
    def video_url_path(self) -> str:
        return "/video_feed"

    def start(self) -> None:
        if self._server is not None:
            return
        handler = _make_handler(self._state)
        self._server = ThreadingHTTPServer((self.host, self.port), handler)
        self._encoder.start()
        self._server_thread = threading.Thread(
            target=self._server.serve_forever,
            name="mjpeg-http",
            daemon=True,
        )
        self._server_thread.start()

    def publish(self, frame: np.ndarray) -> None:
        if self._server is None:
            return
        try:
            self._pending.put_nowait(frame)
        except queue.Full:
            try:
                self._pending.get_nowait()
            except queue.Empty:
                pass
            try:
                self._pending.put_nowait(frame)
            except queue.Full:
                return

    def close(self) -> None:
        try:
            self._pending.put_nowait(None)
        except queue.Full:
            pass
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None
        if self._encoder.is_alive():
            self._encoder.join(timeout=0.5)
        if self._server_thread is not None and self._server_thread.is_alive():
            self._server_thread.join(timeout=0.5)

    def _encode_loop(self) -> None:
        while True:
            frame = self._pending.get()
            if frame is None:
                return
            if self.stream_width > 0 and self.stream_height > 0:
                frame = cv2.resize(frame, (self.stream_width, self.stream_height))
            ok, encoded = cv2.imencode(".jpg", frame, self._encode_params)
            if not ok:
                continue
            payload = encoded.tobytes()
            with self._state.lock:
                self._state.latest_jpeg = payload


def _make_handler(state: _StreamState) -> type[BaseHTTPRequestHandler]:
    video_path = "/video_feed"

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, _format: str, *_args: object) -> None:
            return

        def do_GET(self) -> None:
            if self.path in {"/", "/index.html"}:
                body = (
                    "<html><body>"
                    "<h1>Pi Vision MJPEG</h1>"
                    f'<img src="{video_path}" />'
                    "</body></html>"
                ).encode("utf-8")
                self.send_response(HTTPStatus.OK)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return

            if self.path.split("?", 1)[0] != "/video_feed":
                self.send_error(HTTPStatus.NOT_FOUND)
                return

            self.send_response(HTTPStatus.OK)
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Connection", "close")
            self.send_header("Content-Type", CONTENT_TYPE)
            self.end_headers()

            with state.lock:
                state.clients += 1
            try:
                while True:
                    with state.lock:
                        jpeg = state.latest_jpeg
                    if jpeg is None:
                        time.sleep(0.02)
                        continue
                    try:
                        self.wfile.write(BOUNDARY + b"\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(f"Content-Length: {len(jpeg)}\r\n\r\n".encode("ascii"))
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                        self.wfile.flush()
                    except (BrokenPipeError, ConnectionResetError, OSError):
                        break
                    time.sleep(0.03)
            finally:
                with state.lock:
                    state.clients = max(0, state.clients - 1)

    return Handler
