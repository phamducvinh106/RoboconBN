from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from block_detected_for_pi.cdc_publisher import CdcPublisher, PROTOCOL_VERSION


class _MockSerial:
    def write(self, data: bytes) -> int:
        self.line = data.decode("ascii")
        return len(data)

    def close(self) -> None:
        return


def verify_cdc_packet() -> int:
    port = _MockSerial()
    publisher = CdcPublisher.__new__(CdcPublisher)
    publisher._serial_timeout = Exception  # type: ignore[attr-defined]
    publisher._serial = port  # type: ignore[attr-defined]
    import queue
    import threading

    publisher._pending = queue.Queue(maxsize=1)  # type: ignore[attr-defined]
    publisher._heartbeat = 0  # type: ignore[attr-defined]
    publisher.dropped = 0  # type: ignore[attr-defined]
    publisher.written = 0  # type: ignore[attr-defined]
    publisher.last_error = ""  # type: ignore[attr-defined]
    publisher._worker = threading.Thread(target=publisher._write_loop, daemon=True)  # type: ignore[attr-defined]
    publisher._worker.start()  # type: ignore[attr-defined]

    left = SimpleNamespace(found=True, class_id=1, class_name="02", confidence=0.9, x=0.4, y=0.5)
    right = SimpleNamespace(found=True, class_id=3, class_name="04", confidence=0.8, x=0.6, y=0.5)
    publisher.publish(left=left, right=right, frame_valid=True)
    publisher.close()

    packet = json.loads(port.line.strip())
    assert packet["v"] == PROTOCOL_VERSION
    assert packet["hb"] == 1
    assert packet["frame_valid"] is True
    print("CDC packet layout OK:", json.dumps(packet, separators=(",", ":")))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Pi5 transport bench utilities")
    parser.add_argument("--mode", choices=("cdc-packet",), default="cdc-packet")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.mode == "cdc-packet":
        return verify_cdc_packet()
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
