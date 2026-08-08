#!/usr/bin/env python3
"""Click block centers for the 108-image test set."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from block_detected_for_pi.model_benchmark.ground_truth import (  # noqa: E402
    DEFAULT_CSV,
    GroundTruthEntry,
    bootstrap_centers,
    build_class_entries,
    list_dataset_images,
    load_ground_truth,
    refine_centers_with_model,
    save_ground_truth,
    validate_ground_truth,
)


class Annotator:
    def __init__(self, entries: list[GroundTruthEntry], dataset_dir: Path) -> None:
        self.entries = entries
        self.dataset_dir = dataset_dir
        self.index = 0
        self.pending_click: tuple[float, float] | None = None

    def _current(self) -> GroundTruthEntry:
        return self.entries[self.index]

    def _draw(self, frame):
        entry = self._current()
        display = frame.copy()
        if entry.annotated:
            cx = int(entry.center_x * frame.shape[1])
            cy = int(entry.center_y * frame.shape[0])
            cv2.circle(display, (cx, cy), 8, (0, 255, 0), 2)
        if self.pending_click is not None:
            cx = int(self.pending_click[0] * frame.shape[1])
            cy = int(self.pending_click[1] * frame.shape[0])
            cv2.circle(display, (cx, cy), 8, (0, 255, 255), 2)
        text = (
            f"{entry.image} class={entry.class_name} "
            f"[{self.index + 1}/{len(self.entries)}] "
            "click=center s=save n=next b=back q=quit"
        )
        cv2.putText(display, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        return display

    def _on_mouse(self, event, x, y, _flags, _userdata) -> None:
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        frame = cv2.imread(str(self.dataset_dir / self._current().image))
        if frame is None:
            return
        self.pending_click = (x / frame.shape[1], y / frame.shape[0])

    def run(self) -> None:
        window = "annotate_block_centers"
        cv2.namedWindow(window)
        cv2.setMouseCallback(window, self._on_mouse)
        while 0 <= self.index < len(self.entries):
            entry = self._current()
            frame = cv2.imread(str(self.dataset_dir / entry.image))
            if frame is None:
                self.index += 1
                continue
            self.pending_click = None
            while True:
                cv2.imshow(window, self._draw(frame))
                key = cv2.waitKey(20) & 0xFF
                if key in {ord("q"), 27}:
                    cv2.destroyAllWindows()
                    return
                if key == ord("b"):
                    self.index = max(0, self.index - 1)
                    break
                if key == ord("n"):
                    self.index += 1
                    break
                if key == ord("s") and self.pending_click is not None:
                    self.entries[self.index] = GroundTruthEntry(
                        image=entry.image,
                        class_id=entry.class_id,
                        class_name=entry.class_name,
                        center_x=self.pending_click[0],
                        center_y=self.pending_click[1],
                        annotated=True,
                    )
                    self.index += 1
                    break
        cv2.destroyAllWindows()


def main() -> int:
    parser = argparse.ArgumentParser(description="Annotate block centers for test dataset")
    parser.add_argument("--dataset", type=Path, default=DEFAULT_CSV.parent)
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV)
    parser.add_argument("--bootstrap", action="store_true", help="auto-estimate centers before manual refine")
    parser.add_argument("--refine-model", type=Path, help="refine centers using a trusted ONNX model")
    parser.add_argument("--validate-only", action="store_true")
    args = parser.parse_args()

    entries = load_ground_truth(args.csv)
    if not entries:
        entries = build_class_entries(args.dataset)
    if args.bootstrap:
        entries = bootstrap_centers(entries, dataset_dir=args.dataset)
    if args.refine_model is not None:
        entries = refine_centers_with_model(
            entries,
            dataset_dir=args.dataset,
            model_path=args.refine_model,
        )
    if args.bootstrap or args.refine_model is not None:
        save_ground_truth(entries, args.csv)
        print(f"saved centers -> {args.csv}")

    errors = validate_ground_truth(entries, dataset_dir=args.dataset, require_annotated=not args.bootstrap)
    if errors:
        print("validation issues:")
        for error in errors[:20]:
            print(f"  - {error}")
        if len(errors) > 20:
            print(f"  ... and {len(errors) - 20} more")
    elif args.validate_only:
        print(f"OK: {len(entries)} rows validated")
        return 0

    if args.validate_only:
        return 1 if errors else 0

    if errors and not args.bootstrap:
        print("run with --bootstrap first or annotate missing centers")
        return 1

    annotator = Annotator(entries, args.dataset)
    annotator.run()
    save_ground_truth(annotator.entries, args.csv)
    print(f"saved {args.csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
