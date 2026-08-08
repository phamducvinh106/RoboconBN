from __future__ import annotations

import csv
import re
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

DATASET_DIR = Path(__file__).resolve().parents[1] / "block_dataset"
DEFAULT_CSV = DATASET_DIR / "ground_truth.csv"
IMAGE_PATTERN = re.compile(r"dt(\d+)\.jpg$", re.IGNORECASE)
CSV_FIELDS = ("image", "class_id", "class_name", "center_x", "center_y", "annotated")


@dataclass(frozen=True, slots=True)
class GroundTruthEntry:
    image: str
    class_id: int
    class_name: str
    center_x: float
    center_y: float
    annotated: bool = False


def image_number(name: str) -> int:
    match = IMAGE_PATTERN.search(name)
    if not match:
        raise ValueError(f"unexpected image name: {name}")
    return int(match.group(1))


def expected_class_id(name: str) -> int:
    """Map dt ranges to ONNX class ids 0..3 (names 1..4)."""
    number = image_number(name)
    if 1 <= number <= 33:
        return 2
    if 34 <= number <= 60:
        return 3
    if 61 <= number <= 86:
        return 1
    if 87 <= number <= 108:
        return 0
    raise ValueError(f"image number out of range: {number}")


def expected_class_name(name: str) -> str:
    return str(expected_class_id(name) + 1)


def list_dataset_images(dataset_dir: Path = DATASET_DIR) -> list[Path]:
    return sorted(dataset_dir.glob("dt*.jpg"), key=lambda path: image_number(path.name))


def load_ground_truth(csv_path: Path = DEFAULT_CSV) -> list[GroundTruthEntry]:
    if not csv_path.is_file():
        return []
    entries: list[GroundTruthEntry] = []
    with csv_path.open(encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            entries.append(
                GroundTruthEntry(
                    image=row["image"],
                    class_id=int(row["class_id"]),
                    class_name=str(row["class_name"]),
                    center_x=float(row["center_x"]),
                    center_y=float(row["center_y"]),
                    annotated=row.get("annotated", "").lower() in {"1", "true", "yes"},
                )
            )
    return entries


def save_ground_truth(entries: list[GroundTruthEntry], csv_path: Path = DEFAULT_CSV) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(CSV_FIELDS))
        writer.writeheader()
        for entry in entries:
            writer.writerow(
                {
                    "image": entry.image,
                    "class_id": entry.class_id,
                    "class_name": entry.class_name,
                    "center_x": f"{entry.center_x:.6f}",
                    "center_y": f"{entry.center_y:.6f}",
                    "annotated": "1" if entry.annotated else "0",
                }
            )


def build_class_entries(dataset_dir: Path = DATASET_DIR) -> list[GroundTruthEntry]:
    entries: list[GroundTruthEntry] = []
    for image_path in list_dataset_images(dataset_dir):
        class_id = expected_class_id(image_path.name)
        entries.append(
            GroundTruthEntry(
                image=image_path.name,
                class_id=class_id,
                class_name=str(class_id + 1),
                center_x=0.0,
                center_y=0.0,
                annotated=False,
            )
        )
    return entries


def bootstrap_centers(
    entries: list[GroundTruthEntry],
    *,
    dataset_dir: Path = DATASET_DIR,
) -> list[GroundTruthEntry]:
    """Estimate block centroid from dark block face near image center."""
    updated: list[GroundTruthEntry] = []
    for entry in entries:
        image_path = dataset_dir / entry.image
        frame = cv2.imread(str(image_path))
        if frame is None:
            updated.append(entry)
            continue
        height, width = frame.shape[:2]
        center_x_img = width / 2.0
        center_y_img = height / 2.0
        roi = frame[int(height * 0.2) : int(height * 0.95), int(width * 0.1) : int(width * 0.9)]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        _, mask = cv2.threshold(blur, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best_score = -1.0
        best_center: tuple[float, float] | None = None
        offset_x = int(width * 0.1)
        offset_y = int(height * 0.2)
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 400 or area > width * height * 0.25:
                continue
            x, y, w, h = cv2.boundingRect(contour)
            aspect = w / max(h, 1)
            if aspect < 0.35 or aspect > 2.8:
                continue
            moments = cv2.moments(contour)
            if moments["m00"] <= 0:
                continue
            cx = moments["m10"] / moments["m00"] + offset_x
            cy = moments["m01"] / moments["m00"] + offset_y
            dist = ((cx - center_x_img) ** 2 + (cy - center_y_img) ** 2) ** 0.5
            compactness = area / max(w * h, 1)
            score = compactness - dist / max(width, height)
            if score > best_score:
                best_score = score
                best_center = (cx / width, cy / height)
        if best_center is None:
            updated.append(entry)
            continue
        updated.append(
            GroundTruthEntry(
                image=entry.image,
                class_id=entry.class_id,
                class_name=entry.class_name,
                center_x=min(1.0, max(0.0, best_center[0])),
                center_y=min(1.0, max(0.0, best_center[1])),
                annotated=True,
            )
        )
    return updated


def refine_centers_with_model(
    entries: list[GroundTruthEntry],
    *,
    dataset_dir: Path = DATASET_DIR,
    model_path: Path,
    confidence: float = 0.25,
    image_size: int = 320,
) -> list[GroundTruthEntry]:
    """Use a trusted model's correct-class detections to refine center labels."""
    from block_detected_for_pi.core import TargetingCore

    refined: list[GroundTruthEntry] = []
    with TargetingCore(model_path, image_size=image_size, confidence=confidence) as core:
        for entry in entries:
            frame = cv2.imread(str(dataset_dir / entry.image))
            if frame is None:
                refined.append(entry)
                continue
            target = core.process(frame)
            if target.found and target.class_id == entry.class_id and target.confidence >= confidence:
                refined.append(
                    GroundTruthEntry(
                        image=entry.image,
                        class_id=entry.class_id,
                        class_name=entry.class_name,
                        center_x=target.x,
                        center_y=target.y,
                        annotated=True,
                    )
                )
            else:
                refined.append(entry)
    return refined


def validate_ground_truth(
    entries: list[GroundTruthEntry],
    *,
    dataset_dir: Path = DATASET_DIR,
    require_annotated: bool = True,
) -> list[str]:
    errors: list[str] = []
    images = list_dataset_images(dataset_dir)
    if len(images) != 108:
        errors.append(f"expected 108 images, found {len(images)}")
    by_name = {entry.image: entry for entry in entries}
    for image_path in images:
        name = image_path.name
        if name not in by_name:
            errors.append(f"missing ground truth row for {name}")
            continue
        entry = by_name[name]
        expected_id = expected_class_id(name)
        if entry.class_id != expected_id:
            errors.append(f"{name}: class_id {entry.class_id} != expected {expected_id}")
        if entry.class_name != str(expected_id + 1):
            errors.append(f"{name}: class_name {entry.class_name} != expected {expected_id + 1}")
        if require_annotated and not entry.annotated:
            errors.append(f"{name}: center not annotated")
        if not 0.0 <= entry.center_x <= 1.0 or not 0.0 <= entry.center_y <= 1.0:
            errors.append(f"{name}: center out of range")
        if cv2.imread(str(image_path)) is None:
            errors.append(f"{name}: unreadable image")
    extra = set(by_name) - {path.name for path in images}
    for name in sorted(extra):
        errors.append(f"unknown ground truth row: {name}")
    return errors
