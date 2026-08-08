#!/usr/bin/env python3
"""Benchmark ONNX models for accuracy and Pi 5 performance."""

from __future__ import annotations

import argparse
import csv
import json
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np

try:
    import resource
except ImportError:  # pragma: no cover - Windows
    resource = None

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from block_detected_for_pi.core import TargetingCore  # noqa: E402
from block_detected_for_pi.model_benchmark.ground_truth import (  # noqa: E402
    DEFAULT_CSV,
    GroundTruthEntry,
    load_ground_truth,
    validate_ground_truth,
)
from block_detected_for_pi.model_benchmark.metrics import (  # noqa: E402
    BenchmarkGate,
    ModelMetrics,
    apply_gate,
    compute_metrics,
    select_pareto_winner,
)
from block_detected_for_pi.onnx_pose import OnnxPoseModel  # noqa: E402


def _percentile(values: list[float], pct: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round((pct / 100.0) * (len(ordered) - 1)))))
    return ordered[index]


def _peak_rss_mib() -> float:
    if resource is None:
        return 0.0
    usage = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    if sys.platform == "win32":
        return usage / (1024 * 1024)
    return usage / 1024


def _read_temp_c() -> float | None:
    for path in (
        "/sys/class/thermal/thermal_zone0/temp",
        "/sys/devices/virtual/thermal/thermal_zone0/temp",
    ):
        candidate = Path(path)
        if candidate.is_file():
            try:
                return int(candidate.read_text(encoding="ascii").strip()) / 1000.0
            except (OSError, ValueError):
                continue
    try:
        output = subprocess.check_output(["vcgencmd", "measure_temp"], text=True, timeout=2)
        if "temp=" in output:
            return float(output.split("temp=")[1].split("'")[0])
    except (subprocess.SubprocessError, OSError, ValueError):
        return None
    return None


def _load_frames(entries: list[GroundTruthEntry], dataset_dir: Path) -> list[tuple[GroundTruthEntry, np.ndarray]]:
    loaded: list[tuple[GroundTruthEntry, np.ndarray]] = []
    for entry in entries:
        frame = cv2.imread(str(dataset_dir / entry.image))
        if frame is not None:
            loaded.append((entry, frame))
    return loaded


def benchmark_model(
    model_path: Path,
    samples: list[tuple[GroundTruthEntry, np.ndarray]],
    *,
    image_size: int,
    confidence: float,
    threads: int,
    warmup: int,
    sustain_seconds: float,
) -> ModelMetrics:
    metrics = ModelMetrics(
        model=model_path.name,
        file_mib=model_path.stat().st_size / (1024 * 1024),
    )
    load_start = time.perf_counter()
    try:
        model = OnnxPoseModel(model_path, threads=threads)
    except Exception as error:  # noqa: BLE001
        metrics.compatible = False
        metrics.error = str(error)
        return metrics
    metrics.load_ms = (time.perf_counter() - load_start) * 1000
    metrics.task = model.task
    metrics.quantization = str(model.metadata.get("quantize", model.metadata.get("half", "fp32")))
    metrics.input_shape = model.input_spec.describe()
    effective_size = model.effective_image_size(image_size)
    if effective_size != image_size:
        metrics.notes.append(f"forced image_size={effective_size}")

    predictions: list[tuple[bool, int | None, float, float]] = []
    latencies: list[float] = []

    with TargetingCore(
        model_path,
        image_size=effective_size,
        confidence=confidence,
        threads=threads,
    ) as core:
        for entry, frame in samples:
            start = time.perf_counter()
            target = core.process(frame)
            latencies.append((time.perf_counter() - start) * 1000)
            predictions.append(
                (
                    target.found,
                    target.class_id if target.found else None,
                    target.x,
                    target.y,
                )
            )

        truths = [(entry.class_id, entry.center_x, entry.center_y) for entry, _frame in samples]
        (
            metrics.top1_accuracy,
            metrics.macro_f1,
            metrics.miss_rate,
            metrics.center_p50,
            metrics.center_p95,
            metrics.confusion,
        ) = compute_metrics(predictions=predictions, truths=truths)

        for _ in range(warmup):
            for _entry, frame in samples[: min(3, len(samples))]:
                core.process(frame)

        bench_latencies: list[float] = []
        sustain_start = time.monotonic()
        temps: list[float] = []
        iterations = 0
        while time.monotonic() - sustain_start < sustain_seconds:
            for _entry, frame in samples:
                start = time.perf_counter()
                core.process(frame)
                bench_latencies.append((time.perf_counter() - start) * 1000)
                iterations += 1
            temp = _read_temp_c()
            if temp is not None:
                temps.append(temp)

        metrics.latency_p50_ms = _percentile(bench_latencies, 50)
        metrics.latency_p95_ms = _percentile(bench_latencies, 95)
        elapsed = max(time.monotonic() - sustain_start, 1e-6)
        metrics.throughput_fps = iterations / elapsed
        metrics.sustain_seconds = elapsed
        metrics.peak_rss_mib = _peak_rss_mib()
        if temps:
            metrics.max_temp_c = max(temps)
            metrics.throttled = metrics.max_temp_c >= 80.0
        metrics.notes.append("no negative images in test set; false-positive rate not measured")
    return metrics


def write_reports(results: list[ModelMetrics], output_dir: Path, gate: BenchmarkGate) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    json_path = output_dir / "results.json"
    csv_path = output_dir / "results.csv"
    summary_path = output_dir / "summary.txt"

    serializable = []
    for item in results:
        gate_result = apply_gate(item, gate)
        serializable.append(
            {
                "model": item.model,
                "compatible": item.compatible,
                "error": item.error,
                "task": item.task,
                "quantization": item.quantization,
                "input_shape": item.input_shape,
                "file_mib": item.file_mib,
                "load_ms": item.load_ms,
                "top1_accuracy": item.top1_accuracy,
                "macro_f1": item.macro_f1,
                "miss_rate": item.miss_rate,
                "center_p50": item.center_p50,
                "center_p95": item.center_p95,
                "latency_p50_ms": item.latency_p50_ms,
                "latency_p95_ms": item.latency_p95_ms,
                "throughput_fps": item.throughput_fps,
                "peak_rss_mib": item.peak_rss_mib,
                "sustain_seconds": item.sustain_seconds,
                "max_temp_c": item.max_temp_c,
                "throttled": item.throttled,
                "gate_passed": gate_result.passed,
                "gate_reasons": list(gate_result.reasons),
                "notes": item.notes,
                "confusion": item.confusion,
            }
        )
    json_path.write_text(json.dumps(serializable, indent=2), encoding="utf-8")

    fieldnames = [
        "model",
        "compatible",
        "macro_f1",
        "miss_rate",
        "center_p95",
        "latency_p95_ms",
        "file_mib",
        "peak_rss_mib",
        "gate_passed",
    ]
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        for item, row in zip(results, serializable, strict=True):
            writer.writerow({key: row[key] for key in fieldnames})

    winner = select_pareto_winner(results)
    lines = ["ONNX model benchmark summary", ""]
    for item in results:
        gate_result = apply_gate(item, gate)
        status = "PASS" if gate_result.passed else "FAIL"
        lines.append(
            f"{item.model}: {status} "
            f"f1={item.macro_f1:.3f} miss={item.miss_rate:.3f} "
            f"center_p95={item.center_p95:.3f} latency_p95={item.latency_p95_ms:.1f}ms "
            f"size={item.file_mib:.2f}MiB"
        )
        if gate_result.reasons:
            lines.append(f"  reasons: {', '.join(gate_result.reasons)}")
    lines.append("")
    lines.append(f"winner: {winner.model if winner else 'none'}")
    summary_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(summary_path.read_text(encoding="utf-8"), end="")


def main() -> int:
    parser = argparse.ArgumentParser(description="Benchmark ONNX models against block_dataset")
    parser.add_argument("--models-dir", type=Path, default=ROOT / "block_detected_for_pi" / "models")
    parser.add_argument("--dataset", type=Path, default=DEFAULT_CSV.parent)
    parser.add_argument("--csv", type=Path, default=DEFAULT_CSV)
    parser.add_argument("--output", type=Path, default=ROOT / "artifacts" / "model-benchmark")
    parser.add_argument("--image-size", type=int, default=320)
    parser.add_argument("--confidence", type=float, default=0.25)
    parser.add_argument("--threads", type=int, default=4)
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument("--sustain-seconds", type=float, default=10.0)
    parser.add_argument("--model", action="append", help="limit to specific model file names")
    args = parser.parse_args()

    entries = load_ground_truth(args.csv)
    errors = validate_ground_truth(entries, dataset_dir=args.dataset, require_annotated=True)
    if errors:
        print("ground truth invalid:")
        for error in errors[:10]:
            print(f"  - {error}")
        return 1

    samples = _load_frames(entries, args.dataset)
    model_paths = sorted(args.models_dir.glob("*.onnx"))
    if args.model:
        allowed = {name.casefold() for name in args.model}
        model_paths = [path for path in model_paths if path.name.casefold() in allowed]

    gate = BenchmarkGate()
    results: list[ModelMetrics] = []
    for model_path in model_paths:
        print(f"==> {model_path.name}")
        try:
            results.append(
                benchmark_model(
                    model_path,
                    samples,
                    image_size=args.image_size,
                    confidence=args.confidence,
                    threads=args.threads,
                    warmup=args.warmup,
                    sustain_seconds=args.sustain_seconds,
                )
            )
        except Exception as error:  # noqa: BLE001
            results.append(
                ModelMetrics(
                    model=model_path.name,
                    compatible=False,
                    error=str(error),
                    file_mib=model_path.stat().st_size / (1024 * 1024),
                )
            )

    write_reports(results, args.output, gate)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
