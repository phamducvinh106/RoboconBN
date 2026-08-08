from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(frozen=True, slots=True)
class BenchmarkGate:
    macro_f1_min: float = 0.90
    miss_rate_max: float = 0.05
    center_p95_max: float = 0.03
    latency_p95_max_ms: float = 60.0


@dataclass(slots=True)
class ModelMetrics:
    model: str
    compatible: bool = True
    error: str = ""
    task: str = ""
    quantization: str = ""
    input_shape: str = ""
    file_mib: float = 0.0
    load_ms: float = 0.0
    top1_accuracy: float = 0.0
    macro_f1: float = 0.0
    miss_rate: float = 0.0
    center_p50: float = 0.0
    center_p95: float = 0.0
    latency_p50_ms: float = 0.0
    latency_p95_ms: float = 0.0
    throughput_fps: float = 0.0
    peak_rss_mib: float = 0.0
    sustain_seconds: float = 0.0
    max_temp_c: float | None = None
    throttled: bool = False
    confusion: dict[str, int] = field(default_factory=dict)
    notes: list[str] = field(default_factory=list)


@dataclass(frozen=True, slots=True)
class GateResult:
    passed: bool
    reasons: tuple[str, ...]


def _percentile(values: list[float], pct: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, int(round((pct / 100.0) * (len(ordered) - 1)))))
    return ordered[index]


def compute_metrics(
    *,
    predictions: list[tuple[bool, int | None, float, float]],
    truths: list[tuple[int, float, float]],
) -> tuple[float, float, float, float, float, dict[str, int]]:
    """Return top1, macro_f1, miss_rate, center_p50, center_p95, confusion."""
    labels = sorted({truth[0] for truth in truths})
    confusion: dict[str, int] = {}
    tp = {label: 0 for label in labels}
    fp = {label: 0 for label in labels}
    fn = {label: 0 for label in labels}
    correct = 0
    misses = 0
    center_errors: list[float] = []

    for (found, pred_class, pred_x, pred_y), (truth_class, truth_x, truth_y) in zip(
        predictions, truths, strict=True
    ):
        if not found:
            misses += 1
            fn[truth_class] += 1
            confusion[f"miss->{truth_class}"] = confusion.get(f"miss->{truth_class}", 0) + 1
            continue
        assert pred_class is not None
        key = f"{pred_class}->{truth_class}"
        confusion[key] = confusion.get(key, 0) + 1
        if pred_class == truth_class:
            correct += 1
            tp[truth_class] += 1
            center_errors.append(((pred_x - truth_x) ** 2 + (pred_y - truth_y) ** 2) ** 0.5)
        else:
            fp[pred_class] += 1
            fn[truth_class] += 1

    total = len(truths)
    top1 = correct / total if total else 0.0
    miss_rate = misses / total if total else 0.0
    f1_scores: list[float] = []
    for label in labels:
        precision = tp[label] / (tp[label] + fp[label]) if (tp[label] + fp[label]) else 0.0
        recall = tp[label] / (tp[label] + fn[label]) if (tp[label] + fn[label]) else 0.0
        if precision + recall == 0:
            f1_scores.append(0.0)
        else:
            f1_scores.append(2 * precision * recall / (precision + recall))
    macro_f1 = sum(f1_scores) / len(f1_scores) if f1_scores else 0.0
    return (
        top1,
        macro_f1,
        miss_rate,
        _percentile(center_errors, 50),
        _percentile(center_errors, 95),
        confusion,
    )


def apply_gate(metrics: ModelMetrics, gate: BenchmarkGate = BenchmarkGate()) -> GateResult:
    reasons: list[str] = []
    if not metrics.compatible:
        reasons.append(metrics.error or "incompatible model")
    if metrics.macro_f1 < gate.macro_f1_min:
        reasons.append(f"macro_f1 {metrics.macro_f1:.3f} < {gate.macro_f1_min}")
    if metrics.miss_rate > gate.miss_rate_max:
        reasons.append(f"miss_rate {metrics.miss_rate:.3f} > {gate.miss_rate_max}")
    if metrics.center_p95 > gate.center_p95_max:
        reasons.append(f"center_p95 {metrics.center_p95:.3f} > {gate.center_p95_max}")
    if metrics.latency_p95_ms > gate.latency_p95_max_ms:
        reasons.append(f"latency_p95 {metrics.latency_p95_ms:.1f}ms > {gate.latency_p95_max_ms}ms")
    return GateResult(passed=not reasons, reasons=tuple(reasons))


def select_pareto_winner(candidates: list[ModelMetrics]) -> ModelMetrics | None:
    passing = [item for item in candidates if apply_gate(item).passed]
    if not passing:
        return None
    return min(
        passing,
        key=lambda item: (
            item.latency_p95_ms,
            item.peak_rss_mib,
            item.file_mib,
            -item.macro_f1,
        ),
    )
