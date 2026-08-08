"""Model benchmark utilities for ONNX selection on Pi 5."""

from block_detected_for_pi.model_benchmark.ground_truth import (
    GroundTruthEntry,
    bootstrap_centers,
    expected_class_id,
    load_ground_truth,
    refine_centers_with_model,
    save_ground_truth,
    validate_ground_truth,
)
from block_detected_for_pi.model_benchmark.metrics import (
    BenchmarkGate,
    GateResult,
    ModelMetrics,
    apply_gate,
    compute_metrics,
    select_pareto_winner,
)

__all__ = [
    "BenchmarkGate",
    "GateResult",
    "GroundTruthEntry",
    "ModelMetrics",
    "apply_gate",
    "bootstrap_centers",
    "compute_metrics",
    "expected_class_id",
    "load_ground_truth",
    "save_ground_truth",
    "select_pareto_winner",
    "validate_ground_truth",
]
