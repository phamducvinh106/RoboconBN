from __future__ import annotations

import unittest
from unittest import mock

from block_detected_for_pi.model_benchmark.ground_truth import (
    expected_class_id,
    validate_ground_truth,
    GroundTruthEntry,
)
from block_detected_for_pi.model_benchmark.metrics import (
    BenchmarkGate,
    ModelMetrics,
    apply_gate,
    compute_metrics,
    select_pareto_winner,
)
from block_detected_for_pi.onnx_pose import validate_model_schema


class GroundTruthTests(unittest.TestCase):
    def test_class_ranges(self) -> None:
        self.assertEqual(expected_class_id("dt1.jpg"), 2)
        self.assertEqual(expected_class_id("dt33.jpg"), 2)
        self.assertEqual(expected_class_id("dt34.jpg"), 3)
        self.assertEqual(expected_class_id("dt60.jpg"), 3)
        self.assertEqual(expected_class_id("dt61.jpg"), 1)
        self.assertEqual(expected_class_id("dt86.jpg"), 1)
        self.assertEqual(expected_class_id("dt87.jpg"), 0)
        self.assertEqual(expected_class_id("dt108.jpg"), 0)

    def test_validate_rejects_bad_class(self) -> None:
        entries = [
            GroundTruthEntry("dt1.jpg", 0, "1", 0.5, 0.5, True),
        ]
        fake_image = type("Path", (), {"name": "dt1.jpg"})()
        with mock.patch(
            "block_detected_for_pi.model_benchmark.ground_truth.list_dataset_images",
            return_value=[fake_image],
        ), mock.patch(
            "block_detected_for_pi.model_benchmark.ground_truth.cv2.imread",
            return_value=object(),
        ):
            errors = validate_ground_truth(entries, require_annotated=True)
        self.assertTrue(any("class_id" in error for error in errors))


class MetricsTests(unittest.TestCase):
    def test_compute_metrics_perfect(self) -> None:
        predictions = [(True, 1, 0.5, 0.5), (True, 2, 0.4, 0.6)]
        truths = [(1, 0.5, 0.5), (2, 0.4, 0.6)]
        top1, macro_f1, miss_rate, p50, p95, _confusion = compute_metrics(
            predictions=predictions,
            truths=truths,
        )
        self.assertEqual(top1, 1.0)
        self.assertEqual(macro_f1, 1.0)
        self.assertEqual(miss_rate, 0.0)
        self.assertAlmostEqual(p50, 0.0)
        self.assertAlmostEqual(p95, 0.0)

    def test_gate_rejects_latency(self) -> None:
        metrics = ModelMetrics(
            model="slow.onnx",
            macro_f1=0.95,
            miss_rate=0.0,
            center_p95=0.01,
            latency_p95_ms=120.0,
        )
        result = apply_gate(metrics)
        self.assertFalse(result.passed)
        self.assertTrue(any("latency_p95" in reason for reason in result.reasons))

    def test_select_pareto_winner(self) -> None:
        fast = ModelMetrics(
            model="fast.onnx",
            macro_f1=0.92,
            miss_rate=0.02,
            center_p95=0.02,
            latency_p95_ms=40.0,
            file_mib=3.0,
            peak_rss_mib=80.0,
        )
        slow = ModelMetrics(
            model="slow.onnx",
            macro_f1=0.95,
            miss_rate=0.01,
            center_p95=0.01,
            latency_p95_ms=80.0,
            file_mib=10.0,
            peak_rss_mib=120.0,
        )
        winner = select_pareto_winner([slow, fast])
        self.assertIsNotNone(winner)
        assert winner is not None
        self.assertEqual(winner.model, "fast.onnx")


class SchemaTests(unittest.TestCase):
    def test_validate_model_schema(self) -> None:
        validate_model_schema({0: "1", 1: "2", 2: "3", 3: "4"}, output_channels=8)
        with self.assertRaises(ValueError):
            validate_model_schema({0: "1"}, output_channels=8)


class OnnxBatchTests(unittest.TestCase):
    def test_fixed_batch_runs_per_frame(self) -> None:
        from types import SimpleNamespace

        from block_detected_for_pi.onnx_pose import OnnxPoseModel
        import numpy as np

        frame_a = np.zeros((480, 640, 3), dtype=np.uint8)
        frame_b = np.ones((480, 640, 3), dtype=np.uint8) * 255
        prepared = [np.zeros((3, 320, 320), dtype=np.float32)]
        contexts = [((480, 640), ((1.0, 1.0), (0, 0)))]
        model = mock.Mock(spec=OnnxPoseModel)
        model.input_spec = SimpleNamespace(batch_fixed=1)
        model._prepare_frames = mock.Mock(return_value=(prepared, contexts, (320, 320)))
        model._run_prepared_batch = mock.Mock(return_value=[[]])
        model.predict_batch = OnnxPoseModel.predict_batch.__get__(model, OnnxPoseModel)

        model.predict_batch((frame_a, frame_b), image_size=320, confidence=0.25)
        self.assertEqual(model._run_prepared_batch.call_count, 2)


if __name__ == "__main__":
    unittest.main()
