# ONNX model selection

Benchmark harness for choosing the lightest accurate model on Pi 5.

## Test set

- Images: `block_dataset/dt1.jpg` … `dt108.jpg` (test only, not for training)
- Class labels by image index:
  - `dt1–33` → class `3`
  - `dt34–60` → class `4`
  - `dt61–86` → class `2`
  - `dt87–108` → class `1`
- Centers: edit `ground_truth.csv` with `tools/annotate_block_centers.py`

## Commands

```bash
# bootstrap + refine centers (initial pass)
python tools/annotate_block_centers.py --bootstrap \
  --refine-model block_detected_for_pi/models/pose11-fp16.onnx

# benchmark all ONNX models
python tools/benchmark_models.py --sustain-seconds 10

# Pi sustained check on finalist
python tools/benchmark_models.py --model pose11-fp16.onnx --sustain-seconds 240
```

## Gate (balanced)

- macro F1 ≥ 0.90
- miss rate ≤ 5%
- center error P95 ≤ 3% frame width
- inference P95 ≤ 60 ms

## Current winner

`pose11-fp16.onnx` — smallest compatible dynamic INT8 model, fastest on dev bench, passes balanced gate.

Reports are written to `artifacts/model-benchmark/` (gitignored).
