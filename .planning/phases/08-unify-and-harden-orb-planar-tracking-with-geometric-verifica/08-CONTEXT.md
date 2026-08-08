# Phase 8 — Pi5 metadata transport (supersedes Hub ORB plans)

## Decisions

- **D-01**: Pi5 owns dual-camera capture and ONNX vision. Control Hub consumes validated USB CDC NDJSON only.
- **D-02**: `PiCdcPacket` is the single wire contract: version, heartbeat, `frame_valid`, left/right detections, Hub-local receive timestamps.
- **D-03**: Movement authorization uses fresh Hub-time frames with finite `dxPx = (x - 0.5) * frameWidth` on the left channel only.
- **D-04**: Stale, malformed, duplicate-heartbeat, disconnected, or top-level-invalid packets fail closed.

## Out of scope

- Control Hub webcams, EasyOpenCV, ORB, template matching, UART, and packed-register transport.
- Lifting state machine route logic and autonomous tuning beyond camera freshness gates.

## Evidence

- Bench OpMode: **Pi USB CDC Communication Test** (`PiCdcCommTestOpMode`)
- Offline: `:TeamCode:piCdcPacketTest`, `python -m block_detected_for_pi.test_cdc`
