# Phase 2 Context: Lifting Sequence State Machine

## Mục tiêu

Xây dựng state machine điều khiển một chu kỳ lấy và đặt hai block trên một kệ/tầng, sau đó lặp cho 3 kệ × 2 tầng. State machine điều phối drive, Localizer, stepper elevator, hai servo fork, hai IR và kết quả camera; không để camera hoặc stepper tự ý điều khiển ngoài state hiện tại.

## Phạm vi phần cứng

| Thành phần | Tên | Vai trò |
|---|---|---|
| Drive | `leftfront`, `leftback`, `rightfront`, `rightback` | Di chuyển đến kệ, căn ngang chậm, tiến/lùi, đi factory |
| Odometry | `leftfront` forward, `rightfront` strafe | Đọc pose robot; pose sau hai IR xác nhận là mốc kệ |
| Elevator | `step`, `dir` | Xung bước và hướng nâng/hạ |
| Home | `endstop1` | Xác nhận `HOME`, reset logical step position |
| Fork | `servoLeft`, `servoRight` | `PLACE` lấy/đặt; `HOLD` giữ block khi đi xa |
| Presence | `leftIR`, `rightIR` | Xác nhận block đã vào vị trí; không phân loại |
| Camera | `webcam1`, `webcam2` | `webcam1` căn block trái; camera classification lưu loại hai block theo Phase 3 |

## State machine tổng thể

```text
START
→ HOMING
→ SET_PLACE
→ MOVE_TO_SHELF
→ SELECT_LEVEL
→ READY1 / READY2
→ SCAN_RIGHT
→ SCAN_LEFT
→ CENTER_LEFT_SLOW
→ APPROACH_IR_SLOW
→ CONFIRM_IR
→ SAVE_SHELF_POSE
→ CALIBRATE_SHELF_COORDINATE
→ LIFT1 / LIFT2
→ BACK_OUT_FROM_SHELF
→ HOLD
→ PLACE_LEFT_SEQUENCE
→ PLACE_RIGHT_SEQUENCE
→ CYCLE_COMPLETE
```

Outer loop:

```text
for shelf = 1..3:
    for level = 1..2:
        run pickup + left placement + right placement
```

Không chuyển kệ/tầng nếu chu kỳ hiện tại chưa đạt `CYCLE_COMPLETE`. Tổng: 6 lượt, 12 block.

## Height states

- `HOME`: elevator hạ đủ thấp để block chạm đất và robot lùi an toàn; `endstop1` xác nhận.
- `READY1`: độ cao lấy/đặt tầng 1; cũng dùng làm độ cao đẩy block đã đặt sâu hơn.
- `READY2`: độ cao lấy/đặt tầng 2.
- `LIFT1`: nâng block tầng 1 rời mặt kệ vài cm.
- `LIFT2`: nâng block tầng 2 rời mặt kệ vài cm.

Stepper chỉ đổi `dir` khi không phát xung. Mỗi lệnh có giới hạn bước và stop request.

## Pickup sequence

```text
START
→ HOMING
→ SET_PLACE
→ MOVE_TO_SHELF
→ READY1 hoặc READY2
→ SCAN_RIGHT
→ SCAN_LEFT
→ CENTER_LEFT_SLOW
→ APPROACH_IR_SLOW
→ CONFIRM_IR
→ SAVE_SHELF_POSE
→ CALIBRATE_SHELF_COORDINATE
→ LIFT1 hoặc LIFT2
→ BACK_OUT_FROM_SHELF
→ HOLD
```

### Căn ngang

`CENTER_LEFT_SLOW` luôn dùng công suất thấp, không dùng lệnh strafe nhanh. Chỉ `webcam1` điều khiển căn ngang qua `dxPx`; camera phải không điều khiển drive.

Điều kiện hoàn tất:

- detection hợp lệ, ổn định đủ frame, chưa stale;
- `abs(dxPx)` trong deadband;
- drive dừng ổn định.

Mất target hoặc stale result: dừng drive. Recovery policy remains deferred; explicit stop or invalid safety conditions may enter `SAFE_STOP`.

### Tiến vào kệ

`APPROACH_IR_SLOW` tiến rất chậm. Khi một IR active, tiếp tục chậm; chỉ dừng khi cả hai active. `CONFIRM_IR` yêu cầu cả hai active liên tục trong cửa sổ debounce.

### Pose calibration tại kệ

Sau khi hai IR được xác nhận:

1. Dừng drive.
2. Gọi `Localizer.update()` một lần cuối.
3. Đọc `x`, `y`, `headingDeg`.
4. Lưu pose hiện tại làm `shelfPose[shelf][level]`.
5. Dùng pose này làm mốc hiệu chỉnh tọa độ robot theo kệ cho back-out và các chuyển động tiếp theo.
6. Chỉ sau đó mới lift.

Pose tối thiểu:

```text
shelfIndex, level, xCm, yCm, headingDeg, timestampMs
```

Không lưu pose khi chỉ một IR active, camera stale, pose không hữu hạn hoặc heading vượt giới hạn.

## Placement sequence — block trái rồi block phải

Hai block dùng cùng sequence, chạy tuần tự. Block trái luôn hoàn tất trước block phải.

```text
LIFT1 / LIFT2
→ BACK_OUT
→ HOLD
→ MOVE_NEAR_FACTORY
→ PLACE
→ READY1
→ MOVE_TO_PLACEMENT_POSITION
→ HOME
→ BLOCK_RELEASED
→ BACK_OUT_AFTER_RELEASE_20CM
```

### Ý nghĩa

- `BACK_OUT`: rời kệ sau pickup.
- `HOLD`: servo vuông góc mặt đất, giữ block khi đi xa.
- `MOVE_NEAR_FACTORY`: đi đến gần factory, servo vẫn `HOLD`.
- `PLACE`: dừng robot, servo chuyển song song mặt đất.
- `READY1`: hạ elevator đến độ cao đặt/đẩy thấp, chưa về sàn.
- `MOVE_TO_PLACEMENT_POSITION`: đi chậm đến pose đặt chính xác.
- `HOME`: robot đã đúng pose; hạ stepper đến endstop, block chạm factory và rời fork.
- `BLOCK_RELEASED`: xác nhận logical release.
- `BACK_OUT_AFTER_RELEASE_20CM`: lùi chậm 20 cm để fork rời hoàn toàn block. Dùng pose bắt đầu state + Localizer; pose không đổi hoặc invalid safety input requires stop; recovery policy remains deferred.

Block trái:

```text
... → HOLD_LEFT
→ MOVE_NEAR_FACTORY_LEFT
→ PLACE_LEFT
→ READY1_LEFT
→ MOVE_TO_PLACEMENT_LEFT
→ HOME_LEFT
→ LEFT_BLOCK_RELEASED
→ BACK_OUT_AFTER_LEFT_RELEASE_20CM
```

Block phải:

```text
... → HOLD_RIGHT
→ MOVE_NEAR_FACTORY_RIGHT
→ PLACE_RIGHT
→ READY1_RIGHT
→ MOVE_TO_PLACEMENT_RIGHT
→ HOME_RIGHT
→ RIGHT_BLOCK_RELEASED
→ BACK_OUT_AFTER_RIGHT_RELEASE_20CM
```

Nếu block phải cũng phải chạy y hệt block trái, sau khi đặt block trái phải thực hiện lại phần nâng cần thiết theo tầng trước `BACK_OUT_RIGHT`; không giả định elevator còn ở `LIFT1/LIFT2` sau `HOME_LEFT`.

## Factory routing

- `leftBlockType` chọn factory trái.
- `rightBlockType` chọn factory phải.
- Factory pose là tọa độ cố định; không suy ra loại block từ IR.
- Mapping block → factory phải được cấu hình riêng, không hard-code rải trong state machine.

## Error handling và recovery

### Deferred Error Handling

Recovery policy, retries, no-progress policy, stale input policy, and `SAFE_STOP` transitions require a later discuss-phase decision. This phase preserves encoder arrival gates, fresh/finite pose, target tolerance, settle cycles, no-progress detection, JSON-only tuning, and `SAFE_STOP` only for explicit stop or invalid safety conditions; it does not define replacement recovery behavior.

### Nguyên tắc chung

Mọi state có chuyển động phải có:

- kiểm tra `isStopRequested()` và `opModeIsActive()` mỗi vòng;
- dừng drive và ngừng step pulses khi thoát;
- telemetry state, lỗi, shelf, level, pose, target;
- không tự chuyển state khi điều kiện bắt buộc chưa đạt.

### Bảng lỗi

| Lỗi | Phát hiện | Xử lý |
|---|---|---|
| `HOMING_TIMEOUT` | Endstop không active trước giới hạn | Ngừng step, drive stop, `SAFE_STOP` |
| `ELEVATOR_TIMEOUT` | Không đạt height/bước | Ngừng step, giữ servo an toàn, `SAFE_STOP` |
| `CAMERA_STALE` | Quá tuổi result | Dừng strafe/tiến, retry scan hữu hạn |
| `CAMERA_CLASSIFICATION_FAILED` | Label thiếu/confidence thấp/label nhảy | Không tiến vào kệ; retry rồi `SAFE_STOP` |
| `CENTERING_TIMEOUT` | Không đạt deadband | Drive stop, retry tại chỗ hoặc `SAFE_STOP` |
| `IR_PARTIAL` | Chỉ một IR active | Tiếp tục tiến rất chậm trong giới hạn; không lift |
| `IR_TIMEOUT` | Hai IR không active/debounce fail | Dừng tiến, không lift, `SAFE_STOP` hoặc retry lùi |
| `POSE_INVALID` | Pose NaN/infinite hoặc heading bất thường | Không lưu calibration; `SAFE_STOP` |
| `LIFT_TIMEOUT` | Không đạt `LIFT1/LIFT2` | Ngừng step, không back-out, `SAFE_STOP` |
| `BACK_OUT_TIMEOUT` | Không lùi đạt pose | Drive stop, `SAFE_STOP` |
| `FACTORY_MOVE_TIMEOUT` | Không đạt pose factory | Drive stop, giữ block, `SAFE_STOP` |
| `PLACEMENT_TIMEOUT` | Không đạt pose đặt/endstop | Drive/step stop, `SAFE_STOP` |
| `RELEASE_UNCONFIRMED` | Không chắc block rời fork | Không tiến chu kỳ tiếp; operator recovery |
| `BACK_OUT_AFTER_RELEASE_TIMEOUT` | Không lùi đủ 20 cm | Drive stop, `SAFE_STOP` |
| `STOP_REQUESTED` | FTC stop request | Dừng mọi cơ cấu trong `finally` |

### SAFE_STOP

- drive power = 0;
- ngừng phát `step`;
- không đổi `dir` khi stepper đang chạy;
- servo giữ trạng thái cơ khí an toàn;
- ghi nguyên nhân và vị trí `shelf/level/state`;
- không tự resume; operator reset toàn chu kỳ.

## Invariants

1. `HOMING` thành công trước mọi lệnh elevator.
2. Chưa đạt `READY1/READY2` không được scan/tiến.
3. Camera stale không được căn hoặc điều khiển drive.
4. Căn ngang và tiến vào kệ đều chậm.
5. Chỉ hai IR đã debounce mới cho phép lift.
6. Pose kệ chỉ lưu sau khi robot dừng và hai IR xác nhận.
7. `LIFT` xong mới `BACK_OUT`; `BACK_OUT` xong mới `HOLD`.
8. `PLACE` trước `READY1`; `READY1` trước `MOVE_TO_PLACEMENT_POSITION`; `HOME` chỉ sau khi đạt pose đặt.
9. Block trái release xong mới xử lý block phải.
10. Sau mỗi `HOME` placement, lùi 20 cm trước state kế tiếp.
11. Chưa `CYCLE_COMPLETE` không được sang shelf/level tiếp theo.
12. Mọi lỗi bắt buộc dẫn đến retry hữu hạn hoặc `SAFE_STOP`.

## Open decisions for error-handling discussion

1. Retry camera tại chỗ hay lùi về pose scan sau `CAMERA_STALE`?
2. `IR_PARTIAL` được phép tiến tối đa bao nhiêu cm trước khi abort?
3. Khi `RELEASE_UNCONFIRMED`, giữ block hay lùi 20 cm rồi chờ operator?
4. Sau `SAFE_STOP`, chỉ cho reset toàn chu kỳ hay cho resume tại `shelf/level` sau kiểm tra thủ công?
5. Pose factory cố định có cần giới hạn sai số X/Y/heading riêng không?
6. `BACK_OUT_AFTER_RELEASE_20CM` dùng tolerance bao nhiêu cm và có cần xác nhận fork đã rời block bằng cảm biến không?
7. Khi một shelf/level lỗi, bỏ lượt đó hay dừng toàn bộ task?

## Decisions to preserve

- Hai block dùng cùng placement sequence.
- Block trái đặt trước block phải.
- `READY1` nằm sau `PLACE` và trước `MOVE_TO_PLACEMENT_POSITION`.
- `HOME` chỉ chạy khi robot đã ở đúng pose đặt.
- Camera chỉ cung cấp detection; state machine/drive mới phát lệnh chuyển động.
- Không thêm dependency; giữ FTC SDK/OpenCV hiện có.
- Phase 2 không triển khai OpenCV/template matching; chỉ giữ camera output contract để Phase 3 thực hiện vision.
- Hai camera là hai channel/identity tường minh: `webcam1` giữ role căn block trái, `webcam2` giữ role camera còn lại; không fallback hoặc gộp identity.
- Camera nhận dữ liệu từ Raspberry Pi 5 qua một I2C device duy nhất, địa chỉ 7-bit 0x42, được FTC Control Hub nhận bằng HardwareMap với cấu hình tên `pi5Camera`. Device manager giữ hai logical channels `webcam1` và `webcam2` qua channel-select register; frame format/parser deferred; Phase 2 chỉ tạo placeholder transport/frame contract. Android/FTC I2C API phải được kiểm tra theo SDK pin của repository trước khi chốt implementation.
- Placeholder data invalid/incomplete không được authorize movement; phải expose validity và freshness.

## Locked configuration decision

- **D-01 — External runtime configuration:** Every runtime-tunable or configurable lifting parameter MUST come from one versioned external JSON document loaded from FTC app assets or the repository's equivalent existing resource pattern. This includes servo `PLACE`/`HOLD` positions; step/dir pulse timing; elevator heights and travel bounds; endstop/IR polarity and debounce; stale, timeout, retry, and no-progress limits; camera thresholds, freshness, and centering policy; drive speeds; pose tolerances; factory coordinates; release confirmation; and the 20 cm back-out tolerance. Java may retain only safety-critical structural constants, enum/state names, and immutable protocol identifiers. JSON validation is strict: required fields, schema/version, finite numbers, and bounded ranges; missing or invalid configuration enters `SAFE_STOP` with no silent defaults. The runtime reports loaded config version and a short non-sensitive hash/fingerprint. Calibration workflow, fixtures, and hardware validation must use the same JSON contract.
- **D-02 — Encoder-confirmed movement transitions:** Before every transition involving movement, the state machine MUST confirm fresh, finite, valid `Localizer`/odometry state and encoder progress. The gate requires target tolerance, measured position and heading error within configured bounds, and the configured minimum number of settle cycles. It MUST detect no progress, invalid encoders, stale odometry, non-finite pose, and excessive position/heading error. Applies explicitly to `MOVE_TO_SHELF`, `CENTER`, `APPROACH`, `BACK_OUT`, `MOVE_NEAR_FACTORY`, `MOVE_TO_PLACEMENT_POSITION`, and `BACK_OUT_AFTER_RELEASE`; camera, IR, and stepper gates remain required. All motion tuning and encoder-gate values stay in D-01 JSON. Invalid or stale encoder state gets bounded retry or `SAFE_STOP`, never an unconfirmed transition.

- **D-03 — Pi5 I2C camera boundary:** Camera data arrives from Raspberry Pi 5 through one I2C device at 7-bit address 0x42, configured in FTC HardwareMap as `pi5Camera`. Phase 2 defines a device-manager seam only; SDK API verification is required against the pinned FTC SDK, while frame format/parser remain deferred.
- **D-04 — Explicit camera channels:** Preserve two explicit camera channels and identities: `webcam1` remains left-centering logical role and `webcam2` remains second-camera logical role. No fallback or identity merge.
- **D-05 — Deferred vision implementation:** Phase 2 does not implement OpenCV, template matching, candidate ranking, NMS, or classification. Phase 3 owns vision implementation.
- **D-06 — Invalid placeholder safety:** Placeholder frames expose validity and freshness. Invalid, incomplete, or stale data cannot authorize centering, approach, lift, or movement transitions.
- **D-07 — Hardware-first camera communication:** I2C device detection and manager communication test runs before JSON config/state integration. Bench test must verify HardwareMap recognition, address 0x42, register read/write, heartbeat, and both logical channels. Keep encoder-confirmed transitions, no timeout, JSON tuning, separate manager classes, and deferred error handling.
- **D-08 — Pi5 logical payload:** Treat user payload as packed 20-bit logical value, not 20-byte data and not yet a physical UART/I2C frame. Bits 0-7 are unsigned X center (`X_MASK = 0x000FF`, shift 0), bits 8-15 are unsigned Y center (`Y_MASK = 0x0FF00`, shift 8), bits 16-17 are left-camera block code (`LEFT_TYPE_MASK = 0x30000`, shift 16), and bits 18-19 are right-camera block code (`RIGHT_TYPE_MASK = 0xC0000`, shift 18). Decode only after complete/fresh validity is established; codes 0..3 map configurably to block types 01..04, with JSON mapping when configuration already requires it. Reserved/invalid codes and stale or partial reads cannot authorize movement. Physical framing, endian order, register layout, checksum, sentinel policy, screen dimensions, axis origin, and atomic I2C read remain open/deferred.
