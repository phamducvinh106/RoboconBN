# RoboconBN — Robotics Sóc Sơn × Robotics Bắc Ninh

> Code cho robot tự chạy bảng O2 RBC BNMR 2026 — *Khát vọng công nghệ*  
> Bạn bấm **Start**. Robot lo phần còn lại. *(Lý thuyết thế. Thực tế thì… calibrate đi.)*

[![FTC](https://img.shields.io/badge/FTC-Robot%20Controller-blue)](https://ftc-docs.firstinspires.org/)
[![Platform](https://img.shields.io/badge/Platform-Android-green)](https://developer.android.com/)
[![Language](https://img.shields.io/badge/Language-Java-orange)](https://www.java.com/)
[![Competition](https://img.shields.io/badge/Competition-RBC%20BNMR%202026-red)](#nhiệm-vụ--điểm-số)

**Không có internet trên sân. Không có "ê mentor cầm hộ". Chỉ có OpenCV, odometry, và niềm tin.**

---

## Đây là cái gì?

**Robotics Sóc Sơn** ngồi code cho con robot của **Robotics Bắc Ninh** thi bảng O2 tại *Robocon Bắc Ninh mở rộng 2026* (Tranh Cúp Foxconn).

Robot chạy **hoàn toàn tự động** sau khi bấm Start. Cấm chạm tay. Cấm lái hộ. Trọng tài nhìn chằm chằm lắm đấy.

Xây trên [FTC Robot Controller SDK](https://github.com/FIRST-Tech-Challenge/FtcRobotController) — tức là Java chạy trên Android, không phải ChatGPT cầm tay lái.

---

## Nhiệm vụ & điểm số

| | |
|---|---|
| **Thời gian** | Tối đa 240 giây — hết giờ thì về nhà, không overtime |
| **Nhiệm vụ 1** | Nhận diện, gắp, giao **12 kiện hàng** đúng nhà máy (20 điểm/kiện, max 240) |
| **Nhiệm vụ 2** | Sau khi xong 100% nhiệm vụ 1: giao kiện tổng hợp từ kho rời → nhà máy liên hợp (30 điểm) |
| **Tổng tối đa** | 270 điểm (trước khi trừ reset) |
| **Reset** | Tối đa 5 lần, mỗi lần −10 điểm. Kiện đã đặt đúng thì vẫn giữ nguyên |

### Kiện đi đâu?

| Kiện | Nhà máy | Ghi chú |
|------|---------|---------|
| 01 | Samsung | Đừng giao nhầm sang Foxconn |
| 02 | Foxconn | Đừng giao nhầm sang Samsung |
| 03 | Amkor | |
| 04 | Hana Micron Vina | |

Robot không đổi lỗi được. Code đúng từ đầu.

---

## Robot nhìn từ trên xuống (ảo thôi)

```
┌─────────────────────────────────────────────────────────┐
│  REV Control Hub — não bộ, không được đập              │
│  ┌─────────────┐  ┌──────────────┐  ┌─────────────────┐ │
│  │ Mecanum ×4  │  │ goBILDA 4-bar│  │ Thang máy step  │ │
│  │ lăn lung   │  │ odometry     │  │ + fork 2 lưỡi   │ │
│  │ tung tăng  │  │ (đếm mét)    │  │ (gắp hàng)      │ │
│  └─────────────┘  └──────────────┘  └─────────────────┘ │
│  ┌─────────────┐  ┌──────────────┐  ┌─────────────────┐ │
│  │ webcam1     │  │ leftIR       │  │ rightIR         │ │
│  │ mắt thần    │  │ "có hàng chưa│  │ "có hàng chưa"  │ │
│  │ OpenCV      │  │  ở đây chưa?" │  │                 │ │
│  └─────────────┘  └──────────────┘  └─────────────────┘ │
└─────────────────────────────────────────────────────────┘
```

### Bảng tên phần cứng

Tên device **phân biệt hoa thường**. Đặt sai một chữ → robot đứng im → bạn ngồi khóc.

| Hệ thống | Tên trong config | Làm gì |
|----------|------------------|--------|
| Bánh | `leftfront`, `leftback`, `rightfront`, `rightback` | Mecanum — lăn mọi hướng, không cần quay đầu |
| Odometry | `leftfront` (tiến), `rightfront` (ngang) | goBILDA 4-bar — biết robot đang ở đâu (hy vọng là đúng) |
| IMU | `imu` | Biết robot quay bao nhiêu độ |
| Fork | `servoLeft`, `servoRight` | `PLACE` (song song đất) / `HOLD` (vuông góc đất) |
| Thang máy | `step`, `dir`, `endstop1` | Motor bước nâng hạ + endstop homing |
| Camera | `webcam1` | Nhận diện kiện + căn giữa — chạy offline |
| IR | `leftIR`, `rightIR` | Xác nhận kiện đã vào chỗ gắp. **Không** thay camera. Camera hỏng thì đừng đổ lỗi IR |

---

## Robot đi làm việc thế nào?

Checklist cho người mới — đọc xong biết robot sẽ làm gì (nếu không bug):

1. **Homing** thang máy bằng `endstop1` — về gốc trước, hỏi sau
2. **Lăn** đến kho hải quan nhờ odometry
3. **Quét** kệ bằng `webcam1`, OpenCV đoán kiện 01–04
4. **Căn** fork vào pallet (camera + odometry), IR xác nhận "ừ ổn rồi"
5. **Luồn** fork dưới pallet, chuyển `HOLD`, nâng bằng stepper
6. **Chở** đến nhà máy đúng loại (vùng 250 × 250 mm)
7. **PLACE** → hạ → rút fork → ghi nhận xong 1 kiện
8. Lặp đến đủ **12 kiện**, rồi mới được làm nhiệm vụ 2 (nếu còn giờ)

Hết 240 giây thì dừng. Không tranh luận với đồng hồ.

---

## Code nằm ở đâu?

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── core/      # Não, cơ bắp, mắt — logic chính
├── opmode/    # Cái hiện trên Driver Station
└── test/      # Test offline — chạy không cần robot (may mắn thì pass)
```

### Module quan trọng (đừng sửa lung tung)

| Class | Nói đơn giản |
|-------|--------------|
| `RobotHardware` | Map toàn bộ phần cứng — motor, servo, step, sensor, IMU |
| `MecanumDrive` | Lái mecanum + PID `goToPosition` — "đi đến tọa độ X,Y đi" |
| `Localizer` / `TwoWheelOdometry` | Ước lượng vị trí từ encoder + IMU |
| `LiftingSequenceStateMachine` | Bộ não nhiệm vụ: `HOMING` → `SCAN` → `LIFT` → `PLACE` → … Nếu thấy `SAFE_STOP` thì đừng hỏi "sao không chạy", hỏi telemetry |
| `LiftingSequenceConfig` | Tọa độ, góc servo, timeout — load từ JSON |
| `CameraAdapterManager` | Camera có frame mới không, còn tươi không |
| `ForkServoManager` / `StepperElevatorManager` | Điều khiển fork + thang máy, có giới hạn an toàn |
| `PidController` | PID dùng chung — vặn P/I/D cho đến khi đỡ rung |

### Vision — 2 chế độ, 0 internet

| Mode | Làm gì |
|------|--------|
| `MULTI_TARGET` | Nhận diện loại kiện 01–04 ở độ cao kệ |
| `SINGLE_TARGET` | Căn giữa pallet trái nhanh + chính xác; fork phải tự căn theo khoảng cơ khí |

IR chỉ confirm "kiện đã vào chỗ". Camera mới biết kiện **là cái gì**. Đừng nhầm vai trò.

### An toàn — đọc trước khi chạy trên sân thật

| Tình huống | Robot làm gì |
|------------|--------------|
| Mất camera | Dừng nâng. **Không** đoán mò loại kiện |
| Odometry lệch nặng | Dừng. Đừng lấn sân đối phương |
| Homing fail | Không chạy nhiệm vụ. Về sửa `endstop1` |
| IR không confirm | Không nâng. Hoặc hạ tốc độ kiểm tra |
| Bất kỳ lỗi nào | `SAFE_STOP` + timeout + giới hạn tốc độ |

---

## Cấu trúc repo

```
RoboconBN/
├── FtcRobotController/     # SDK gốc FTC — đừng sửa cho vui, sửa là tự upgrade
├── TeamCode/               # ★ CODE ĐỘI — sửa ở đây
│   ├── src/main/java/.../teamcode/
│   ├── src/main/assets/    # JSON config, template ảnh vision
│   └── src/main/res/       # Manifest, calibration camera
├── robot-context.txt       # Bản tóm tắt đề O2 + robot (tiếng Việt)
├── .planning/              # Kế hoạch phase, research — đọc khi buồn ngủ
├── build.gradle
└── settings.gradle
```

---

## Cài đặt (cho người lần đầu)

### Cần có gì?

- [Android Studio](https://developer.android.com/studio) **Ladybug (2024.2)** trở lên  
  (SDK 11.2+ cần **Narwhal 3 Feature Drop** — Studio cũ sync fail thì đừng blame Gradle)
- REV Control Hub + Driver Station (điện thoại hoặc REV Driver Hub)
- Cáp USB hoặc WiFi pairing

### Clone & build

```bash
git clone https://github.com/<your-org>/RoboconBN.git
cd RoboconBN
```

Android Studio → **File → Open** → chọn thư mục gốc repo.

Deploy:

1. Chọn run config **TeamCode**
2. Cắm Control Hub
3. Bấm nút xanh **Run**
4. Cầu nguyện build thành công

### Config robot

Tạo FTC configuration khớp [bảng phần cứng](#bảng-tên-phần-cứng) ở trên. Sai tên = robot không nhúc nhích.

Tọa độ nhà máy, góc servo, timeout nằm ở:

```
TeamCode/src/main/assets/phase2-lifting-config.json
```

**Cấm bịa số trên giấy.** Calibrate góc `PLACE`/`HOLD`, steps/mm, tọa độ sân trên robot thật trước khi thi.

---

## OpModes — menu Driver Station

### Tự động (Autonomous)

| Tên hiển thị | Class | Để làm gì |
|--------------|-------|-----------|
| Lifting Sequence | `LiftingSequenceOpMode` | Chạy full state machine gắp–nâng–giao. Cái này là mục tiêu cuối |

### Hiệu chuẩn (Calibration)

| Tên hiển thị | Class | Để làm gì |
|--------------|-------|-----------|
| Odometry Calibration Clockwise | `OdometryCalibrationOpMode` | Calibrate bánh odometry — chạy vòng tròn, hy vọng ra hình tròn |
| Localizer Motion Test | `LocalizerMotionTestOpMode` | Xem pose có update không |
| PID Tuning | `PidTuningOpMode` | Vặn P/I/D tay — đến khi robot không rung như điện thoại rung chuông |
| Automatic PID Tuning | `AutomaticPidTuningOpMode` | PID tự tune trong vùng giới hạn — lười thì dùng cái này |
| Pod Sign Offset Test | `PodSignOffsetTestOpMode` | Kiểm tra encoder quay đúng chiều chưa |
| Goto Position Direction Test | `GotoPositionDirectionTestOpMode` | `goToPosition` đi đúng hướng chưa |
| Mecanum Target Gamepad Test | `MecanumDriveGamepadTargetTestOpMode` | Gamepad + target pose |

### Test / debug

| Tên hiển thị | Class | Để làm gì |
|--------------|-------|-----------|
| Camera + Odometry Modular | `CameraOdometryMain` | Debug camera + pose cùng lúc |
| Lifting Hardware Communication Test | `LiftingHardwareTestOpMode` | Test thang máy, fork, IR có nói chuyện với nhau không |
| Pi5 UART Communication Test | `Pi5UartCommTestOpMode` | **Test link Pi→Hub UART** — chỉ cần `pi5UartRx`, không cần motor/servo |
| Left Camera Centering Test | `LeftCameraCenteringTestOpMode` | Test `SINGLE_TARGET` căn giữa |
| Right Camera Sticker Classification Test | `RightCameraClassificationTestOpMode` | Test `MULTI_TARGET` nhận diện sticker |
| Left Color Centering Test | `LeftColorCenteringTestOpMode` | Căn giữa bằng màu (thử nghiệm) |
| Mecanum Drive Gamepad Test | `MecanumDriveGamepadTestOpMode` | Lái thô bằng gamepad |
| Servo Joystick Test | `ServoJoystickTestOpMode` | Vặn servo fork bằng joystick — tìm góc PLACE/HOLD |

---

## Nguyên tắc code (đọc trước khi PR)

- **Cấm bịa số** — góc servo, steps/mm, tốc độ, ngưỡng OpenCV phải calibrate trên robot thật
- **State machine trước, hack sau** — `INIT → HOME → SCAN → APPROACH → PICK → LIFT → DELIVER → PLACE → VERIFY → FAILSAFE`
- **Offline only** — không gọi API, không ChatGPT trên sân
- **Đừng rename hardware** — đổi `leftfront` thành `LeftFront` rồi hỏi "sao không chạy" là tự mình làm
- **Nhiệm vụ 1 trước, 2 sau** — chưa đủ 12 kiện thì đừng mơ đến kho rời

### Chạy test offline

```bash
./gradlew :TeamCode:test
```

Pass hết test vẫn có thể fail trên sân. Đó là robotics.

### Tài liệu thêm

- [`.planning/PROJECT.md`](.planning/PROJECT.md) — overview dự án
- [`.planning/ROADMAP.md`](.planning/ROADMAP.md) — các phase phát triển
- [`robot-context.txt`](robot-context.txt) — đề O2, hardware, chiến lược đầy đủ

---

## Luật O2 — những thứ không được phá

| Luật | Giới hạn | Ghi chú |
|------|----------|---------|
| Kích thước lúc xuất phát | 400 × 400 × 400 mm | Xuất phát xong mới được bung |
| Motor + servo | ≤ 12 | Đếm kỹ. Đừng lắp thêm cho oai |
| Cổng I/O | ≤ 16 | Control Hub có hạn mức |
| Điện | ≤ 12 V, pin ≤ 5000 mAh | |
| Khung | Không khung kim loại | Ốc vít thì được |
| Vision | Camera nối Control Hub, OpenCV local | Không PC ngoài, không internet |
| Reset | Tối đa 5 lần | Mỗi lần −10 điểm |

---

## Đội

| | |
|---|---|
| **CLB code** | Robotics Sóc Sơn |
| **Đội thi** | Robotics Bắc Ninh |
| **Giải** | RBC BNMR 2026 — Tranh Cúp Foxconn, Bảng O2 |
| **Nền tảng** | FTC Robot Controller trên REV Control Hub |

---

## Cảm ơn

- [FIRST Tech Challenge](https://www.firstinspires.org/robotics/ftc) & [FTC SDK](https://github.com/FIRST-Tech-Challenge/FtcRobotController)
- [FTC Docs](https://ftc-docs.firstinspires.org/) — đọc khi stuck
- REV Robotics, goBILDA, và cộng đồng FTC chịu khó trả lời câu hỏi 3h sáng

---

## License

Code trong `TeamCode/` do Robotics Sóc Sơn / Robotics Bắc Ninh phát triển.

Module `FtcRobotController/` tuân theo [license FTC SDK](LICENSE). Muốn biết SDK version bao nhiêu thì xem release notes upstream — đừng hỏi README này.
