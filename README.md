# RoboconBN — Robot vẽ tranh

> Robot FTC vẽ tranh bằng odometry + servo bút.  
> Bấm **Start** → robot đi theo waypoint trong `draw-path.json`.

[![FTC](https://img.shields.io/badge/FTC-Robot%20Controller-blue)](https://ftc-docs.firstinspires.org/)
[![Platform](https://img.shields.io/badge/Platform-Android-green)](https://developer.android.com/)
[![Language](https://img.shields.io/badge/Language-Java-orange)](https://www.java.com/)

---

## Mục đích

Sau mùa thi, repo được tái cấu trúc thành **robot vẽ tranh**:

- Giữ: mecanum drive, odometry (goBILDA 4-bar + IMU), PID, OpMode calibration
- Thêm: servo `penServo`, follower đường vẽ JSON, heading cố định suốt bài vẽ
- Bỏ: lift, fork, camera, Pi vision

---

## Phần cứng

```
┌─────────────────────────────────────────────────────────┐
│  REV Control Hub                                        │
│  ┌─────────────┐  ┌──────────────┐  ┌───────────────┐ │
│  │ Mecanum ×4  │  │ Odometry pods│  │ Servo penServo│ │
│  │             │  │ + IMU        │  │ (nhấc/hạ bút) │ │
│  └─────────────┘  └──────────────┘  └───────────────┘ │
└─────────────────────────────────────────────────────────┘
```

| Hệ thống | Tên trong Robot Config | Ghi chú |
|----------|------------------------|---------|
| Bánh | `leftfront`, `leftback`, `rightfront`, `rightback` | Mecanum |
| Odometry | `leftfront` (tiến), `rightfront` (ngang) | goBILDA 4-bar |
| IMU | `imu` | Heading |
| Bút | `penServo` | Servo nhấc/hạ bút |

**Robot Config trên Driver Station:** giữ 4 motor + 2 encoder pod + IMU, thêm servo `penServo`. Xóa fork, IR, stepper, camera.

---

## Cấu trúc code

```
TeamCode/src/main/java/org/firstinspires/ftc/teamcode/
├── core/
│   ├── drive/          # MecanumDrive, PidController
│   ├── odometry/       # Localizer (đơn vị cm)
│   ├── pen/            # PenServoManager, DrawPath*, RobotConfig*
│   └── RobotHardware.java
└── opmode/
    ├── DrawArtOpMode.java
    └── calibration/    # OpMode hiệu chuẩn drivetrain
```

### Assets

| File | Mô tả |
|------|-------|
| `robot-config.json` | Vị trí servo bút, power, tolerance, settle cycles |
| `draw-path.json` | Waypoint vẽ — **tọa độ mm** trong JSON, code chuyển sang **cm** |

### Ví dụ `draw-path.json`

```json
{
  "version": 1,
  "name": "demo-square",
  "startPose": { "xMm": 0, "yMm": 0, "headingDeg": 0 },
  "drawingHeadingDeg": 0,
  "waypoints": [
    { "xMm": 0, "yMm": 0, "penDown": false },
    { "xMm": 100, "yMm": 0, "penDown": true }
  ]
}
```

- `startPose`: pose tuyệt đối khi bấm START (`localizer.setPose`)
- `drawingHeadingDeg`: heading giữ suốt bài vẽ (không xoay giữa các đoạn)
- `penDown`: `false` = nhấc bút trước khi di chuyển, `true` = hạ bút vẽ

---

## OpModes

### Autonomous

| Tên | Class | Mô tả |
|-----|-------|-------|
| Draw Art | `DrawArtOpMode` | Vẽ theo `draw-path.json` |

### Calibration (`opmode/calibration/`)

| Tên | Class |
|-----|-------|
| Odometry Calibration Clockwise | `OdometryCalibrationOpMode` |
| Localizer Motion Test | `LocalizerMotionTestOpMode` |
| PID Tuning | `PidTuningOpMode` |
| Automatic PID Tuning | `AutomaticPidTuningOpMode` |
| Pod Sign Offset Test | `PodSignOffsetTestOpMode` |
| Goto Position Direction Test | `GotoPositionDirectionTestOpMode` |
| Mecanum Drive Gamepad Test | `MecanumDriveGamepadTestOpMode` |
| Mecanum Target Gamepad Test | `MecanumDriveGamepadTargetTestOpMode` |

---

## Cài đặt

```bash
git clone https://github.com/<your-org>/RoboconBN.git
cd RoboconBN
```

Android Studio → **File → Open** → chọn thư mục gốc repo → Run config **TeamCode**.

Calibrate `upPosition` / `downPosition` trong `robot-config.json` trên robot thật trước khi vẽ.

---

## Test offline

```bash
python3 run_tests.py --suite localizer-math --suite draw-path --suite mecanum --suite localizer-calibration
```

Hoặc compile Java:

```bash
./gradlew :TeamCode:compileDebugJavaWithJavac
```

---

## Lưu ý vận hành

1. **Đơn vị:** JSON dùng mm, `MecanumDrive`/`Localizer` dùng cm — loader tự chuyển đổi
2. **Heading hold:** `DrawArtOpMode` bật `HeadingHoldMode.ALWAYS` — PID heading chạy cả khi đứng yên
3. **Pen transition:** đổi `penDown` → nhấc/hạ bút **trước** khi robot di chuyển
4. Tune `headingToleranceDeg` và `headingHoldPower` trên sân nếu heading trôi

---

## License

Code trong `TeamCode/` do Robotics Sóc Sơn / Robotics Bắc Ninh phát triển.

Module `FtcRobotController/` tuân theo [license FTC SDK](LICENSE).
