package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.core.*;

/**
 * Vòng chạy tự động dùng chung cho cả đội XANH và ĐỎ.
 *
 * <p>Lớp con chỉ cần chọn màu đội qua {@link #alliance()}.
 * Lớp này sẽ đọc cấu hình, khởi tạo robot, rồi chạy máy trạng thái nâng và đặt khối.</p>
 */
public abstract class LiftingSequenceOpMode extends LinearOpMode {

    /** ponytail: tăng lên 0.20 sau khi chạy Automatic PID Tuning trên sân. */
    private static final double AUTO_DRIVE_POWER = 0.15;
    private static final long TELEMETRY_PERIOD_NS = 100_000_000L;

    /** Trả về màu đội mà OpMode hiện tại sử dụng. */
    protected abstract Alliance alliance();

    @Override
    public void runOpMode() throws InterruptedException {
        Alliance alliance = alliance();

        // Đọc vị trí trên sân và các thông số robot từ file JSON.
        // Nếu file sai hoặc thiếu, robot dừng để tránh chạy với thông số không an toàn.
        FieldBlueConfig field;
        LiftingSequenceConfig config;
        try {
            String fieldJson = RobotConfigAssets.readAsset(hardwareMap.appContext.getAssets(),
                    RobotConfigAssets.FIELD_BLUE_PATH);
            field = FieldBlueConfigLoader.load(fieldJson);
            config = RobotConfigAssets.load(hardwareMap.appContext.getAssets(), alliance);
        } catch (Exception e) {
            telemetry.addData("config", "SAFE_STOP: %s", e.getMessage());
            telemetry.update();
            return;
        }

        // Trong lúc chờ bấm START, hiện cấu hình để người vận hành kiểm tra.
        while (!isStarted() && !isStopRequested()) {
            telemetry.addData("alliance", alliance.name());
            telemetry.addData("start", "%.1f, %.1f, %.1f",
                    field.placeAtFactory.x, field.placeAtFactory.y, field.placeAtFactory.heading);
            telemetry.addData("field", field.calibrated ? "calibrated" : "NOT CALIBRATED");
            if (!field.calibrated) {
                telemetry.addLine("Edit field-blue.json with measured poses, then set calibrated=true");
            }
            telemetry.update();
            idle();
        }
        if (!opModeIsActive()) return;

        // Không cho robot chạy nếu các tọa độ sân chưa được đo và xác nhận.
        if (!field.calibrated) {
            telemetry.addLine("SAFE_STOP: field not calibrated");
            telemetry.update();
            return;
        }

        // Khởi tạo phần cứng, hệ thống định vị, bộ lái mecanum và camera Raspberry Pi.
        RobotHardware robot;
        MecanumDrive drive;
        PiCdcCameraTransport cameraTransport = null;
        try {
            robot = new RobotHardware(hardwareMap, config);
            LiftingSequenceConfig.Pose startPose = field.placeAtFactory;
            robot.localizer.setPose(startPose.x, startPose.y, startPose.heading);
            drive = new MecanumDrive(
                    hardwareMap, "leftfront", "rightfront", "leftback", "rightback", robot.localizer);
            drive.setPowerLimits(AUTO_DRIVE_POWER, AUTO_DRIVE_POWER);
            drive.setTolerance(config.positionToleranceCm, config.headingToleranceDeg);
            cameraTransport = Pi5CameraTransportFactory.createTransport(
                    hardwareMap, config, System::nanoTime);
        } catch (Exception e) {
            telemetry.addData("init", "SAFE_STOP: %s", e.getMessage());
            telemetry.update();
            return;
        }

        // Các lớp hỗ trợ biến dữ liệu camera và tọa độ thành thông tin máy trạng thái cần dùng.
        final CameraAdapterManager cameras = new CameraAdapterManager(cameraTransport, config.sensorStaleNs);
        final Pi5GameplayCameraResult camera = new Pi5GameplayCameraResult(cameras, config);
        final PoseNavigation navigation = new PoseNavigation(drive, config);
        final BackOutTracker backOut = new BackOutTracker();

        // Máy trạng thái quyết định robot phải làm gì ở từng bước.
        // Khối Actuators bên dưới nối các lệnh chung của máy trạng thái với phần cứng thật.
        LiftingSequenceStateMachine machine = new LiftingSequenceStateMachine(System::nanoTime,
                new LiftingSequenceStateMachine.Actuators() {
                    // Dừng toàn bộ cơ cấu và bánh xe.
                    public void stop() {
                        robot.stopActuators();
                        drive.stop();
                    }

                    // Đưa thang nâng về vị trí gốc. Trả về true khi đã về tới nơi.
                    public boolean home() {
                        return robot.homeElevator();
                    }

                    // Di chuyển thang nâng từng bước tới độ cao yêu cầu.
                    public boolean elevatorAt(LiftingSequenceConfig.ElevatorTarget target) {
                        return robot.stepElevatorToward(config.elevatorSteps(target), System.nanoTime());
                    }

                    // Đặt hai servo càng nâng vào tư thế giữ hoặc thả khối.
                    public void setFork(LiftingSequenceConfig.ForkPose pose) {
                        robot.servoLeft.setPosition(
                                pose == LiftingSequenceConfig.ForkPose.HOLD ? config.holdLeft : config.placeLeft);
                        robot.servoRight.setPosition(
                                pose == LiftingSequenceConfig.ForkPose.HOLD ? config.holdRight : config.placeRight);
                    }

                    // Lái trực tiếp khi robot đang tiến chậm vào kệ bằng cảm biến IR.
                    public void drive(double forward, double strafe) {
                        navigation.clear();
                        // Khi robot quay 90°, hướng -X của sân là hướng đi vào mặt kệ.
                        drive.driveFieldCentric(0, -forward, strafe);
                    }

                    public void stopDrive() {
                        drive.stop();
                    }

                    public void resetNavigation() {
                        navigation.clear();
                    }

                    // Ghi nhớ vị trí bắt đầu lùi để tính quãng đường đã lùi.
                    public void markBackOutAnchor() {
                        backOut.mark(robot.localizer.getX(), robot.localizer.getY());
                    }

                    public double backOutDistanceCm() {
                        return backOut.distanceCm(robot.localizer.getX(), robot.localizer.getY());
                    }

                    // Trả về vị trí và góc hiện tại của robot.
                    public LiftingSequenceStateMachine.PoseReading pose() {
                        return new LiftingSequenceStateMachine.PoseReading(
                                robot.localizer.getX(),
                                robot.localizer.getY(),
                                robot.localizer.getHeadingDeg(),
                                System.nanoTime());
                    }

                    // Ra lệnh đi tới target và báo true khi robot đã tới đủ gần.
                    public boolean arrival(LiftingSequenceConfig.Pose target, long nowNs) {
                        return navigation.arrival(target);
                    }
                }, camera, config);

        long lastTelemetryNs = 0;
        long stateEnterNs = System.nanoTime();
        LiftingSequenceStateMachine.State lastState = machine.getState();

        try {
            // Vòng lặp chính: đọc cảm biến, cập nhật máy trạng thái, rồi điều khiển robot.
            while (opModeIsActive() && !isStopRequested()
                    && machine.getState() != LiftingSequenceStateMachine.State.SAFE_STOP) {
                long loopStart = System.nanoTime();
                robot.localizer.update();
                camera.update(loopStart);

                // Đọc IR một lần mỗi vòng, dùng chung cho máy trạng thái và telemetry.
                boolean leftIr = !robot.leftIR.getState();
                boolean rightIr = !robot.rightIR.getState();
                machine.setIrState(leftIr, rightIr);
                machine.tick();

                if (machine.getState() != lastState) {
                    lastState = machine.getState();
                    stateEnterNs = loopStart;
                }

                // Chỉ cập nhật bộ đi tới tọa độ trong các trạng thái cần di chuyển.
                if (navigationActive(machine.getState())) {
                    drive.update();
                } else {
                    navigation.clear();
                    drive.stop();
                }

                if (loopStart - lastTelemetryNs >= TELEMETRY_PERIOD_NS) {
                    lastTelemetryNs = loopStart;
                    double stateSec = (loopStart - stateEnterNs) / 1_000_000_000.0;
                    double loopMs = (System.nanoTime() - loopStart) / 1_000_000.0;
                    telemetry.addData("alliance", alliance.name());
                    telemetry.addData("state", machine.getState());
                    telemetry.addData("timing", "state %.2fs settle %d retries %d loop %.1fms",
                            stateSec, machine.getSettleCount(), machine.getRetries(), loopMs);
                    telemetry.addData("shelf/level", "%d/%d", machine.getShelf(), machine.getLevel());
                    telemetry.addData("cycles", machine.getCompletedCycles());
                    telemetry.addData("pose", "%.1f, %.1f, %.1f",
                            robot.localizer.getX(), robot.localizer.getY(), robot.localizer.getHeadingDeg());
                    telemetry.addData("route", navigation.describe());
                    telemetry.addData("navErr", "%.1f cm", drive.getRemainingError());
                    telemetry.addData("blocks", "latched %s / %s", machine.getLeftType(), machine.getRightType());
                    telemetry.addData("cameraLeft", "valid=%s fresh=%s type=%s",
                            camera.leftValid(), camera.leftFresh(loopStart), camera.leftBlockType());
                    telemetry.addData("cameraRight", "valid=%s fresh=%s type=%s",
                            camera.rightValid(), camera.rightFresh(loopStart), camera.rightBlockType());
                    telemetry.addData("IR", "left=%s right=%s", leftIr, rightIr);
                    if (machine.getState() == LiftingSequenceStateMachine.State.APPROACH_IR_SLOW) {
                        telemetry.addData("approach", "%.0f%% field -X", config.approachSpeed * 100.0);
                    }
                    telemetry.addData("drivePower", "%.0f%%", AUTO_DRIVE_POWER * 100.0);
                    telemetry.addData("config", "schema %d fingerprint %s", config.version, config.fingerprint);
                    telemetry.addData("failure", machine.getFailure());
                    telemetry.update();
                }
                idle();
            }
        } finally {
            // Luôn đóng camera và dừng động cơ, kể cả khi xảy ra lỗi.
            if (cameraTransport != null) cameraTransport.close();
            drive.stop();
            robot.stopActuators();
        }
    }

    /** Kiểm tra trạng thái hiện tại có cần bộ lái tự động tới tọa độ hay không. */
    private static boolean navigationActive(LiftingSequenceStateMachine.State state) {
        switch (state) {
            case PLACE_AT_FACTORY:
            case MOVE_TO_SHELF:
            case BACK_OUT_FROM_SHELF:
            case MOVE_NEAR_FACTORY_LEFT:
            case MOVE_TO_PLACEMENT_LEFT:
            case BACK_OUT_AFTER_LEFT_RELEASE_20CM:
            case MOVE_NEAR_FACTORY_RIGHT:
            case MOVE_TO_PLACEMENT_RIGHT:
            case BACK_OUT_AFTER_RIGHT_RELEASE_20CM:
            case APPROACH_IR_SLOW:
                return true;
            default:
                return false;
        }
    }

    /** Giữ tọa độ đích hiện tại và tránh gửi lại cùng một lệnh di chuyển mỗi vòng lặp. */
    private static final class PoseNavigation {
        private final MecanumDrive drive;
        private final LiftingSequenceConfig config;
        private LiftingSequenceConfig.Pose activeTarget = null;

        PoseNavigation(MecanumDrive drive, LiftingSequenceConfig config) {
            this.drive = drive;
            this.config = config;
        }

        void clear() {
            activeTarget = null;
        }

        boolean arrival(LiftingSequenceConfig.Pose target) {
            if (target == null) return false;

            // Chỉ tạo lộ trình mới khi tọa độ đích thay đổi.
            if (!samePose(activeTarget, target)) {
                drive.goToPosition(target.x, target.y, target.heading);
                activeTarget = target;
            }
            return drive.atTarget(config.positionToleranceCm, config.headingToleranceDeg);
        }

        String describe() {
            if (activeTarget == null) return "none";
            return String.format("%.1f, %.1f, %.1f", activeTarget.x, activeTarget.y, activeTarget.heading);
        }

        private static boolean samePose(LiftingSequenceConfig.Pose a, LiftingSequenceConfig.Pose b) {
            if (a == null || b == null) return a == b;
            return Double.compare(a.x, b.x) == 0
                    && Double.compare(a.y, b.y) == 0
                    && Double.compare(a.heading, b.heading) == 0;
        }
    }

    /** Tính khoảng cách robot đã lùi từ một vị trí được đánh dấu trước đó. */
    private static final class BackOutTracker {
        private boolean marked = false;
        private double anchorX;
        private double anchorY;

        void mark(double x, double y) {
            anchorX = x;
            anchorY = y;
            marked = true;
        }

        double distanceCm(double x, double y) {
            if (!marked) return 0.0;
            return Math.hypot(x - anchorX, y - anchorY);
        }
    }
}
