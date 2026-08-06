package org.firstinspires.ftc.teamcode.core;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.DMatch;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.features2d.DescriptorMatcher;
import org.opencv.features2d.ORB;
import org.opencv.imgproc.Imgproc;

import java.io.InputStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public final class MultiTargetCamera {

    public static final int STREAM_WIDTH = 640;
    public static final int STREAM_HEIGHT = 480;

    private final OpenCvWebcam webcam;
    private final MultiTargetPipeline pipeline;

    private volatile String cameraState = "CREATED";
    private volatile int cameraErrorCode = 0;

    public MultiTargetCamera(
            HardwareMap hardwareMap,
            String webcamName,
            boolean showPreview,
            String... targetAssetNames
    ) {
        WebcamName cameraName =
                hardwareMap.get(WebcamName.class, webcamName);

        if (showPreview) {
            int monitorViewId =
                    hardwareMap.appContext.getResources().getIdentifier(
                            "cameraMonitorViewId",
                            "id",
                            hardwareMap.appContext.getPackageName()
                    );

            webcam = OpenCvCameraFactory.getInstance()
                    .createWebcam(cameraName, monitorViewId);
        } else {
            webcam = OpenCvCameraFactory.getInstance()
                    .createWebcam(cameraName);
        }

        pipeline = new MultiTargetPipeline(targetAssetNames);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(3000);
    }

    public void startAsync() {
        cameraState = "OPENING";

        webcam.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        try {
                            webcam.startStreaming(
                                    STREAM_WIDTH,
                                    STREAM_HEIGHT,
                                    OpenCvCameraRotation.UPRIGHT
                            );

                            cameraState = "STREAMING";
                        } catch (RuntimeException e) {
                            cameraState = "ERROR";
                            cameraErrorCode = -1;
                        }
                    }

                    @Override
                    public void onError(int errorCode) {
                        cameraErrorCode = errorCode;
                        cameraState = "ERROR";
                    }
                }
        );
    }

    public void stop() {
        cameraState = "STOPPING";

        try {
            webcam.stopStreaming();
        } catch (RuntimeException ignored) {
        }

        try {
            webcam.closeCameraDevice();
        } catch (RuntimeException ignored) {
        }

        pipeline.release();
        cameraState = "CLOSED";
    }

    public CameraResult getLatestResult() {
        return pipeline.latestResult.get();
    }

    public long getLastSuccessfulDetectionMs() {
        return pipeline.lastSuccessfulDetectionMs;
    }

    public String getCameraState() {
        return cameraState;
    }

    public int getCameraErrorCode() {
        return cameraErrorCode;
    }

    public int getLoadedTemplateCount() {
        return pipeline.templates.size();
    }

    public List<String> getLoadErrors() {
        return Collections.unmodifiableList(
                new ArrayList<>(pipeline.loadErrors)
        );
    }

    public double getFrameFps() {
        return pipeline.frameFps;
    }

    public double getProcessingFps() {
        return pipeline.processingFps;
    }

    public double getProcessingMs() {
        return pipeline.processingMs;
    }

    // ---------- Filter debug telemetry ----------

    public double getRawDx() {
        return pipeline.telemetryRawDx;
    }

    public double getSmoothedDx() {
        return pipeline.telemetrySmoothedDx;
    }

    public int getOutlierStreak() {
        return pipeline.telemetryOutlierStreak;
    }

    public int getMissStreak() {
        return pipeline.telemetryMissStreak;
    }

    public boolean isFilterInitialized() {
        return pipeline.telemetryFilterInit;
    }

    public String getActiveLabel() {
        return pipeline.telemetryActiveLabel;
    }

    public static final class Detection {

        public final String label;

        public final double centerX;
        public final double centerY;

        public final double dxPx;
        public final double dyPx;

        public final double distanceToCenter;

        public final int goodMatches;
        public final int inliers;

        public final double confidence;

        public final Point[] corners;

        private Detection(
                String label,
                double centerX,
                double centerY,
                double dxPx,
                double dyPx,
                double distanceToCenter,
                int goodMatches,
                int inliers,
                double confidence,
                Point[] corners
        ) {
            this.label = label;

            this.centerX = centerX;
            this.centerY = centerY;

            this.dxPx = dxPx;
            this.dyPx = dyPx;

            this.distanceToCenter = distanceToCenter;

            this.goodMatches = goodMatches;
            this.inliers = inliers;

            this.confidence = confidence;
            this.corners = corners.clone();
        }
    }

    public static final class CameraResult {

        public final long timestampMs;

        public final List<Detection> detections;
        public final Detection selected;

        public final double dxPx;
        public final double dyPx;

        private CameraResult(
                long timestampMs,
                List<Detection> detections,
                Detection selected,
                double dxPx,
                double dyPx
        ) {
            this.timestampMs = timestampMs;

            this.detections = Collections.unmodifiableList(
                    new ArrayList<>(detections)
            );

            this.selected = selected;
            this.dxPx = dxPx;
            this.dyPx = dyPx;
        }

        public boolean isValid() {
            return selected != null;
        }

        private static CameraResult empty(long timestampMs) {
            return new CameraResult(
                    timestampMs,
                    Collections.<Detection>emptyList(),
                    null,
                    Double.NaN,
                    Double.NaN
            );
        }
    }

    private static final class TemplateData {

        final String label;
        final Mat descriptors;
        final KeyPoint[] keypoints;
        final MatOfPoint2f corners;

        TemplateData(
                String label,
                Mat descriptors,
                KeyPoint[] keypoints,
                MatOfPoint2f corners
        ) {
            this.label = label;
            this.descriptors = descriptors;
            this.keypoints = keypoints;
            this.corners = corners;
        }

        void release() {
            descriptors.release();
            corners.release();
        }
    }

    private static final class MultiTargetPipeline
            extends OpenCvPipeline {

        /*
         * Thuật toán cốt lõi — thắt chặt để tối đa độ ổn định.
         */
        private static final double RATIO_TEST = 0.70;
        private static final int MIN_GOOD_MATCHES = 15;
        private static final double RANSAC_THRESHOLD = 4.0;

        /*
         * Nén ảnh target: canh dài nhất không quá MAX_TARGET_DIM px,
         * giữ nguyên tỉ lệ. Camera chỉ 320x240 nên ảnh >200px là lãng phí.
         */
        private static final int MAX_TARGET_DIM = 200;

        /*
         * Bộ lọc thời gian — tuned cho độ mượt tối đa.
         */
        private static final int OUTLIER_CONFIRM_FRAMES = 4;

        private static final double OUTLIER_LIMIT_PX = 45.0;

        private static final int HOLD_MISS_FRAMES = 6;
        private static final long HOLD_MISS_TIMEOUT_MS = 300;

        private static final int LABEL_SWITCH_CONFIRM_FRAMES = 3;

        private static final double CENTER_DEADBAND_PX = 2.0;
        private static final double EMA_ALPHA_SLOW = 0.08;
        private static final double EMA_ALPHA_FAST = 0.45;
        private static final double FAST_MOTION_THRESHOLD_PX = 8.0;
        private static final double MAX_OUTPUT_STEP_PX = 20.0;

        private static final double FPS_EMA_ALPHA = 0.20;

        private static final long MISS_REINIT_TIMEOUT_MS = 250;

        private final ORB orb = ORB.create(
                350,
                1.2f,
                4,
                15,
                0,
                2,
                ORB.HARRIS_SCORE,
                31,
                20
        );

        private final DescriptorMatcher matcher =
                DescriptorMatcher.create(
                        DescriptorMatcher.BRUTEFORCE_HAMMING
                );

        private final List<TemplateData> templates =
                new ArrayList<>();

        private final List<String> loadErrors =
                new ArrayList<>();

        private final AtomicReference<CameraResult> latestResult =
                new AtomicReference<>(CameraResult.empty(0));

        /*
         * Reuse Mat để giảm garbage collection.
         */
        private final Mat gray = new Mat();
        private final Mat emptyMask = new Mat();

        private final Mat frameDescriptors = new Mat();
        private final MatOfKeyPoint frameKeypoints =
                new MatOfKeyPoint();

        private boolean filterInitialized = false;

        private double smoothedDx = 0.0;
        private double smoothedDy = 0.0;

        private String activeLabel = "";
        private String pendingLabel = "";

        private int pendingLabelHits = 0;
        private int missStreak = 0;
        private int outlierStreak = 0;

        private Detection lastStableSelected = null;

        private List<Detection> lastStableDetections =
                Collections.emptyList();

        private long lastFrameNs = 0;
        private long lastProcessingNs = 0;

        volatile double frameFps = 0.0;
        volatile double processingFps = 0.0;
        volatile double processingMs = 0.0;

        volatile long lastSuccessfulDetectionMs = 0;

        private double lastRawDx = 0.0;
        private double lastRawDy = 0.0;

        private long missStartMs = 0;

        /*
         * Telemetry copies — volatile để CameraOdometryMain đọc
         * không cần lock từ main thread.
         */
        volatile double telemetryRawDx = 0.0;
        volatile double telemetrySmoothedDx = 0.0;
        volatile int    telemetryOutlierStreak = 0;
        volatile int    telemetryMissStreak = 0;
        volatile boolean telemetryFilterInit = false;
        volatile String telemetryActiveLabel = "";

        MultiTargetPipeline(String[] targetAssetNames) {
            if (targetAssetNames == null
                    || targetAssetNames.length == 0) {
                loadErrors.add(
                        "No target PNG assets configured"
                );
                return;
            }

            for (String assetName : targetAssetNames) {
                loadTemplate(assetName);
            }
        }

        private void loadTemplate(String assetName) {
            if (assetName == null
                    || assetName.trim().isEmpty()) {
                loadErrors.add("Empty target asset name");
                return;
            }

            Bitmap bitmap = null;

            Mat rgba = new Mat();
            Mat targetGray = new Mat();
            Mat targetDescriptors = new Mat();

            MatOfKeyPoint targetKeypoints =
                    new MatOfKeyPoint();

            try (InputStream inputStream =
                         AppUtil.getDefContext()
                                 .getAssets()
                                 .open(assetName)) {

                bitmap = BitmapFactory.decodeStream(inputStream);

                if (bitmap == null) {
                    throw new IllegalArgumentException(
                            "Bitmap decode returned null"
                    );
                }

                Utils.bitmapToMat(bitmap, rgba);

                Imgproc.cvtColor(
                        rgba,
                        targetGray,
                        Imgproc.COLOR_RGBA2GRAY
                );

                /*
                 * Nén ảnh target xuống nếu quá lớn, giữ nguyên tỉ lệ.
                 */
                int tw = targetGray.cols();
                int th = targetGray.rows();

                if (tw > MAX_TARGET_DIM || th > MAX_TARGET_DIM) {
                    double scale = Math.min(
                            (double) MAX_TARGET_DIM / tw,
                            (double) MAX_TARGET_DIM / th
                    );

                    int nw = (int) Math.round(tw * scale);
                    int nh = (int) Math.round(th * scale);

                    // Không dùng Mat mới — resize ngược về targetGray
                    Mat resized = new Mat();
                    Imgproc.resize(
                            targetGray,
                            resized,
                            new org.opencv.core.Size(nw, nh)
                    );

                    targetGray.release();
                    targetGray = resized;

                    tw = nw;
                    th = nh;
                }

                orb.detectAndCompute(
                        targetGray,
                        emptyMask,
                        targetKeypoints,
                        targetDescriptors
                );

                KeyPoint[] keypointArray =
                        targetKeypoints.toArray();

                if (targetDescriptors.empty()
                        || keypointArray.length
                        < MIN_GOOD_MATCHES) {
                    throw new IllegalArgumentException(
                            "Too few ORB features: "
                                    + keypointArray.length
                    );
                }

                MatOfPoint2f corners =
                        new MatOfPoint2f(
                                new Point(0, 0),
                                new Point(
                                        targetGray.cols(),
                                        0
                                ),
                                new Point(
                                        targetGray.cols(),
                                        targetGray.rows()
                                ),
                                new Point(
                                        0,
                                        targetGray.rows()
                                )
                        );

                templates.add(
                        new TemplateData(
                                labelFromAsset(assetName),
                                targetDescriptors.clone(),
                                keypointArray,
                                corners
                        )
                );

            } catch (Exception e) {
                loadErrors.add(
                        assetName + ": " + e.getMessage()
                );
            } finally {
                targetKeypoints.release();
                targetDescriptors.release();
                targetGray.release();
                rgba.release();

                if (bitmap != null
                        && !bitmap.isRecycled()) {
                    bitmap.recycle();
                }
            }
        }

        @Override
        public Mat processFrame(Mat frame) {
            long startNs = System.nanoTime();
            long nowMs = System.currentTimeMillis();

            updateFrameFps(startNs);
            updateProcessingFps(startNs);

            CameraResult result;

            try {
                result = detect(frame, nowMs);
            } catch (RuntimeException ignored) {
                /*
                 * Một lỗi xử lý đơn lẻ không làm kết quả nhấp nháy.
                 * Pipeline giữ kết quả ổn định gần nhất trong thời gian ngắn.
                 */
                result = handleMiss(nowMs);
            }

            latestResult.set(result);

            processingMs =
                    (System.nanoTime() - startNs) / 1e6;

            drawResult(frame, result);

            return frame;
        }

        private CameraResult detect(
                Mat frame,
                long nowMs
        ) {
            if (templates.isEmpty()) {
                return CameraResult.empty(nowMs);
            }

            Imgproc.cvtColor(
                    frame,
                    gray,
                    Imgproc.COLOR_RGBA2GRAY
            );

            /*
             * Detect frame ORB một lần. Các tham số nhận diện
             * được giữ nguyên so với class ban đầu.
             */
            orb.detectAndCompute(
                    gray,
                    emptyMask,
                    frameKeypoints,
                    frameDescriptors
            );

            if (frameDescriptors.empty()) {
                return handleMiss(nowMs);
            }

            KeyPoint[] frameKeypointArray =
                    frameKeypoints.toArray();

            List<Detection> detections =
                    new ArrayList<>();

            /*
             * Duyệt template theo thứ tự: active label trước,
             * rồi các label còn lại. Nếu dã có detection tốt
             * (confidence cao + gần tâm) thì bỏ qua các template sau.
             */
            List<TemplateData> sorted = new ArrayList<>(templates);
            sortTemplatesByPriority(sorted);

            for (TemplateData template : sorted) {
                /*
                 * Early exit: nếu dã có detection gần tâm với
                 * confidence cao, không cần check thêm template.
                 */
                Detection best = selectNearestToCenter(detections);
                if (best != null
                        && best.confidence > 0.85
                        && best.distanceToCenter < 80) {
                    break;
                }

                Detection detection =
                        detectSingleTemplate(
                                template,
                                frameKeypointArray,
                                frameDescriptors,
                                frame.cols(),
                                frame.rows()
                        );

                if (detection != null) {
                    detections.add(detection);
                }
            }

            Detection nearest =
                    selectNearestToCenter(detections);

            if (nearest == null) {
                return handleMiss(nowMs);
            }

            /*
             * Chống label đổi qua lại giữa các frame. Nếu label mới
             * chưa đủ số frame xác nhận, tiếp tục dùng label hiện tại.
             */
            Detection selected =
                    applyLabelHysteresis(
                            detections,
                            nearest
                    );

            if (selected == null) {
                return handleMiss(nowMs);
            }

            boolean labelChanged =
                    !selected.label.equals(activeLabel);

            if (labelChanged) {
                activeLabel = selected.label;
                resetPositionFilter(
                        selected.dxPx,
                        selected.dyPx
                );
            }

            /*
             * Re-init filter nếu vừa ra khỏi miss kéo dài.
             */
            if (missStreak > 0) {
                long missAgeMs = nowMs - missStartMs;
                if (missAgeMs > MISS_REINIT_TIMEOUT_MS) {
                    resetPositionFilter(
                            selected.dxPx,
                            selected.dyPx
                    );
                }
            }

            /*
             * Outlier đơn lẻ không còn trả result rỗng. Giữ kết quả cũ.
             * Nếu vị trí mới lặp lại đủ 3 frame, coi đó là chuyển động thật.
             *
             * isOutlier() so sánh raw hiện tại với lastRawDx/Dy (raw frame trước).
             * lastRawDx/Dy được cập nhật SAU outlier check, trong success path.
             */
            if (isOutlier(
                    selected.dxPx,
                    selected.dyPx
            )) {
                outlierStreak++;

                if (outlierStreak
                        < OUTLIER_CONFIRM_FRAMES) {
                    return handleMiss(nowMs);
                }

                /*
                 * Outlier confirmed sau 3 frame — re-init từ raw để
                 * không bị giật khi chấp nhận vị trí mới.
                 */
                resetPositionFilter(
                        selected.dxPx,
                        selected.dyPx
                );
            }

            outlierStreak = 0;

            updatePositionFilter(
                    selected.dxPx,
                    selected.dyPx,
                    selected.confidence
            );

            /*
             * lastRawDx/Dy ghi raw của frame hiện tại để frame sau
             * dùng làm baseline cho isOutlier().
             */
            lastRawDx = selected.dxPx;
            lastRawDy = selected.dyPx;

            telemetryRawDx = selected.dxPx;
            telemetrySmoothedDx = smoothedDx;
            telemetryOutlierStreak = outlierStreak;
            telemetryMissStreak = missStreak;
            telemetryFilterInit = filterInitialized;
            telemetryActiveLabel = activeLabel;

            Detection stableSelected =
                    stabilizeDetection(selected);

            List<Detection> stableDetections =
                    replaceSelectedDetection(
                            detections,
                            selected,
                            stableSelected
                    );

            missStreak = 0;
            lastSuccessfulDetectionMs = nowMs;

            lastStableSelected = stableSelected;
            lastStableDetections = stableDetections;

            return new CameraResult(
                    nowMs,
                    stableDetections,
                    stableSelected,
                    smoothedDx,
                    smoothedDy
            );
        }

        /*
         * Thuật toán cũ chạy độc lập với một PNG.
         */
        private Detection detectSingleTemplate(
                TemplateData template,
                KeyPoint[] frameKeypointArray,
                Mat currentFrameDescriptors,
                int frameWidth,
                int frameHeight
        ) {
            List<MatOfDMatch> knnMatches =
                    new ArrayList<>();

            List<DMatch> goodMatches =
                    new ArrayList<>(40);

            MatOfPoint2f targetPointMat =
                    new MatOfPoint2f();

            MatOfPoint2f framePointMat =
                    new MatOfPoint2f();

            Mat homography = null;
            Mat inlierMask = new Mat();

            MatOfPoint2f projectedCorners =
                    new MatOfPoint2f();

            try {
                /*
                 * Giữ nguyên hướng match cũ:
                 * target descriptors -> frame descriptors.
                 */
                matcher.knnMatch(
                        template.descriptors,
                        currentFrameDescriptors,
                        knnMatches,
                        2
                );

                for (MatOfDMatch matchPair : knnMatches) {
                    DMatch[] pair = matchPair.toArray();

                    if (pair.length >= 2
                            && pair[0].distance
                            < RATIO_TEST
                            * pair[1].distance) {
                        goodMatches.add(pair[0]);
                    }
                }

                if (goodMatches.size()
                        < MIN_GOOD_MATCHES) {
                    return null;
                }

                List<Point> targetPoints =
                        new ArrayList<>(
                                goodMatches.size()
                        );

                List<Point> framePoints =
                        new ArrayList<>(
                                goodMatches.size()
                        );

                for (DMatch match : goodMatches) {
                    targetPoints.add(
                            template.keypoints[
                                    match.queryIdx
                                    ].pt
                    );

                    framePoints.add(
                            frameKeypointArray[
                                    match.trainIdx
                                    ].pt
                    );
                }

                targetPointMat.fromList(targetPoints);
                framePointMat.fromList(framePoints);

                homography = Calib3d.findHomography(
                        targetPointMat,
                        framePointMat,
                        Calib3d.RANSAC,
                        RANSAC_THRESHOLD,
                        inlierMask
                );

                if (homography.empty()) {
                    return null;
                }

                Core.perspectiveTransform(
                        template.corners,
                        projectedCorners,
                        homography
                );

                Point[] corners =
                        projectedCorners.toArray();

                if (corners.length != 4) {
                    return null;
                }

                double centerX =
                        (
                                corners[0].x
                                        + corners[1].x
                                        + corners[2].x
                                        + corners[3].x
                        ) / 4.0;

                double centerY =
                        (
                                corners[0].y
                                        + corners[1].y
                                        + corners[2].y
                                        + corners[3].y
                        ) / 4.0;

                /*
                 * Giữ nguyên kiểm tra tâm cũ.
                 */
                if (centerX < 0
                        || centerX > frameWidth
                        || centerY < 0
                        || centerY > frameHeight) {
                    return null;
                }

                double dxPx =
                        centerX - frameWidth / 2.0;

                double dyPx =
                        centerY - frameHeight / 2.0;

                double distanceToCenter =
                        Math.sqrt(
                                dxPx * dxPx
                                        + dyPx * dyPx
                        );

                int inliers = inlierMask.empty()
                        ? 0
                        : Core.countNonZero(inlierMask);

                double inlierRatio =
                        goodMatches.isEmpty()
                                ? 0.0
                                : inliers
                                / (double) goodMatches.size();

                double matchScore =
                        Math.min(
                                1.0,
                                goodMatches.size() / 30.0
                        );

                double confidence =
                        Math.min(
                                1.0,
                                0.7 * inlierRatio
                                        + 0.3 * matchScore
                        );

                return new Detection(
                        template.label,
                        centerX,
                        centerY,
                        dxPx,
                        dyPx,
                        distanceToCenter,
                        goodMatches.size(),
                        inliers,
                        confidence,
                        corners
                );

            } finally {
                for (MatOfDMatch match : knnMatches) {
                    match.release();
                }

                targetPointMat.release();
                framePointMat.release();

                if (homography != null) {
                    homography.release();
                }

                inlierMask.release();
                projectedCorners.release();
            }
        }

        /**
         * Sap xep template de uu tien label dang theo doi.
         * Giam so lan can phai chay RANSAC cho template khong can thiet.
         */
        private void sortTemplatesByPriority(
                List<TemplateData> list
        ) {
            if (activeLabel.isEmpty() || list.size() <= 1) {
                return;
            }

            // Tim template cua active label, dua len dầu
            for (int i = 0; i < list.size(); i++) {
                if (list.get(i).label.equals(activeLabel)) {
                    if (i > 0) {
                        TemplateData t = list.remove(i);
                        list.add(0, t);
                    }
                    return;
                }
            }
        }

        private Detection selectNearestToCenter(
                List<Detection> detections
        ) {
            Detection nearest = null;

            for (Detection detection : detections) {
                if (nearest == null
                        || detection.distanceToCenter
                        < nearest.distanceToCenter) {
                    nearest = detection;
                }
            }

            return nearest;
        }

        private Detection applyLabelHysteresis(
                List<Detection> detections,
                Detection nearest
        ) {
            if (activeLabel.isEmpty()) {
                clearPendingLabel();
                return nearest;
            }

            if (nearest.label.equals(activeLabel)) {
                clearPendingLabel();
                return nearest;
            }

            if (nearest.label.equals(pendingLabel)) {
                pendingLabelHits++;
            } else {
                pendingLabel = nearest.label;
                pendingLabelHits = 1;
            }

            if (pendingLabelHits
                    >= LABEL_SWITCH_CONFIRM_FRAMES) {
                clearPendingLabel();
                return nearest;
            }

            /*
             * Label mới chưa được xác nhận. Nếu label đang theo dõi
             * vẫn xuất hiện, tiếp tục dùng nó thay vì nhảy target.
             */
            return findDetectionByLabel(
                    detections,
                    activeLabel
            );
        }

        private Detection findDetectionByLabel(
                List<Detection> detections,
                String label
        ) {
            for (Detection detection : detections) {
                if (detection.label.equals(label)) {
                    return detection;
                }
            }

            return null;
        }

        private void clearPendingLabel() {
            pendingLabel = "";
            pendingLabelHits = 0;
        }

        private boolean isOutlier(
                double newDx,
                double newDy
        ) {
            if (!filterInitialized) {
                return false;
            }

            /*
             * So sánh raw hiện tại với raw frame trước (zero lag).
             * Không so sánh với smoothed (EMA) vì EMA luôn lag → false positive
             * với mọi chuyển động đều có vận tốc > 11 px/frame.
             */
            double diffX = newDx - lastRawDx;
            double diffY = newDy - lastRawDy;

            return Math.hypot(diffX, diffY)
                    > OUTLIER_LIMIT_PX;
        }

        private void updatePositionFilter(
                double newDx,
                double newDy,
                double confidence
        ) {
            if (!filterInitialized) {
                smoothedDx = newDx;
                smoothedDy = newDy;
                filterInitialized = true;
                return;
            }

            double diffX = newDx - smoothedDx;
            double diffY = newDy - smoothedDy;
            double distance = Math.hypot(diffX, diffY);

            /*
             * Deadband mở rộng khi confidence thấp.
             * Confidence 1.0 → deadband = CENTER_DEADBAND_PX
             * Confidence 0.5 → deadband gấp 1.5x
             */
            double effectiveDeadband =
                    CENTER_DEADBAND_PX
                            * (1.0 + 1.0 - confidence);

            if (distance <= effectiveDeadband) {
                return;
            }

            /*
             * Chuyển động nhỏ dùng alpha thấp để tâm mượt.
             * Chuyển động lớn dùng alpha cao để không làm robot phản hồi chậm.
             */
            double baseAlpha =
                    distance >= FAST_MOTION_THRESHOLD_PX
                            ? EMA_ALPHA_FAST
                            : EMA_ALPHA_SLOW;

            /*
             * Scale alpha theo confidence.
             * Confidence 1.0 → alpha không đổi
             * Confidence 0.5 → alpha giảm 25%
             */
            double effectiveAlpha =
                    baseAlpha * (0.5 + 0.5 * confidence);

            double stepX = effectiveAlpha * diffX;
            double stepY = effectiveAlpha * diffY;
            double stepDistance = Math.hypot(stepX, stepY);

            if (stepDistance > MAX_OUTPUT_STEP_PX) {
                double scale =
                        MAX_OUTPUT_STEP_PX / stepDistance;

                stepX *= scale;
                stepY *= scale;
            }

            smoothedDx += stepX;
            smoothedDy += stepY;
        }

        private Detection stabilizeDetection(
                Detection rawDetection
        ) {
            double shiftX =
                    smoothedDx - rawDetection.dxPx;

            double shiftY =
                    smoothedDy - rawDetection.dyPx;

            Point[] stableCorners =
                    new Point[rawDetection.corners.length];

            for (int i = 0;
                 i < rawDetection.corners.length;
                 i++) {
                stableCorners[i] = new Point(
                        rawDetection.corners[i].x + shiftX,
                        rawDetection.corners[i].y + shiftY
                );
            }

            return new Detection(
                    rawDetection.label,
                    rawDetection.centerX + shiftX,
                    rawDetection.centerY + shiftY,
                    smoothedDx,
                    smoothedDy,
                    Math.hypot(
                            smoothedDx,
                            smoothedDy
                    ),
                    rawDetection.goodMatches,
                    rawDetection.inliers,
                    rawDetection.confidence,
                    stableCorners
            );
        }

        private List<Detection> replaceSelectedDetection(
                List<Detection> detections,
                Detection rawSelected,
                Detection stableSelected
        ) {
            List<Detection> stableDetections =
                    new ArrayList<>(detections.size());

            for (Detection detection : detections) {
                stableDetections.add(
                        detection == rawSelected
                                ? stableSelected
                                : detection
                );
            }

            return stableDetections;
        }

        private CameraResult handleMiss(long nowMs) {
            if (missStreak == 0) {
                missStartMs = nowMs;
            }

            missStreak++;

            long ageMs =
                    lastSuccessfulDetectionMs <= 0
                            ? Long.MAX_VALUE
                            : nowMs
                            - lastSuccessfulDetectionMs;

            boolean canHold =
                    lastStableSelected != null
                            && missStreak
                            <= HOLD_MISS_FRAMES
                            && ageMs
                            <= HOLD_MISS_TIMEOUT_MS;

            if (canHold) {
                return new CameraResult(
                        nowMs,
                        lastStableDetections,
                        lastStableSelected,
                        smoothedDx,
                        smoothedDy
                );
            }

            if (lastStableSelected != null
                    || missStreak > HOLD_MISS_FRAMES) {
                clearTrackingState();
            }

            return CameraResult.empty(nowMs);
        }

        private void resetPositionFilter(
                double rawDx,
                double rawDy
        ) {
            filterInitialized = false;
            smoothedDx = rawDx;
            smoothedDy = rawDy;
            outlierStreak = 0;
        }

        private void clearTrackingState() {
            resetPositionFilter(0.0, 0.0);

            activeLabel = "";
            clearPendingLabel();

            missStreak = 0;
            missStartMs = 0;

            lastStableSelected = null;
            lastStableDetections =
                    Collections.emptyList();
        }

        private void updateFrameFps(long nowNs) {
            if (lastFrameNs != 0) {
                double dt =
                        (nowNs - lastFrameNs) / 1e9;

                if (dt > 0.0) {
                    double instantFps = 1.0 / dt;

                    frameFps = frameFps <= 0.0
                            ? instantFps
                            : frameFps
                            + FPS_EMA_ALPHA
                            * (instantFps - frameFps);
                }
            }

            lastFrameNs = nowNs;
        }

        private void updateProcessingFps(long nowNs) {
            if (lastProcessingNs != 0) {
                double dt =
                        (nowNs - lastProcessingNs) / 1e9;

                if (dt > 0.0) {
                    double instantFps = 1.0 / dt;

                    processingFps = processingFps <= 0.0
                            ? instantFps
                            : processingFps
                            + FPS_EMA_ALPHA
                            * (instantFps
                            - processingFps);
                }
            }

            lastProcessingNs = nowNs;
        }

        private void drawResult(
                Mat frame,
                CameraResult result
        ) {
            Imgproc.circle(
                    frame,
                    new Point(
                            frame.cols() / 2.0,
                            frame.rows() / 2.0
                    ),
                    4,
                    new Scalar(255, 0, 0),
                    -1
            );

            for (Detection detection : result.detections) {
                boolean isSelected =
                        detection == result.selected;

                Scalar color = isSelected
                        ? new Scalar(0, 255, 0)
                        : new Scalar(255, 255, 0);

                for (int i = 0; i < 4; i++) {
                    Imgproc.line(
                            frame,
                            detection.corners[i],
                            detection.corners[
                                    (i + 1) % 4
                                    ],
                            color,
                            isSelected ? 2 : 1
                    );
                }

                Imgproc.circle(
                        frame,
                        new Point(
                                detection.centerX,
                                detection.centerY
                        ),
                        isSelected ? 5 : 3,
                        color,
                        -1
                );

                Imgproc.putText(
                        frame,
                        detection.label,
                        new Point(
                                detection.centerX + 5,
                                detection.centerY - 5
                        ),
                        Imgproc.FONT_HERSHEY_SIMPLEX,
                        0.4,
                        color,
                        1
                );
            }
        }

        void release() {
            clearTrackingState();

            gray.release();
            emptyMask.release();

            frameDescriptors.release();
            frameKeypoints.release();

            for (TemplateData template : templates) {
                template.release();
            }

            templates.clear();

            orb.clear();
            matcher.clear();
        }

        private static String labelFromAsset(
                String assetName
        ) {
            int slashIndex = Math.max(
                    assetName.lastIndexOf('/'),
                    assetName.lastIndexOf('\\')
            );

            String fileName =
                    slashIndex >= 0
                            ? assetName.substring(
                            slashIndex + 1
                    )
                            : assetName;

            int dotIndex = fileName.lastIndexOf('.');

            return dotIndex > 0
                    ? fileName.substring(0, dotIndex)
                    : fileName;
        }
    }
}