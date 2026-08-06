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
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Single target camera — chỉ match MÔT template.
 *
 * Không có vòng lặp duyêt nhiêu template, không có label hysteresis,
 * không có selectNearestToCenter. Mỗi instance chi match 1 ành.
 *
 * Dùng `setTarget(assetName)` dê chuyên target giữa các state.
 *
 * Cách dùng:
 * <pre>{@code
 *   SingleTargetCamera cam = new SingleTargetCamera(
 *       hwMap, "Webcam 1", true, "sample.png");
 *   cam.startAsync();
 *
 *   while (opModeIsActive()) {
 *       CameraResult r = cam.getLatestResult();
 *       if (r.isValid()) {
 *           double dx = r.dxPx;
 *           double dy = r.dyPx;
 *       }
 *       sleep(20);
 *   }
 *   cam.stop();
 * }</pre>
 */
public final class SingleTargetCamera {

    // ============================================================
    //  >>>  TUNE  <<<
    // ============================================================
    public static final int STREAM_WIDTH  = 640;
    public static final int STREAM_HEIGHT = 480;

    // ============================================================
    //  Instance
    // ============================================================

    private final OpenCvWebcam webcam;
    private final SingleTargetPipeline pipeline;

    private volatile String cameraState = "CREATED";
    private volatile int    cameraErrorCode;

    // ============================================================
    //  Constructor
    // ============================================================

    /**
     * @param targetAssetName tên file PNG trong assets/ (1 file duy nhất).
     *                        Có thê dổi sau bằng `setTarget()`.
     */
    public SingleTargetCamera(
            HardwareMap hardwareMap,
            String webcamName,
            boolean showPreview,
            String targetAssetName
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

        pipeline = new SingleTargetPipeline(targetAssetName);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(3000);
    }

    /**
     * Test constructor — không cần FTC hardware.
     */
    SingleTargetCamera(String targetAssetName) {
        this.webcam = null;
        this.pipeline = new SingleTargetPipeline(targetAssetName);
    }

    // ============================================================
    //  Lifecycle
    // ============================================================

    public void startAsync() {
        if (webcam == null) return;

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

        try { if (webcam != null) webcam.stopStreaming(); } catch (RuntimeException ignored) {}
        try { if (webcam != null) webcam.closeCameraDevice(); } catch (RuntimeException ignored) {}

        pipeline.release();
        cameraState = "CLOSED";
    }

    // ============================================================
    //  Public API
    // ============================================================

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

    public boolean isTemplateLoaded() {
        return pipeline.template != null;
    }

    public String getLoadError() {
        List<String> errors = pipeline.loadErrors;
        return errors.isEmpty() ? null : errors.get(0);
    }

    public String getTemplateLabel() {
        return pipeline.templateLabel;
    }

    public double getFrameFps()      { return pipeline.frameFps; }
    public double getProcessingFps() { return pipeline.processingFps; }
    public double getProcessingMs()  { return pipeline.processingMs; }

    // Filter debug
    public double  getRawDx()           { return pipeline.telemetryRawDx; }
    public double  getSmoothedDx()      { return pipeline.telemetrySmoothedDx; }
    public int     getOutlierStreak()   { return pipeline.telemetryOutlierStreak; }
    public int     getMissStreak()      { return pipeline.telemetryMissStreak; }
    public boolean isFilterInitialized(){ return pipeline.telemetryFilterInit; }
    public String  getActiveLabel()     { return pipeline.templateLabel; }

    /**
     * Chuyên sang target mới. Reset toàn bô filter + template cũ.
     */
    public void setTarget(String assetName) {
        pipeline.setTarget(assetName);
    }

    /**
     * Dùng trong test offline: trà vê pipeline dê feed raw data vào filter.
     */
    SingleTargetPipeline getPipeline() {
        return pipeline;
    }

    // ============================================================
    //  Data classes
    // ============================================================

    public static final class Detection {

        public final String label;
        public final double centerX, centerY;
        public final double dxPx, dyPx;
        public final double distanceToCenter;
        public final int    goodMatches, inliers;
        public final double confidence;
        public final Point[] corners;

        private Detection(
                String label,
                double centerX, double centerY,
                double dxPx, double dyPx,
                double distanceToCenter,
                int goodMatches, int inliers,
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

        public final long      timestampMs;
        public final Detection detection;  // null nếu không detect duọc

        public final double dxPx;
        public final double dyPx;

        private CameraResult(
                long timestampMs,
                Detection detection,
                double dxPx,
                double dyPx
        ) {
            this.timestampMs = timestampMs;
            this.detection = detection;
            this.dxPx = dxPx;
            this.dyPx = dyPx;
        }

        public boolean isValid() {
            return detection != null;
        }

        private static CameraResult empty(long timestampMs) {
            return new CameraResult(timestampMs, null, Double.NaN, Double.NaN);
        }
    }

    // ============================================================
    //  TemplateData
    // ============================================================

    private static final class TemplateData {

        final String        label;
        final Mat           descriptors;
        final KeyPoint[]    keypoints;
        final MatOfPoint2f  corners;

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

    // ============================================================
    //  Pipeline (single-target)
    // ============================================================

    static final class SingleTargetPipeline
            extends OpenCvPipeline {

        // ---------- Detection (accuracy tuned — 1 template nên rộng rãi) ----------
        private static final double RATIO_TEST       = 0.76;
        private static final int    MIN_GOOD_MATCHES = 12;
        private static final double RANSAC_THRESHOLD = 3.0;

        // ---------- Target resize ----------
        private static final int MAX_TARGET_DIM = 200;

        // ---------- Temporal filter ----------
        private static final int    OUTLIER_CONFIRM_FRAMES = 4;
        private static final double OUTLIER_LIMIT_PX       = 45.0;
        private static final int    HOLD_MISS_FRAMES       = 6;
        private static final long   HOLD_MISS_TIMEOUT_MS   = 300;
        private static final long   MISS_REINIT_TIMEOUT_MS = 250;

        private static final double CENTER_DEADBAND_PX     = 2.0;
        private static final double EMA_ALPHA_SLOW         = 0.08;
        private static final double EMA_ALPHA_FAST         = 0.45;
        private static final double FAST_MOTION_THRESHOLD_PX = 8.0;
        private static final double MAX_OUTPUT_STEP_PX     = 20.0;
        private static final double FPS_EMA_ALPHA          = 0.20;

        // ---------- ORB (high-accuracy — 1000 features, 6 pyramid levels) ----------
        private final ORB orb = ORB.create(
                1000,
                1.2f,
                6,
                20,
                0,
                2,
                ORB.HARRIS_SCORE,
                31,
                10
        );

        private final DescriptorMatcher matcher =
                DescriptorMatcher.create(DescriptorMatcher.BRUTEFORCE_HAMMING);

        // ============================================================
        //  State
        // ============================================================

        private TemplateData template;
        private String       templateLabel = "";

        private final List<String> loadErrors = new ArrayList<>();

        private final AtomicReference<CameraResult> latestResult =
                new AtomicReference<>(CameraResult.empty(0));

        // Reusable Mats
        private final Mat gray         = new Mat();
        private final Mat emptyMask    = new Mat();
        private final Mat frameDescriptors = new Mat();
        private final MatOfKeyPoint frameKeypoints = new MatOfKeyPoint();

        // Temporal filter state
        private boolean filterInitialized;
        private double  smoothedDx, smoothedDy;
        private double  lastRawDx, lastRawDy;
        private int     missStreak, outlierStreak;
        private long    missStartMs;

        private Detection lastStableDetection;

        private long lastFrameNs, lastProcessingNs;

        volatile double  frameFps, processingFps, processingMs;
        volatile long    lastSuccessfulDetectionMs;

        // Telemetry copies
        volatile double  telemetryRawDx;
        volatile double  telemetrySmoothedDx;
        volatile int     telemetryOutlierStreak;
        volatile int     telemetryMissStreak;
        volatile boolean telemetryFilterInit;

        // ============================================================
        //  Init
        // ============================================================

        SingleTargetPipeline(String targetAssetName) {
            if (targetAssetName != null && !targetAssetName.trim().isEmpty()) {
                loadTemplate(targetAssetName);
            }
        }

        void setTarget(String assetName) {
            clearTrackingState();

            if (template != null) {
                template.release();
                template = null;
            }

            templateLabel = "";
            loadErrors.clear();

            if (assetName != null && !assetName.trim().isEmpty()) {
                loadTemplate(assetName);
            }
        }

        private String labelFromAsset(String assetName) {
            int slashIndex = Math.max(
                    assetName.lastIndexOf('/'),
                    assetName.lastIndexOf('\\')
            );
            String fileName = slashIndex >= 0
                    ? assetName.substring(slashIndex + 1)
                    : assetName;
            int dotIndex = fileName.lastIndexOf('.');
            return dotIndex > 0
                    ? fileName.substring(0, dotIndex)
                    : fileName;
        }

        private void loadTemplate(String assetName) {
            if (assetName == null || assetName.trim().isEmpty()) {
                loadErrors.add("Empty target asset name");
                return;
            }

            Bitmap bitmap = null;
            Mat rgba = new Mat();
            Mat targetGray = new Mat();
            Mat targetDescriptors = new Mat();
            MatOfKeyPoint targetKeypoints = new MatOfKeyPoint();

            try (InputStream in = AppUtil.getDefContext()
                    .getAssets().open(assetName)) {

                bitmap = BitmapFactory.decodeStream(in);
                if (bitmap == null) {
                    throw new IllegalArgumentException("Bitmap decode returned null");
                }

                Utils.bitmapToMat(bitmap, rgba);
                Imgproc.cvtColor(rgba, targetGray, Imgproc.COLOR_RGBA2GRAY);

                // Resize if needed
                int tw = targetGray.cols();
                int th = targetGray.rows();
                if (tw > MAX_TARGET_DIM || th > MAX_TARGET_DIM) {
                    double scale = Math.min(
                            (double) MAX_TARGET_DIM / tw,
                            (double) MAX_TARGET_DIM / th
                    );
                    int nw = (int) Math.round(tw * scale);
                    int nh = (int) Math.round(th * scale);
                    Mat resized = new Mat();
                    Imgproc.resize(targetGray, resized,
                            new org.opencv.core.Size(nw, nh));
                    targetGray.release();
                    targetGray = resized;
                }

                orb.detectAndCompute(targetGray, emptyMask,
                        targetKeypoints, targetDescriptors);

                KeyPoint[] kpArray = targetKeypoints.toArray();

                if (targetDescriptors.empty()
                        || kpArray.length < MIN_GOOD_MATCHES) {
                    throw new IllegalArgumentException(
                            "Too few ORB features: " + kpArray.length
                    );
                }

                MatOfPoint2f corners = new MatOfPoint2f(
                        new Point(0, 0),
                        new Point(targetGray.cols(), 0),
                        new Point(targetGray.cols(), targetGray.rows()),
                        new Point(0, targetGray.rows())
                );

                template = new TemplateData(
                        labelFromAsset(assetName),
                        targetDescriptors.clone(),
                        kpArray,
                        corners
                );

                templateLabel = template.label;

            } catch (Exception e) {
                loadErrors.add(assetName + ": " + e.getMessage());
            } finally {
                targetKeypoints.release();
                targetDescriptors.release();
                targetGray.release();
                rgba.release();
                if (bitmap != null && !bitmap.isRecycled()) {
                    bitmap.recycle();
                }
            }
        }

        // ============================================================
        //  processFrame
        // ============================================================

        @Override
        public Mat processFrame(Mat frame) {
            long startNs = System.nanoTime();
            long nowMs  = System.currentTimeMillis();

            updateFrameFps(startNs);
            updateProcessingFps(startNs);

            CameraResult result;
            try {
                result = detect(frame, nowMs);
            } catch (RuntimeException ignored) {
                result = handleMiss(nowMs);
            }

            latestResult.set(result);
            processingMs = (System.nanoTime() - startNs) / 1e6;

            drawResult(frame, result);
            return frame;
        }

        // ============================================================
        //  detect — single template
        // ============================================================

        private CameraResult detect(Mat frame, long nowMs) {
            if (template == null) {
                return CameraResult.empty(nowMs);
            }

            Imgproc.cvtColor(frame, gray, Imgproc.COLOR_RGBA2GRAY);

            orb.detectAndCompute(gray, emptyMask,
                    frameKeypoints, frameDescriptors);

            if (frameDescriptors.empty()) {
                return handleMiss(nowMs);
            }

            KeyPoint[] fkArray = frameKeypoints.toArray();

            Detection detection = detectSingleTemplate(
                    template, fkArray, frameDescriptors,
                    frame.cols(), frame.rows()
            );

            if (detection == null) {
                return handleMiss(nowMs);
            }

            // Re-init filter sau miss kéo dài
            if (missStreak > 0) {
                long missAge = nowMs - missStartMs;
                if (missAge > MISS_REINIT_TIMEOUT_MS) {
                    resetPositionFilter(detection.dxPx, detection.dyPx);
                }
            }

            // Outlier check
            if (isOutlier(detection.dxPx, detection.dyPx)) {
                outlierStreak++;
                if (outlierStreak < OUTLIER_CONFIRM_FRAMES) {
                    return handleMiss(nowMs);
                }
                resetPositionFilter(detection.dxPx, detection.dyPx);
            }

            outlierStreak = 0;
            updatePositionFilter(detection.dxPx, detection.dyPx,
                    detection.confidence);

            lastRawDx = detection.dxPx;
            lastRawDy = detection.dyPx;

            telemetryRawDx         = detection.dxPx;
            telemetrySmoothedDx    = smoothedDx;
            telemetryOutlierStreak = outlierStreak;
            telemetryMissStreak    = missStreak;
            telemetryFilterInit    = filterInitialized;

            Detection stable = stabilizeDetection(detection);

            missStreak = 0;
            lastSuccessfulDetectionMs = nowMs;
            lastStableDetection = stable;

            return new CameraResult(nowMs, stable, smoothedDx, smoothedDy);
        }

        // ============================================================
        //  detectSingleTemplate — copy từ MultiTargetCamera
        // ============================================================

        private Detection detectSingleTemplate(
                TemplateData template,
                KeyPoint[] frameKeypointArray,
                Mat currentFrameDescriptors,
                int frameWidth,
                int frameHeight
        ) {
            List<MatOfDMatch> knnMatches = new ArrayList<>();
            List<DMatch> goodMatches = new ArrayList<>(40);
            MatOfPoint2f targetPointMat = new MatOfPoint2f();
            MatOfPoint2f framePointMat  = new MatOfPoint2f();
            Mat homography = null;
            Mat inlierMask = new Mat();
            MatOfPoint2f projectedCorners = new MatOfPoint2f();

            try {
                matcher.knnMatch(template.descriptors,
                        currentFrameDescriptors, knnMatches, 2);

                for (MatOfDMatch matchPair : knnMatches) {
                    DMatch[] pair = matchPair.toArray();
                    if (pair.length >= 2
                            && pair[0].distance < RATIO_TEST * pair[1].distance) {
                        goodMatches.add(pair[0]);
                    }
                }

                if (goodMatches.size() < MIN_GOOD_MATCHES) {
                    return null;
                }

                List<Point> targetPoints = new ArrayList<>(goodMatches.size());
                List<Point> framePoints  = new ArrayList<>(goodMatches.size());

                for (DMatch match : goodMatches) {
                    targetPoints.add(template.keypoints[match.queryIdx].pt);
                    framePoints.add(frameKeypointArray[match.trainIdx].pt);
                }

                targetPointMat.fromList(targetPoints);
                framePointMat.fromList(framePoints);

                homography = Calib3d.findHomography(
                        targetPointMat, framePointMat,
                        Calib3d.RANSAC, RANSAC_THRESHOLD, inlierMask
                );

                if (homography.empty()) return null;

                Core.perspectiveTransform(
                        template.corners, projectedCorners, homography
                );

                Point[] corners = projectedCorners.toArray();
                if (corners.length != 4) return null;

                double cx = (corners[0].x + corners[1].x
                           + corners[2].x + corners[3].x) / 4.0;
                double cy = (corners[0].y + corners[1].y
                           + corners[2].y + corners[3].y) / 4.0;

                if (cx < 0 || cx > frameWidth
                        || cy < 0 || cy > frameHeight) {
                    return null;
                }

                double dxPx = cx - frameWidth  / 2.0;
                double dyPx = cy - frameHeight / 2.0;
                double dist = Math.sqrt(dxPx * dxPx + dyPx * dyPx);

                int inliers = inlierMask.empty()
                        ? 0 : Core.countNonZero(inlierMask);

                double inlierRatio = goodMatches.isEmpty()
                        ? 0.0 : inliers / (double) goodMatches.size();

                double matchScore = Math.min(1.0,
                        goodMatches.size() / 30.0);

                double conf = Math.min(1.0,
                        0.7 * inlierRatio + 0.3 * matchScore);

                return new Detection(template.label,
                        cx, cy, dxPx, dyPx, dist,
                        goodMatches.size(), inliers, conf, corners);

            } finally {
                for (MatOfDMatch m : knnMatches) { m.release(); }
                targetPointMat.release();
                framePointMat.release();
                if (homography != null) homography.release();
                inlierMask.release();
                projectedCorners.release();
            }
        }

        // ============================================================
        //  Temporal filter (copy từ MultiTargetCamera)
        // ============================================================

        private boolean isOutlier(double newDx, double newDy) {
            if (!filterInitialized) return false;
            double diffX = newDx - lastRawDx;
            double diffY = newDy - lastRawDy;
            return Math.hypot(diffX, diffY) > OUTLIER_LIMIT_PX;
        }

        private void updatePositionFilter(
                double newDx, double newDy, double confidence
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

            double effectiveDeadband =
                    CENTER_DEADBAND_PX * (1.0 + 1.0 - confidence);
            if (distance <= effectiveDeadband) return;

            double baseAlpha = distance >= FAST_MOTION_THRESHOLD_PX
                    ? EMA_ALPHA_FAST : EMA_ALPHA_SLOW;
            double effectiveAlpha = baseAlpha * (0.5 + 0.5 * confidence);

            double stepX = effectiveAlpha * diffX;
            double stepY = effectiveAlpha * diffY;
            double stepDist = Math.hypot(stepX, stepY);

            if (stepDist > MAX_OUTPUT_STEP_PX) {
                double scale = MAX_OUTPUT_STEP_PX / stepDist;
                stepX *= scale;
                stepY *= scale;
            }
            smoothedDx += stepX;
            smoothedDy += stepY;
        }

        private Detection stabilizeDetection(Detection raw) {
            double shiftX = smoothedDx - raw.dxPx;
            double shiftY = smoothedDy - raw.dyPx;

            Point[] stableCorners = new Point[raw.corners.length];
            for (int i = 0; i < raw.corners.length; i++) {
                stableCorners[i] = new Point(
                        raw.corners[i].x + shiftX,
                        raw.corners[i].y + shiftY
                );
            }

            return new Detection(
                    raw.label,
                    raw.centerX + shiftX,
                    raw.centerY + shiftY,
                    smoothedDx, smoothedDy,
                    Math.hypot(smoothedDx, smoothedDy),
                    raw.goodMatches, raw.inliers,
                    raw.confidence,
                    stableCorners
            );
        }

        private CameraResult handleMiss(long nowMs) {
            if (missStreak == 0) missStartMs = nowMs;
            missStreak++;

            long ageMs = lastSuccessfulDetectionMs <= 0
                    ? Long.MAX_VALUE
                    : nowMs - lastSuccessfulDetectionMs;

            boolean canHold = lastStableDetection != null
                    && missStreak <= HOLD_MISS_FRAMES
                    && ageMs <= HOLD_MISS_TIMEOUT_MS;

            if (canHold) {
                return new CameraResult(nowMs,
                        lastStableDetection, smoothedDx, smoothedDy);
            }

            if (lastStableDetection != null
                    || missStreak > HOLD_MISS_FRAMES) {
                clearTrackingState();
            }

            return CameraResult.empty(nowMs);
        }

        private void resetPositionFilter(double rawDx, double rawDy) {
            filterInitialized = false;
            smoothedDx = rawDx;
            smoothedDy = rawDy;
            outlierStreak = 0;
        }

        private void clearTrackingState() {
            resetPositionFilter(0.0, 0.0);
            missStreak = 0;
            missStartMs = 0;
            lastStableDetection = null;
        }

        // ============================================================
        //  FPS / draw / release
        // ============================================================

        private void updateFrameFps(long nowNs) {
            if (lastFrameNs != 0) {
                double dt = (nowNs - lastFrameNs) / 1e9;
                if (dt > 0.0) {
                    double instantFps = 1.0 / dt;
                    frameFps = frameFps <= 0.0
                            ? instantFps
                            : frameFps + FPS_EMA_ALPHA * (instantFps - frameFps);
                }
            }
            lastFrameNs = nowNs;
        }

        private void updateProcessingFps(long nowNs) {
            if (lastProcessingNs != 0) {
                double dt = (nowNs - lastProcessingNs) / 1e9;
                if (dt > 0.0) {
                    double instantFps = 1.0 / dt;
                    processingFps = processingFps <= 0.0
                            ? instantFps
                            : processingFps
                            + FPS_EMA_ALPHA * (instantFps - processingFps);
                }
            }
            lastProcessingNs = nowNs;
        }

        private void drawResult(Mat frame, CameraResult result) {
            // Center crosshair
            Imgproc.circle(frame,
                    new Point(frame.cols() / 2.0, frame.rows() / 2.0),
                    4, new Scalar(255, 0, 0), -1);

            if (result.detection == null) return;

            Detection d = result.detection;
            Scalar color = new Scalar(0, 255, 0);

            for (int i = 0; i < 4; i++) {
                Imgproc.line(frame,
                        d.corners[i],
                        d.corners[(i + 1) % 4],
                        color, 2);
            }

            Imgproc.circle(frame,
                    new Point(d.centerX, d.centerY),
                    5, color, -1);

            Imgproc.putText(frame, d.label,
                    new Point(d.centerX + 5, d.centerY - 5),
                    Imgproc.FONT_HERSHEY_SIMPLEX,
                    0.4, color, 1);
        }

        void release() {
            clearTrackingState();

            gray.release();
            emptyMask.release();
            frameDescriptors.release();
            frameKeypoints.release();

            if (template != null) {
                template.release();
                template = null;
            }

            orb.clear();
            matcher.clear();
        }
    }
}
