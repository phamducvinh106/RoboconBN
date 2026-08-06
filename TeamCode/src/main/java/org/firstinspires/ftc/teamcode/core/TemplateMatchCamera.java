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
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.io.InputStream;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Template-matching camera — dùng Imgproc.matchTemplate() TM_CCOEFF_NORMED
 * thay vì ORB+RANSAC. Nhanh hơn ~10-50x, chính xác sub-pixel, lý tưởng cho
 * thi dấu realtime khi camera có góc nhìn cố dịnh và target ít xoay/scale.
 *
 * Cách dùng:
 * <pre>{@code
 *   TemplateMatchCamera cam = new TemplateMatchCamera(
 *       hwMap, "webcam1", true, "target.png");
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
public final class TemplateMatchCamera {

    public enum CameraMode { SINGLE_TARGET, MULTI_TARGET }

    public static final class CameraConfig {
        public final double confidenceThreshold, nmsIoU, minimumDistancePx, centerDeadbandPx;
        public final int frameScale, roiX, roiY, roiWidth, roiHeight, holdMissFrames;
        public final long holdTimeoutMs;

        public CameraConfig() { this(0.55, 2, 0, 0, 0, 0, 6, 300, 0.50, 12); }
        public CameraConfig(double confidenceThreshold, int frameScale, int roiX, int roiY,
                            int roiWidth, int roiHeight, int holdMissFrames, long holdTimeoutMs,
                            double nmsIoU, double minimumDistancePx) {
            if (!(confidenceThreshold >= 0 && confidenceThreshold <= 1) || frameScale < 1
                    || holdMissFrames < 0 || holdTimeoutMs < 0 || nmsIoU < 0 || nmsIoU > 1
                    || minimumDistancePx < 0) throw new IllegalArgumentException("Invalid camera config");
            this.confidenceThreshold = confidenceThreshold; this.frameScale = frameScale;
            this.roiX = roiX; this.roiY = roiY; this.roiWidth = roiWidth; this.roiHeight = roiHeight;
            this.holdMissFrames = holdMissFrames; this.holdTimeoutMs = holdTimeoutMs;
            this.nmsIoU = nmsIoU; this.minimumDistancePx = minimumDistancePx;
            this.centerDeadbandPx = 2.0;
        }
    }

    // ============================================================
    //  >>>  TUNE  <<<
    // ============================================================
    public static final int STREAM_WIDTH  = 640;
    public static final int STREAM_HEIGHT = 480;

    // ============================================================
    //  Instance
    // ============================================================

    private final OpenCvWebcam webcam;
    private final TemplateMatchPipeline pipeline;
    private final CameraMode mode;
    private final CameraConfig config;

    private volatile String cameraState = "CREATED";
    private volatile int    cameraErrorCode;

    // ============================================================
    //  Constructor
    // ============================================================

    public TemplateMatchCamera(
            HardwareMap hardwareMap,
            String webcamName,
            boolean showPreview,
            String targetAssetName
    ) {
        this(hardwareMap, webcamName, showPreview, targetAssetName,
                CameraMode.SINGLE_TARGET, new CameraConfig());
    }

    public TemplateMatchCamera(HardwareMap hardwareMap, String webcamName, boolean showPreview,
                               String targetAssetName, CameraMode mode, CameraConfig config) {
        if (mode == null || config == null) throw new IllegalArgumentException("mode/config required");
        this.mode = mode;
        this.config = config;
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

        pipeline = new TemplateMatchPipeline(targetAssetName, mode, config);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(3000);
    }

    /** Test constructor — không cần FTC hardware. */
    TemplateMatchCamera(String targetAssetName) {
        this.webcam = null;
        this.mode = CameraMode.SINGLE_TARGET;
        this.config = new CameraConfig();
        this.pipeline = new TemplateMatchPipeline(targetAssetName, mode, config);
    }

    public CameraMode getMode() { return mode; }
    public CameraConfig getConfig() { return config; }

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
                                    STREAM_WIDTH, STREAM_HEIGHT,
                                    OpenCvCameraRotation.UPRIGHT);
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
        String prevState = cameraState;
        cameraState = "STOPPING";
        if ("STREAMING".equals(prevState)) {
            try { if (webcam != null) webcam.stopStreaming(); } catch (RuntimeException ignored) {}
        }
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

    public double  getRawDx()           { return pipeline.telemetryRawDx; }
    public double  getSmoothedDx()      { return pipeline.telemetrySmoothedDx; }
    public int     getOutlierStreak()   { return pipeline.telemetryOutlierStreak; }
    public int     getMissStreak()      { return pipeline.telemetryMissStreak; }
    public boolean isFilterInitialized(){ return pipeline.telemetryFilterInit; }
    public String  getActiveLabel()     { return pipeline.templateLabel; }

    public void setTarget(String assetName) {
        pipeline.setTarget(assetName);
    }

    TemplateMatchPipeline getPipeline() {
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
        public final double confidence;
        public final Point[] corners;

        Detection(String label,
                  double centerX, double centerY,
                  double dxPx, double dyPx,
                  double distanceToCenter,
                  double confidence,
                  Point[] corners) {
            this.label = label;
            this.centerX = centerX; this.centerY = centerY;
            this.dxPx = dxPx; this.dyPx = dyPx;
            this.distanceToCenter = distanceToCenter;
            this.confidence = confidence;
            this.corners = corners.clone();
        }
    }

    public static final class CameraResult {
        public final long      timestampMs;
        public final Detection detection;
        public final List<Detection> detections;
        public final double dxPx;
        public final double dyPx;
        public final double centerX, centerY, confidence;
        public final long staleAgeMs;

        CameraResult(long timestampMs, Detection detection,
                     double dxPx, double dyPx) {
            this(timestampMs, detection, dxPx, dyPx, timestampMs, detection == null ? Double.NaN : detection.confidence);
        }

        CameraResult(long timestampMs, Detection detection, double dxPx, double dyPx,
                     List<Detection> detections, double confidence) {
            this.timestampMs = timestampMs;
            this.detection = detection;
            this.detections = Collections.unmodifiableList(new ArrayList<>(detections));
            this.dxPx = dxPx; this.dyPx = dyPx;
            this.centerX = detection == null ? Double.NaN : detection.centerX;
            this.centerY = detection == null ? Double.NaN : detection.centerY;
            this.confidence = confidence;
            this.staleAgeMs = 0;
        }

        CameraResult(long timestampMs, Detection detection, double dxPx, double dyPx,
                     long lastSuccessMs, double confidence) {
            this.timestampMs = timestampMs;
            this.detection = detection;
            this.detections = detection == null ? Collections.<Detection>emptyList()
                    : Collections.singletonList(detection);
            this.dxPx = dxPx; this.dyPx = dyPx;
            this.centerX = detection == null ? Double.NaN : detection.centerX;
            this.centerY = detection == null ? Double.NaN : detection.centerY;
            this.confidence = confidence;
            this.staleAgeMs = lastSuccessMs <= 0 ? Long.MAX_VALUE : Math.max(0, timestampMs - lastSuccessMs);
        }

        public boolean isValid() { return detection != null; }

        static CameraResult empty(long timestampMs) {
            return new CameraResult(timestampMs, null, Double.NaN, Double.NaN);
        }
    }

    public static final class Candidate {
        public final String label;
        public final double confidence;
        public final Point[] corners;
        public final double centerX, centerY;
        public final int order;

        public Candidate(String label, double confidence, Point[] corners, int order) {
            if (label == null || corners == null || corners.length != 4
                    || !Double.isFinite(confidence) || !Double.isFinite(order))
                throw new IllegalArgumentException("Invalid candidate");
            double minX = Double.POSITIVE_INFINITY, minY = Double.POSITIVE_INFINITY;
            double maxX = Double.NEGATIVE_INFINITY, maxY = Double.NEGATIVE_INFINITY;
            for (Point p : corners) {
                if (p == null || !Double.isFinite(p.x) || !Double.isFinite(p.y))
                    throw new IllegalArgumentException("Invalid candidate geometry");
                minX = Math.min(minX, p.x); minY = Math.min(minY, p.y);
                maxX = Math.max(maxX, p.x); maxY = Math.max(maxY, p.y);
            }
            if (!(maxX > minX && maxY > minY)) throw new IllegalArgumentException("Invalid candidate bounds");
            this.label = label; this.confidence = confidence; this.corners = corners.clone();
            this.centerX = (minX + maxX) / 2.0; this.centerY = (minY + maxY) / 2.0;
            this.order = order;
        }
    }

    public static List<Candidate> suppressCandidates(List<Candidate> input, double iouThreshold,
                                                       double minimumDistancePx) {
        if (input == null || !(iouThreshold >= 0 && iouThreshold <= 1)
                || !(minimumDistancePx >= 0)) throw new IllegalArgumentException("Invalid suppression policy");
        List<Candidate> sorted = new ArrayList<>(input);
        sorted.sort(Comparator.comparingDouble((Candidate c) -> c.confidence).reversed()
                .thenComparingInt(c -> c.order));
        List<Candidate> kept = new ArrayList<>();
        for (Candidate candidate : sorted) {
            boolean suppressed = false;
            for (Candidate existing : kept) {
                if (iou(candidate, existing) >= iouThreshold
                        || Math.hypot(candidate.centerX - existing.centerX,
                                      candidate.centerY - existing.centerY) < minimumDistancePx) {
                    suppressed = true; break;
                }
            }
            if (!suppressed) kept.add(candidate);
        }
        return Collections.unmodifiableList(kept);
    }

    private static double iou(Candidate a, Candidate b) {
        double ax1 = a.corners[0].x, ay1 = a.corners[0].y;
        double ax2 = a.corners[2].x, ay2 = a.corners[2].y;
        double bx1 = b.corners[0].x, by1 = b.corners[0].y;
        double bx2 = b.corners[2].x, by2 = b.corners[2].y;
        double ix = Math.max(0, Math.min(ax2, bx2) - Math.max(ax1, bx1));
        double iy = Math.max(0, Math.min(ay2, by2) - Math.max(ay1, by1));
        double inter = ix * iy;
        double areaA = (ax2 - ax1) * (ay2 - ay1), areaB = (bx2 - bx1) * (by2 - by1);
        return inter / (areaA + areaB - inter);
    }

    // ============================================================
    //  TemplateData
    // ============================================================

    private static final class TemplateData {
        final String        label;
        final Mat           grayTemplate;
        final int           width, height;
        final MatOfPoint2f  corners;

        TemplateData(String label, Mat grayTemplate,
                     int width, int height, MatOfPoint2f corners) {
            this.label = label;
            this.grayTemplate = grayTemplate;
            this.width = width; this.height = height;
            this.corners = corners;
        }

        void release() {
            grayTemplate.release();
            corners.release();
        }
    }

    // ============================================================
    //  Pipeline (template-matching)
    // ============================================================

    static final class TemplateMatchPipeline extends OpenCvPipeline {

        // ---------- Detection ----------
        private final CameraMode mode;
        private final CameraConfig config;
        private static final double MIN_CONFIDENCE = 0.55;

        // ---------- Downscale frame 2x before matchTemplate ----------
        private static final int    FRAME_SCALE     = 2;
        private static final double FRAME_SCALE_INV = 1.0 / FRAME_SCALE;

        // ---------- Target resize ----------
        private static final int MAX_TARGET_DIM = 200;

        // ---------- Temporal filter ----------
        private static final int    OUTLIER_CONFIRM_FRAMES  = 4;
        private static final double OUTLIER_LIMIT_PX        = 45.0;
        private static final int    HOLD_MISS_FRAMES        = 6;
        private static final long   HOLD_MISS_TIMEOUT_MS    = 300;
        private static final long   MISS_REINIT_TIMEOUT_MS  = 250;

        private static final double CENTER_DEADBAND_PX      = 2.0;
        private static final double EMA_ALPHA_SLOW          = 0.08;
        private static final double EMA_ALPHA_FAST          = 0.45;
        private static final double FAST_MOTION_THRESHOLD_PX = 8.0;
        private static final double MAX_OUTPUT_STEP_PX      = 20.0;
        private static final double FPS_EMA_ALPHA           = 0.20;

        // ============================================================
        //  State
        // ============================================================

        private TemplateData template;
        private String       templateLabel = "";
        private final List<TemplateData> templates = new ArrayList<>();

        private final List<String> loadErrors = new ArrayList<>();

        private final AtomicReference<CameraResult> latestResult =
                new AtomicReference<>(CameraResult.empty(0));

        // Reusable Mats
        private final Mat gray        = new Mat();    // 640x480 grayscale
        private final Mat smallGray   = new Mat();    // 320x240 downscaled
        private Mat       smallTemplate;              // downscaled template (allocated once)
        private Mat       matchResult;                // allocated on first use

        // Shutdown guard — prevents processFrame() after release()
        private volatile boolean running = true;

        // Temporal filter state
        private boolean   filterInitialized;
        private double    smoothedDx, smoothedDy;
        private double    lastRawDx, lastRawDy;
        private int       missStreak, outlierStreak;
        private long      missStartMs;

        private Detection lastStableDetection;

        private long lastFrameNs, lastProcessingNs;

        volatile double  frameFps, processingFps, processingMs;
        volatile long    lastSuccessfulDetectionMs;

        volatile double  telemetryRawDx;
        volatile double  telemetrySmoothedDx;
        volatile int     telemetryOutlierStreak;
        volatile int     telemetryMissStreak;
        volatile boolean telemetryFilterInit;

        // ============================================================
        //  Init
        // ============================================================

        TemplateMatchPipeline(String targetAssetName, CameraMode mode, CameraConfig config) {
            this.mode = mode;
            this.config = config;
            if (targetAssetName != null && !targetAssetName.trim().isEmpty()) {
                loadTemplate(targetAssetName);
            }
            if (mode == CameraMode.MULTI_TARGET) {
                String[] defaults = {"target.png", "target2.png", "target3.png", "target4.png"};
                for (String asset : defaults) {
                    if (!asset.equals(targetAssetName)) loadTemplate(asset);
                }
                if (!templates.isEmpty()) {
                    template = templates.get(0);
                    templateLabel = template.label;
                }
            }
        }

        void setTarget(String assetName) {
            clearTrackingState();
            synchronized (this) {
                if (template != null) {
                    template.release();
                    template = null;
                }
                if (smallTemplate != null) {
                    smallTemplate.release();
                    smallTemplate = null;
                }
            }
            templateLabel = "";
            loadErrors.clear();
            if (assetName != null && !assetName.trim().isEmpty()) {
                loadTemplate(assetName);
            }
        }

        private static String labelFromAsset(String assetName) {
            int slash = Math.max(assetName.lastIndexOf('/'),
                                 assetName.lastIndexOf('\\'));
            String fn = slash >= 0 ? assetName.substring(slash + 1) : assetName;
            int dot = fn.lastIndexOf('.');
            return dot > 0 ? fn.substring(0, dot) : fn;
        }

        private void loadTemplate(String assetName) {
            if (assetName == null || assetName.trim().isEmpty()) {
                loadErrors.add("Empty target asset name");
                return;
            }

            Bitmap bitmap = null;
            Mat rgba = new Mat();
            Mat tGray = new Mat();

            try (InputStream in = AppUtil.getDefContext()
                    .getAssets().open(assetName)) {

                bitmap = BitmapFactory.decodeStream(in);
                if (bitmap == null)
                    throw new IllegalArgumentException("Bitmap decode returned null");

                Utils.bitmapToMat(bitmap, rgba);
                Imgproc.cvtColor(rgba, tGray, Imgproc.COLOR_RGBA2GRAY);

                int tw = tGray.cols();
                int th = tGray.rows();
                if (tw > MAX_TARGET_DIM || th > MAX_TARGET_DIM) {
                    double scale = Math.min(
                            (double) MAX_TARGET_DIM / tw,
                            (double) MAX_TARGET_DIM / th);
                    int nw = (int) Math.round(tw * scale);
                    int nh = (int) Math.round(th * scale);
                    Mat resized = new Mat();
                    Imgproc.resize(tGray, resized,
                            new org.opencv.core.Size(nw, nh));
                    tGray.release();
                    tGray = resized;
                    tw = nw; th = nh;
                }

                if (tGray.empty())
                    throw new IllegalArgumentException("Template is empty");

                MatOfPoint2f corners = new MatOfPoint2f(
                        new Point(0, 0),
                        new Point(tw, 0),
                        new Point(tw, th),
                        new Point(0, th));

                // Clone template — không share Mat trong finalizer
                TemplateData loaded = new TemplateData(
                        labelFromAsset(assetName),
                        tGray.clone(),
                        tw, th, corners);
                templates.add(loaded);
                template = loaded;

                templateLabel = template.label;

                // Pre-compute downscaled template (2x smaller)
                if (smallTemplate != null) smallTemplate.release();
                int stw = tw / FRAME_SCALE;
                int sth = th / FRAME_SCALE;
                if (stw >= 4 && sth >= 4) {
                    smallTemplate = new Mat();
                    Imgproc.resize(tGray, smallTemplate,
                            new org.opencv.core.Size(stw, sth),
                            0, 0, Imgproc.INTER_AREA);
                } else {
                    smallTemplate = null;  // template too small to downscale
                }

                loadErrors.clear();

            } catch (Exception e) {
                loadErrors.add(assetName + ": " + e.getMessage());
            } finally {
                tGray.release();
                rgba.release();
                if (bitmap != null && !bitmap.isRecycled()) bitmap.recycle();
            }
        }

        // ============================================================
        //  processFrame
        // ============================================================

        @Override
        public Mat processFrame(Mat frame) {
            if (!running) return frame;

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
        //  detect — template matching
        // ============================================================

        private CameraResult detect(Mat frame, long nowMs) {
            if (mode == CameraMode.MULTI_TARGET) return detectMulti(frame, nowMs);
            TemplateData current;
            synchronized (this) {
                current = template;
                if (current == null)
                    return CameraResult.empty(nowMs);
            }

            Imgproc.cvtColor(frame, gray, Imgproc.COLOR_RGBA2GRAY);

            // Downscale frame 2x
            Imgproc.pyrDown(gray, smallGray);

            // Use downscaled template (or fall back to full-size)
            Mat tpl;
            int tplW, tplH;
            if (smallTemplate != null && !smallTemplate.empty()) {
                tpl  = smallTemplate;
                tplW = smallTemplate.cols();
                tplH = smallTemplate.rows();
            } else {
                // Graceful fallback: use full-size template
                tpl  = current.grayTemplate;
                tplW = current.width;
                tplH = current.height;
            }

            int scaledW = smallGray.cols();
            int scaledH = smallGray.rows();

            int resultW = scaledW - tplW + 1;
            int resultH = scaledH - tplH + 1;
            if (resultW <= 0 || resultH <= 0)
                return handleMiss(nowMs);

            if (matchResult == null
                    || matchResult.cols() != resultW
                    || matchResult.rows() != resultH) {
                if (matchResult != null) matchResult.release();
                matchResult = new Mat(resultH, resultW, smallGray.type());
            }

            // Core: template match on downscaled frame
            Imgproc.matchTemplate(smallGray, tpl,
                    matchResult, Imgproc.TM_CCOEFF_NORMED);

            // Multi-peak: find all candidates, pick closest to center Y
            double centerY_scaled = scaledH / 2.0;
            double bestConfidence = -1;
            int    bestPx = -1, bestPy = -1;
            double bestDistY = Double.MAX_VALUE;

            Mat search = matchResult.clone();
            while (true) {
                Core.MinMaxLocResult mm = Core.minMaxLoc(search);
                if (mm.maxVal < MIN_CONFIDENCE) break;

                int px = (int) Math.round(mm.maxLoc.x);
                int py = (int) Math.round(mm.maxLoc.y);
                double candCenterY = py + tplH / 2.0;
                double distY = Math.abs(candCenterY - centerY_scaled);

                if (distY < bestDistY) {
                    bestDistY = distY;
                    bestPx = px;
                    bestPy = py;
                    bestConfidence = mm.maxVal;
                }

                // Suppress found region (2x template to skip partial overlaps)
                int sx = Math.max(0, px - tplW);
                int sy = Math.max(0, py - tplH);
                int sw = Math.min(search.cols() - sx, tplW * 2);
                int sh = Math.min(search.rows() - sy, tplH * 2);
                Imgproc.rectangle(search,
                        new Point(sx, sy),
                        new Point(sx + sw - 1, sy + sh - 1),
                        new Scalar(0), Core.FILLED);
            }
            search.release();

            if (bestPx < 0)
                return handleMiss(nowMs);

            // Sub-pixel refinement on the selected peak
            double topLeftX_scaled = refinePeakX(matchResult, bestPx, bestPy);
            double topLeftY_scaled = refinePeakY(matchResult, bestPx, bestPy);

            // Scale back to original resolution
            double topLeftX = topLeftX_scaled * FRAME_SCALE;
            double topLeftY = topLeftY_scaled * FRAME_SCALE;

            double cx = topLeftX + current.width  / 2.0;
            double cy = topLeftY + current.height / 2.0;

            if (cx < 0 || cx > frame.cols() || cy < 0 || cy > frame.rows())
                return handleMiss(nowMs);

            double dxPx = cx - frame.cols()  / 2.0;
            double dyPx = cy - frame.rows() / 2.0;
            double dist = Math.hypot(dxPx, dyPx);

            Point[] corners = new Point[] {
                    new Point(topLeftX, topLeftY),
                    new Point(topLeftX + current.width, topLeftY),
                    new Point(topLeftX + current.width, topLeftY + current.height),
                    new Point(topLeftX, topLeftY + current.height)
            };

            Detection d = new Detection(current.label,
                    cx, cy, dxPx, dyPx, dist,
                    bestConfidence, corners);

            // ---- Temporal filter ----

            if (missStreak > 0) {
                long missAge = nowMs - missStartMs;
                if (missAge > MISS_REINIT_TIMEOUT_MS)
                    resetPositionFilter(d.dxPx, d.dyPx);
            }

            if (isOutlier(d.dxPx, d.dyPx)) {
                outlierStreak++;
                if (outlierStreak < OUTLIER_CONFIRM_FRAMES)
                    return handleMiss(nowMs);
                resetPositionFilter(d.dxPx, d.dyPx);
            }

            outlierStreak = 0;
            updatePositionFilter(d.dxPx, d.dyPx, d.confidence);

            lastRawDx = d.dxPx;
            lastRawDy = d.dyPx;

            telemetryRawDx      = d.dxPx;
            telemetrySmoothedDx = smoothedDx;
            telemetryOutlierStreak = outlierStreak;
            telemetryMissStreak    = missStreak;
            telemetryFilterInit    = filterInitialized;

            Detection stable = stabilizeDetection(d);

            missStreak = 0;
            lastSuccessfulDetectionMs = nowMs;
            lastStableDetection = stable;

            return new CameraResult(nowMs, stable, smoothedDx, smoothedDy);
        }

        private CameraResult detectMulti(Mat frame, long nowMs) {
            if (templates.isEmpty()) return handleMiss(nowMs);
            Imgproc.cvtColor(frame, gray, Imgproc.COLOR_RGBA2GRAY);
            Imgproc.pyrDown(gray, smallGray);
            List<Candidate> candidates = new ArrayList<>();
            int order = 0;
            for (TemplateData data : templates) {
                int tw = Math.max(1, data.width / FRAME_SCALE);
                int th = Math.max(1, data.height / FRAME_SCALE);
                Mat tpl = new Mat();
                Imgproc.resize(data.grayTemplate, tpl, new org.opencv.core.Size(tw, th), 0, 0, Imgproc.INTER_AREA);
                int rw = smallGray.cols() - tw + 1, rh = smallGray.rows() - th + 1;
                if (rw > 0 && rh > 0) {
                    Mat matches = new Mat();
                    Imgproc.matchTemplate(smallGray, tpl, matches, Imgproc.TM_CCOEFF_NORMED);
                    Mat search = matches.clone();
                    while (true) {
                        Core.MinMaxLocResult mm = Core.minMaxLoc(search);
                        if (mm.maxVal < config.confidenceThreshold) break;
                        int px = (int) Math.round(mm.maxLoc.x), py = (int) Math.round(mm.maxLoc.y);
                        double x = px * FRAME_SCALE, y = py * FRAME_SCALE;
                        candidates.add(new Candidate(data.label, mm.maxVal, new Point[]{
                                new Point(x, y), new Point(x + data.width, y),
                                new Point(x + data.width, y + data.height), new Point(x, y + data.height)
                        }, order++));
                        int sx = Math.max(0, px - tw), sy = Math.max(0, py - th);
                        int sw = Math.min(search.cols() - sx, tw * 2), sh = Math.min(search.rows() - sy, th * 2);
                        Imgproc.rectangle(search, new Point(sx, sy), new Point(sx + sw - 1, sy + sh - 1), new Scalar(0), Core.FILLED);
                    }
                    search.release(); matches.release();
                }
                tpl.release();
            }
            List<Candidate> kept = suppressCandidates(candidates, config.nmsIoU, config.minimumDistancePx);
            if (kept.isEmpty()) return handleMiss(nowMs);
            List<Detection> detections = new ArrayList<>();
            double frameCx = frame.cols() / 2.0, frameCy = frame.rows() / 2.0;
            for (Candidate c : kept) {
                double dx = c.centerX - frameCx, dy = c.centerY - frameCy;
                detections.add(new Detection(c.label, c.centerX, c.centerY, dx, dy,
                        Math.hypot(dx, dy), c.confidence, c.corners));
            }
            Detection primary = detections.get(0);
            lastSuccessfulDetectionMs = nowMs;
            latestResult.set(new CameraResult(nowMs, primary, primary.dxPx, primary.dyPx,
                    detections, primary.confidence));
            return new CameraResult(nowMs, primary, primary.dxPx, primary.dyPx,
                    detections, primary.confidence);
        }

        // ============================================================
        //  Sub-pixel refinement
        // ============================================================

        private double refinePeakX(Mat result, int px, int py) {
            int w = result.cols();
            if (px <= 0 || px >= w - 1) return px;
            double l = result.get(py, px - 1)[0];
            double c = result.get(py, px)[0];
            double r = result.get(py, px + 1)[0];
            if (c <= l || c <= r) return px;
            return px + (l - r) / (2.0 * (l + r - 2.0 * c));
        }

        private double refinePeakY(Mat result, int px, int py) {
            int h = result.rows();
            if (py <= 0 || py >= h - 1) return py;
            double t = result.get(py - 1, px)[0];
            double c = result.get(py, px)[0];
            double b = result.get(py + 1, px)[0];
            if (c <= t || c <= b) return py;
            return py + (t - b) / (2.0 * (t + b - 2.0 * c));
        }

        // ============================================================
        //  Temporal filter
        // ============================================================

        private boolean isOutlier(double newDx, double newDy) {
            if (!filterInitialized) return false;
            return Math.hypot(newDx - lastRawDx, newDy - lastRawDy)
                    > OUTLIER_LIMIT_PX;
        }

        private void updatePositionFilter(
                double newDx, double newDy, double confidence) {
            if (!filterInitialized) {
                smoothedDx = newDx; smoothedDy = newDy;
                filterInitialized = true;
                return;
            }
            double diffX = newDx - smoothedDx;
            double diffY = newDy - smoothedDy;
            double distance = Math.hypot(diffX, diffY);

            double deadband = CENTER_DEADBAND_PX * (1.0 + 1.0 - confidence);
            if (distance <= deadband) return;

            double baseAlpha = distance >= FAST_MOTION_THRESHOLD_PX
                    ? EMA_ALPHA_FAST : EMA_ALPHA_SLOW;
            double effAlpha = baseAlpha * (0.5 + 0.5 * confidence);

            double stepX = effAlpha * diffX;
            double stepY = effAlpha * diffY;
            double stepDist = Math.hypot(stepX, stepY);

            if (stepDist > MAX_OUTPUT_STEP_PX) {
                double s = MAX_OUTPUT_STEP_PX / stepDist;
                stepX *= s; stepY *= s;
            }
            smoothedDx += stepX; smoothedDy += stepY;
        }

        private Detection stabilizeDetection(Detection raw) {
            double shiftX = smoothedDx - raw.dxPx;
            double shiftY = smoothedDy - raw.dyPx;

            Point[] stableCorners = new Point[raw.corners.length];
            for (int i = 0; i < raw.corners.length; i++)
                stableCorners[i] = new Point(
                        raw.corners[i].x + shiftX,
                        raw.corners[i].y + shiftY);

            return new Detection(raw.label,
                    raw.centerX + shiftX,
                    raw.centerY + shiftY,
                    smoothedDx, smoothedDy,
                    Math.hypot(smoothedDx, smoothedDy),
                    raw.confidence, stableCorners);
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

            if (canHold)
                return new CameraResult(nowMs, lastStableDetection,
                        smoothedDx, smoothedDy);

            if (lastStableDetection != null
                    || missStreak > HOLD_MISS_FRAMES)
                clearTrackingState();

            return CameraResult.empty(nowMs);
        }

        private void resetPositionFilter(double rawDx, double rawDy) {
            filterInitialized = false;
            smoothedDx = rawDx; smoothedDy = rawDy;
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
                    double inst = 1.0 / dt;
                    frameFps = frameFps <= 0.0
                            ? inst
                            : frameFps + FPS_EMA_ALPHA * (inst - frameFps);
                }
            }
            lastFrameNs = nowNs;
        }

        private void updateProcessingFps(long nowNs) {
            if (lastProcessingNs != 0) {
                double dt = (nowNs - lastProcessingNs) / 1e9;
                if (dt > 0.0) {
                    double inst = 1.0 / dt;
                    processingFps = processingFps <= 0.0
                            ? inst
                            : processingFps + FPS_EMA_ALPHA * (inst - processingFps);
                }
            }
            lastProcessingNs = nowNs;
        }

        private void drawResult(Mat frame, CameraResult result) {
            Imgproc.circle(frame,
                    new Point(frame.cols() / 2.0, frame.rows() / 2.0),
                    4, new Scalar(255, 0, 0), -1);

            for (Detection d : result.detections) {
                Scalar color = d == result.detection ? new Scalar(0, 255, 0) : new Scalar(0, 180, 255);
                for (int i = 0; i < 4; i++)
                    Imgproc.line(frame, d.corners[i], d.corners[(i + 1) % 4], color, 2);
                Imgproc.circle(frame, new Point(d.centerX, d.centerY), 5, color, -1);
                Imgproc.putText(frame, d.label, new Point(d.centerX + 5, d.centerY - 5),
                        Imgproc.FONT_HERSHEY_SIMPLEX, 0.4, color, 1);
            }
        }

        void release() {
            running = false;
            clearTrackingState();
            for (TemplateData data : templates) data.release();
            templates.clear();
            template = null;
            gray.release();
            smallGray.release();
            if (matchResult != null) { matchResult.release(); matchResult = null; }
            if (smallTemplate != null) { smallTemplate.release(); smallTemplate = null; }
            if (template != null) { template.release(); template = null; }
        }
    }
}
