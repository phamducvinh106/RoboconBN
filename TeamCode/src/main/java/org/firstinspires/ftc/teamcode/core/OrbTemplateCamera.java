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
import org.opencv.core.Mat;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.features2d.DescriptorMatcher;
import org.opencv.features2d.ORB;
import org.opencv.imgproc.Imgproc;

import java.io.InputStream;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/** One webcam lifecycle with one immutable ORB target template and verified geometry. */
public final class OrbTemplateCamera {
    public enum Mode { SINGLE_TARGET, MULTI_TARGET }
    public enum CameraId { WEBCAM1, WEBCAM2 }
    public enum State { CREATED, OPENING, STREAMING, STOPPING, CLOSED, ERROR }
    public enum DetectionState { SEARCHING, LOCKED, COASTING, LOST }

    public static final int STREAM_WIDTH = 640, STREAM_HEIGHT = 480;
    public static final int MAX_FEATURES = 400, MAX_PYRAMID_LEVELS = 6, FAST_THRESHOLD = 25;
    public static final int MAX_MATCHES = 80;
    public static final int MIN_GOOD_MATCHES = 12, MIN_INLIERS = 10;
    public static final int MAX_ROI_WIDTH = 480, MAX_ROI_HEIGHT = 360;
    public static final long MAX_RESULT_AGE_MS = 120, MAX_FRAME_LATENCY_MS = 100;
    public static final int ACQUIRE_FRAMES = 3, LOSE_MISS_FRAMES = 3;
    public static final double RATIO = 0.70, MAX_HAMMING = 60, MIN_INLIER_RATIO = 0.55;
    public static final double RANSAC_REPROJECTION = 3.0, MAX_REPROJECTION_ERROR = 3.0;
    public static final double MIN_TEMPLATE_COVERAGE = 0.35, EMA_ALPHA = 0.3;
    public static final double MAX_PIXELS_PER_SECOND = 180.0;

    public static final class Result {
        public final CameraId cameraId;
        public final Mode mode;
        public final String targetId;
        public final State state;
        public final DetectionState detectionState;
        public final long timestampMs;
        public final boolean valid, authorizesMovement, geometryValid;
        public final double centerX, centerY, dxPx, dyPx, confidence, processingMs, fps;

        private Result(CameraId cameraId, Mode mode, String targetId, State state, DetectionState detectionState,
                       long timestampMs, boolean valid, boolean authorized, boolean geometryValid,
                       double x, double y, double dx, double dy, double confidence, double processingMs, double fps) {
            this.cameraId = cameraId;
            this.mode = mode;
            this.targetId = targetId;
            this.state = state;
            this.detectionState = detectionState;
            this.timestampMs = timestampMs;
            this.valid = valid;
            authorizesMovement = authorized;
            this.geometryValid = geometryValid;
            centerX = x;
            centerY = y;
            dxPx = dx;
            dyPx = dy;
            this.confidence = confidence;
            this.processingMs = processingMs;
            this.fps = fps;
        }

        static Result invalid(CameraId id, Mode mode, String target, State state) {
            return new Result(id, mode, target, state, DetectionState.LOST, System.currentTimeMillis(),
                    false, false, false, Double.NaN, Double.NaN, Double.NaN, Double.NaN, 0, 0, 0);
        }
    }

    private final OpenCvWebcam webcam;
    private final Pipeline pipeline;
    private final CameraId cameraId;
    private final Mode mode;
    private final String targetId;
    private volatile State state = State.CREATED;
    private volatile int errorCode;
    private long generation;

    public OrbTemplateCamera(HardwareMap map, String cameraName, boolean preview, Mode mode, String targetId, Mat template) {
        if (map == null || mode == null || targetId == null || targetId.isEmpty() || template == null || template.empty()) {
            throw new IllegalArgumentException("validated target required");
        }
        cameraId = "webcam1".equals(cameraName) ? CameraId.WEBCAM1 : "webcam2".equals(cameraName) ? CameraId.WEBCAM2 : failCamera(cameraName);
        if (mode == Mode.SINGLE_TARGET && cameraId != CameraId.WEBCAM1) {
            throw new IllegalArgumentException("SINGLE_TARGET requires webcam1");
        }
        this.mode = mode;
        this.targetId = targetId;
        WebcamName camera = map.get(WebcamName.class, cameraName);
        int view = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName());
        webcam = preview ? OpenCvCameraFactory.getInstance().createWebcam(camera, view) : OpenCvCameraFactory.getInstance().createWebcam(camera);
        pipeline = new Pipeline(cameraId, mode, targetId, template);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(3000);
    }

    public static Mat loadAsset(String assetName) {
        if (assetName == null || assetName.trim().isEmpty()) throw new IllegalArgumentException("target asset required");
        try (InputStream input = AppUtil.getDefContext().getAssets().open(assetName)) {
            Bitmap bitmap = BitmapFactory.decodeStream(input);
            if (bitmap == null) throw new IllegalArgumentException("invalid target asset: " + assetName);
            Mat rgba = new Mat();
            Utils.bitmapToMat(bitmap, rgba);
            bitmap.recycle();
            return rgba;
        } catch (Exception error) {
            throw new IllegalArgumentException("cannot load target asset " + assetName, error);
        }
    }

    private static CameraId failCamera(String name) {
        throw new IllegalArgumentException("camera must be webcam1 or webcam2: " + name);
    }

    public synchronized void startAsync() {
        if (state == State.OPENING || state == State.STREAMING) return;
        final long token = ++generation;
        state = State.OPENING;
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                synchronized (OrbTemplateCamera.this) {
                    if (token != generation || state != State.OPENING) return;
                    try {
                        webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                        state = State.STREAMING;
                    } catch (RuntimeException e) {
                        fail(token, -1);
                    }
                }
            }
            @Override public void onError(int code) {
                synchronized (OrbTemplateCamera.this) {
                    fail(token, code);
                }
            }
        });
    }

    private void fail(long token, int code) {
        if (token != generation) return;
        errorCode = code;
        state = State.ERROR;
        pipeline.invalidate(state);
    }

    public synchronized void stop() {
        if (state == State.CLOSED || state == State.STOPPING) return;
        ++generation;
        state = State.STOPPING;
        pipeline.invalidate(state);
        try { webcam.stopStreaming(); } catch (RuntimeException ignored) {}
        try { webcam.closeCameraDevice(); } catch (RuntimeException ignored) {}
        pipeline.release();
        state = State.CLOSED;
    }

    public Result getLatestResult() {
        Result r = pipeline.latest.get();
        return r.state == state ? r : Result.invalid(cameraId, mode, targetId, state);
    }

    public State getCameraState() { return state; }
    public int getCameraErrorCode() { return errorCode; }
    public double getProcessingMs() { return pipeline.processingMs; }
    public boolean templateLoaded() { return pipeline.templateDescriptors != null && !pipeline.templateDescriptors.empty(); }

    public static boolean fresh(Result r, long now) {
        return r != null && r.state == State.STREAMING && r.authorizesMovement
                && now >= r.timestampMs && now - r.timestampMs <= MAX_RESULT_AGE_MS
                && Double.isFinite(r.dxPx) && Double.isFinite(r.dyPx);
    }

    private static final class Pipeline extends OpenCvPipeline {
        private final CameraId cameraId;
        private final Mode mode;
        private final String targetId;
        private final AtomicReference<Result> latest = new AtomicReference<>();
        private final ORB orb = ORB.create(MAX_FEATURES, 1.2f, MAX_PYRAMID_LEVELS, 31, 0, 2, ORB.HARRIS_SCORE, 31, FAST_THRESHOLD);
        private final DescriptorMatcher matcher = DescriptorMatcher.create(DescriptorMatcher.BRUTEFORCE_HAMMING);
        private final Mat gray = new Mat(), roi = new Mat(), templateGray = new Mat();
        private final MatOfKeyPoint templateKeys = new MatOfKeyPoint();
        private Mat templateDescriptors;
        private final int templateWidth, templateHeight;
        private boolean released;
        volatile double processingMs, fps;
        private long lastFrameNs, lastFrameMs;

        DetectionState detectionState = DetectionState.SEARCHING;
        int consecutiveValid = 0, consecutiveMiss = 0;
        double filteredX = Double.NaN, filteredY = Double.NaN;
        final Point[] medianBuf = new Point[3];
        int medianIdx = 0;
        Result coastingHold;

        Pipeline(CameraId id, Mode mode, String targetId, Mat template) {
            cameraId = id;
            this.mode = mode;
            this.targetId = targetId;
            if (template.channels() == 1) template.copyTo(templateGray);
            else Imgproc.cvtColor(template, templateGray, template.channels() == 4 ? Imgproc.COLOR_RGBA2GRAY : Imgproc.COLOR_RGB2GRAY);
            templateWidth = templateGray.cols();
            templateHeight = templateGray.rows();
            orb.detect(templateGray, templateKeys);
            templateDescriptors = new Mat();
            orb.compute(templateGray, templateKeys, templateDescriptors);
            templateGray.release();
            if (templateDescriptors.empty()) throw new IllegalArgumentException("template has no ORB descriptors");
            invalidate(State.CREATED);
        }

        @Override
        public Mat processFrame(Mat input) {
            if (released || input == null || input.empty()) return input;
            long start = System.nanoTime();
            long nowMs = System.currentTimeMillis();
            double dt = lastFrameMs == 0 ? 0.033 : (nowMs - lastFrameMs) / 1000.0;
            lastFrameMs = nowMs;

            toGray(input, gray);
            int w = Math.min(gray.cols(), MAX_ROI_WIDTH), h = Math.min(gray.rows(), MAX_ROI_HEIGHT);
            Rect bounds = new Rect((gray.cols() - w) / 2, (gray.rows() - h) / 2, w, h);
            gray.submat(bounds).copyTo(roi);

            MatOfKeyPoint keys = new MatOfKeyPoint();
            Mat desc = new Mat();
            orb.detect(roi, keys);
            orb.compute(roi, keys, desc);

            GeometryHit hit = null;
            if (!desc.empty() && desc.rows() >= MIN_GOOD_MATCHES) {
                hit = matchAndVerify(keys, desc, bounds, input.cols(), input.rows());
            }

            keys.release();
            desc.release();

            long end = System.nanoTime();
            processingMs = (end - start) / 1e6;
            fps = lastFrameNs == 0 ? 0 : 1e9 / (end - lastFrameNs);
            lastFrameNs = end;

            publishResult(input, nowMs, dt, hit);
            return input;
        }

        private GeometryHit matchAndVerify(MatOfKeyPoint keys, Mat desc, Rect bounds, int frameW, int frameH) {
            List<MatOfDMatch> forward = new ArrayList<>();
            matcher.knnMatch(templateDescriptors, desc, forward, 2);
            List<MatOfDMatch> reverse = new ArrayList<>();
            matcher.knnMatch(desc, templateDescriptors, reverse, 2);

            DMatch[] selected = new DMatch[MAX_MATCHES];
            int count = 0;
            org.opencv.core.KeyPoint[] tk = templateKeys.toArray();
            org.opencv.core.KeyPoint[] rk = keys.toArray();

            for (int i = 0; i < forward.size() && count < MAX_MATCHES; i++) {
                MatOfDMatch pair = forward.get(i);
                DMatch[] ds = pair.toArray();
                if (ds.length < 2) { pair.release(); continue; }
                DMatch m = ds[0];
                if (m.distance > MAX_HAMMING || m.distance >= RATIO * ds[1].distance) { pair.release(); continue; }
                if (!mutual(m, reverse)) { pair.release(); continue; }
                selected[count++] = m;
                pair.release();
            }
            for (MatOfDMatch pair : reverse) pair.release();

            if (count < MIN_GOOD_MATCHES) return null;

            Point[] src = new Point[count], dst = new Point[count];
            for (int i = 0; i < count; i++) {
                src[i] = tk[selected[i].queryIdx].pt;
                dst[i] = new Point(rk[selected[i].trainIdx].pt.x + bounds.x, rk[selected[i].trainIdx].pt.y + bounds.y);
            }

            MatOfPoint2f srcMat = new MatOfPoint2f(src), dstMat = new MatOfPoint2f(dst);
            Mat mask = new Mat();
            Mat homography = Calib3d.findHomography(srcMat, dstMat, Calib3d.RANSAC, RANSAC_REPROJECTION, mask);
            if (homography.empty()) {
                releaseHomography(srcMat, dstMat, mask, homography);
                return null;
            }

            int inliers = Core.countNonZero(mask);
            double inlierRatio = inliers / (double) count;
            double reproj = medianReprojectionError(srcMat, dstMat, homography, mask);
            double coverageX = axisCoverage(src, mask, templateWidth, true);
            double coverageY = axisCoverage(src, mask, templateHeight, false);
            int quadrantHits = quadrantInlierCount(src, mask, templateWidth, templateHeight);

            MatOfPoint2f corners = new MatOfPoint2f(
                    new Point(0, 0), new Point(templateWidth, 0),
                    new Point(templateWidth, templateHeight), new Point(0, templateHeight));
            Core.perspectiveTransform(corners, corners, homography);
            Point[] quad = corners.toArray();
            corners.release();

            boolean geometryOk = inliers >= MIN_INLIERS
                    && inlierRatio >= MIN_INLIER_RATIO
                    && reproj <= MAX_REPROJECTION_ERROR
                    && coverageX >= MIN_TEMPLATE_COVERAGE
                    && coverageY >= MIN_TEMPLATE_COVERAGE
                    && quadrantHits >= 3
                    && quadSane(quad, frameW, frameH, templateWidth, templateHeight);

            Point center = null;
            if (geometryOk) {
                center = new Point(
                        (quad[0].x + quad[1].x + quad[2].x + quad[3].x) / 4.0,
                        (quad[0].y + quad[1].y + quad[2].y + quad[3].y) / 4.0);
            }

            releaseHomography(srcMat, dstMat, mask, homography);
            if (center == null) return null;
            return new GeometryHit(center, inlierRatio);
        }

        private void publishResult(Mat input, long nowMs, double dt, GeometryHit hit) {
            boolean geometryValid = hit != null;
            Point rawCenter = hit != null ? hit.center : null;

            if (geometryValid && rawCenter != null && filteredX != Double.NaN && dt > 0) {
                double jump = Math.hypot(rawCenter.x - filteredX, rawCenter.y - filteredY);
                if (jump > MAX_PIXELS_PER_SECOND * dt) geometryValid = false;
            }

            if (geometryValid) {
                consecutiveValid++;
                consecutiveMiss = 0;
                smoothCenter(rawCenter);
                if (detectionState == DetectionState.LOST || detectionState == DetectionState.SEARCHING) {
                    if (consecutiveValid >= ACQUIRE_FRAMES) detectionState = DetectionState.LOCKED;
                } else {
                    detectionState = DetectionState.LOCKED;
                }
            } else {
                consecutiveValid = 0;
                consecutiveMiss++;
                if (detectionState == DetectionState.LOCKED && consecutiveMiss < LOSE_MISS_FRAMES) {
                    detectionState = DetectionState.COASTING;
                } else if (consecutiveMiss >= LOSE_MISS_FRAMES) {
                    detectionState = DetectionState.LOST;
                }
            }

            double cx, cy, dx, dy;
            boolean frameValid;
            double confidence = hit != null ? hit.inlierRatio : 0;

            if (detectionState == DetectionState.LOCKED && geometryValid && filteredX != Double.NaN) {
                cx = filteredX;
                cy = filteredY;
                dx = cx - input.cols() / 2.0;
                dy = cy - input.rows() / 2.0;
                frameValid = true;
                coastingHold = buildStreamingResult(nowMs, DetectionState.LOCKED, true, true, cx, cy, dx, dy, confidence);
            } else if (detectionState == DetectionState.COASTING && coastingHold != null) {
                cx = coastingHold.centerX;
                cy = coastingHold.centerY;
                dx = coastingHold.dxPx;
                dy = coastingHold.dyPx;
                frameValid = false;
                confidence = coastingHold.confidence;
            } else {
                cx = Double.NaN;
                cy = Double.NaN;
                dx = Double.NaN;
                dy = Double.NaN;
                frameValid = false;
            }

            boolean authorized = frameValid
                    && detectionState == DetectionState.LOCKED
                    && mode == Mode.SINGLE_TARGET
                    && cameraId == CameraId.WEBCAM1
                    && processingMs <= MAX_FRAME_LATENCY_MS;

            latest.set(new Result(cameraId, mode, targetId, State.STREAMING, detectionState, nowMs,
                    frameValid, authorized, geometryValid, cx, cy, dx, dy, confidence, processingMs, fps));
        }

        private Result buildStreamingResult(long nowMs, DetectionState ds, boolean valid, boolean geom,
                                            double cx, double cy, double dx, double dy, double conf) {
            boolean authorized = valid && ds == DetectionState.LOCKED
                    && mode == Mode.SINGLE_TARGET && cameraId == CameraId.WEBCAM1
                    && processingMs <= MAX_FRAME_LATENCY_MS;
            return new Result(cameraId, mode, targetId, State.STREAMING, ds, nowMs,
                    valid, authorized, geom, cx, cy, dx, dy, conf, processingMs, fps);
        }

        private void smoothCenter(Point raw) {
            medianBuf[medianIdx] = new Point(raw.x, raw.y);
            medianIdx = (medianIdx + 1) % medianBuf.length;
            double mx = medianCoord(medianBuf, true);
            double my = medianCoord(medianBuf, false);
            if (filteredX == Double.NaN) {
                filteredX = mx;
                filteredY = my;
            } else {
                filteredX = EMA_ALPHA * mx + (1 - EMA_ALPHA) * filteredX;
                filteredY = EMA_ALPHA * my + (1 - EMA_ALPHA) * filteredY;
            }
        }

        void invalidate(State s) {
            latest.set(Result.invalid(cameraId, mode, targetId, s));
        }

        void release() {
            if (released) return;
            released = true;
            gray.release();
            roi.release();
            templateKeys.release();
            if (templateDescriptors != null) templateDescriptors.release();
            orb.clear();
        }

        private static void toGray(Mat input, Mat gray) {
            if (input.channels() == 1) input.copyTo(gray);
            else Imgproc.cvtColor(input, gray, input.channels() == 4 ? Imgproc.COLOR_RGBA2GRAY : Imgproc.COLOR_RGB2GRAY);
        }

        private static boolean mutual(DMatch forward, List<MatOfDMatch> reverse) {
            if (forward.trainIdx < 0 || forward.trainIdx >= reverse.size()) return false;
            MatOfDMatch revPair = reverse.get(forward.trainIdx);
            DMatch[] ds = revPair.toArray();
            return ds.length > 0 && ds[0].queryIdx == forward.trainIdx && ds[0].trainIdx == forward.queryIdx;
        }

        private static double medianReprojectionError(MatOfPoint2f src, MatOfPoint2f dst, Mat h, Mat mask) {
            Point[] s = src.toArray(), d = dst.toArray();
            byte[] m = new byte[mask.rows() * mask.cols()];
            mask.get(0, 0, m);
            double[] errs = new double[s.length];
            int n = 0;
            for (int i = 0; i < s.length; i++) {
                if (m[i] == 0) continue;
                MatOfPoint2f p = new MatOfPoint2f(s[i]);
                Core.perspectiveTransform(p, p, h);
                Point proj = p.toArray()[0];
                p.release();
                errs[n++] = Math.hypot(proj.x - d[i].x, proj.y - d[i].y);
            }
            if (n == 0) return Double.MAX_VALUE;
            Arrays.sort(errs, 0, n);
            return errs[n / 2];
        }

        private static double axisCoverage(Point[] src, Mat mask, int span, boolean horizontal) {
            byte[] m = new byte[mask.rows() * mask.cols()];
            mask.get(0, 0, m);
            double min = span, max = 0;
            for (int i = 0; i < src.length; i++) {
                if (m[i] == 0) continue;
                double coord = horizontal ? src[i].x : src[i].y;
                min = Math.min(min, coord);
                max = Math.max(max, coord);
            }
            if (max <= min) return 0;
            return (max - min) / span;
        }

        private static int quadrantInlierCount(Point[] src, Mat mask, int w, int h) {
            byte[] m = new byte[mask.rows() * mask.cols()];
            mask.get(0, 0, m);
            boolean[] q = new boolean[4];
            int hits = 0;
            double midX = w * 0.5, midY = h * 0.5;
            for (int i = 0; i < src.length; i++) {
                if (m[i] == 0) continue;
                int idx = (src[i].x < midX ? 0 : 1) + (src[i].y < midY ? 0 : 2);
                if (!q[idx]) {
                    q[idx] = true;
                    hits++;
                }
            }
            return hits;
        }

        private static boolean quadSane(Point[] p, int frameW, int frameH, int tplW, int tplH) {
            if (p.length != 4) return false;
            double area = quadArea(p);
            double tplArea = tplW * tplH;
            if (area < tplArea * 0.05 || area > tplArea * 4.0) return false;
            if (!isConvex(p)) return false;
            double margin = 80;
            for (Point pt : p) {
                if (pt.x < -margin || pt.y < -margin || pt.x > frameW + margin || pt.y > frameH + margin) return false;
            }
            return true;
        }

        private static double quadArea(Point[] p) {
            double sum = 0;
            for (int i = 0; i < 4; i++) {
                Point a = p[i], b = p[(i + 1) % 4];
                sum += a.x * b.y - b.x * a.y;
            }
            return Math.abs(sum) * 0.5;
        }

        private static boolean isConvex(Point[] p) {
            boolean sign = false;
            for (int i = 0; i < 4; i++) {
                Point a = p[i], b = p[(i + 1) % 4], c = p[(i + 2) % 4];
                double cross = (b.x - a.x) * (c.y - b.y) - (b.y - a.y) * (c.x - b.x);
                if (cross == 0) continue;
                boolean pos = cross > 0;
                if (!sign) sign = pos;
                else if (pos != sign) return false;
            }
            return true;
        }

        private static double medianCoord(Point[] buf, boolean x) {
            double[] v = new double[buf.length];
            int n = 0;
            for (Point p : buf) {
                if (p == null) continue;
                v[n++] = x ? p.x : p.y;
            }
            if (n == 0) return Double.NaN;
            Arrays.sort(v, 0, n);
            return v[n / 2];
        }

        private static void releaseHomography(MatOfPoint2f a, MatOfPoint2f b, Mat c, Mat d) {
            a.release();
            b.release();
            c.release();
            d.release();
        }

        private static final class GeometryHit {
            final Point center;
            final double inlierRatio;
            GeometryHit(Point center, double inlierRatio) {
                this.center = center;
                this.inlierRatio = inlierRatio;
            }
        }
    }
}
