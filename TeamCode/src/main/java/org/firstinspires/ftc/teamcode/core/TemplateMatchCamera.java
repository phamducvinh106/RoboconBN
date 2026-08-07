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
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.io.InputStream;
import java.util.concurrent.atomic.AtomicReference;

/** Fast single-template matcher for fixed-camera scenes. */
public final class TemplateMatchCamera {
    public static final int STREAM_WIDTH = 640, STREAM_HEIGHT = 480;
    private final OpenCvWebcam webcam;
    private final Pipeline pipeline;
    private volatile String state = "CREATED";

    public TemplateMatchCamera(HardwareMap map, String webcamName, boolean preview, String assetName) {
        WebcamName name = map.get(WebcamName.class, webcamName);
        webcam = preview
                ? OpenCvCameraFactory.getInstance().createWebcam(name, monitorId(map))
                : OpenCvCameraFactory.getInstance().createWebcam(name);
        pipeline = new Pipeline(assetName);
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(3000);
    }

    private static int monitorId(HardwareMap map) {
        return map.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", map.appContext.getPackageName());
    }

    public void startAsync() {
        if (!"CREATED".equals(state)) return;
        state = "OPENING";
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() {
                webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                state = "STREAMING";
            }
            @Override public void onError(int errorCode) { pipeline.error = errorCode; state = "ERROR"; }
        });
    }

    public void stop() {
        if ("CLOSED".equals(state)) return;
        state = "STOPPING";
        try { webcam.stopStreaming(); } catch (RuntimeException ignored) { }
        try { webcam.closeCameraDevice(); } catch (RuntimeException ignored) { }
        pipeline.release();
        state = "CLOSED";
    }

    public Result latest() { return pipeline.latest.get(); }
    public int errorCode() { return pipeline.error; }
    public String getCameraState() { return state; }
    public boolean templateLoaded() { return pipeline.template != null && !pipeline.template.empty(); }

    public static final class Result {
        public final long timestampMs;
        public final boolean valid;
        public final double centerX, centerY, dxPx, dyPx, confidence, processingMs;
        private Result(long time, boolean valid, double x, double y, double dx, double dy,
                       double confidence, double processingMs) {
            timestampMs=time; this.valid=valid; centerX=x; centerY=y; dxPx=dx; dyPx=dy;
            this.confidence=confidence; this.processingMs=processingMs;
        }
        static Result empty(long now, double processingMs) {
            return new Result(now, false, Double.NaN, Double.NaN, Double.NaN, Double.NaN, 0, processingMs);
        }
    }

    private static final class Pipeline extends OpenCvPipeline {
        private static final double MIN_CONFIDENCE = .55;
        private static final int SCALE = 2;
        private final AtomicReference<Result> latest = new AtomicReference<>(Result.empty(0, 0));
        private final Mat gray = new Mat(), small = new Mat();
        private Mat template;
        private volatile boolean running = true;
        private volatile int error;

        Pipeline(String assetName) {
            try (InputStream in = AppUtil.getDefContext().getAssets().open(assetName)) {
                Bitmap bitmap = BitmapFactory.decodeStream(in);
                if (bitmap == null) throw new IllegalArgumentException("invalid bitmap");
                Mat rgba = new Mat();
                Mat source = new Mat();
                Utils.bitmapToMat(bitmap, rgba);
                Imgproc.cvtColor(rgba, source, Imgproc.COLOR_RGBA2GRAY);
                template = new Mat();
                Imgproc.resize(source, template, new org.opencv.core.Size(
                        Math.max(4, source.cols() / SCALE), Math.max(4, source.rows() / SCALE)));
                source.release(); rgba.release(); bitmap.recycle();
            } catch (Exception e) {
                error = -1;
            }
        }

        @Override public Mat processFrame(Mat frame) {
            if (!running || template == null || frame == null || frame.empty()) return frame;
            long start = System.nanoTime();
            Imgproc.cvtColor(frame, gray, frame.channels() == 4 ? Imgproc.COLOR_RGBA2GRAY : Imgproc.COLOR_RGB2GRAY);
            Imgproc.resize(gray, small, new org.opencv.core.Size(gray.cols() / SCALE, gray.rows() / SCALE));
            int rw = small.cols() - template.cols() + 1, rh = small.rows() - template.rows() + 1;
            if (rw <= 0 || rh <= 0) return frame;
            Mat result = new Mat();
            Imgproc.matchTemplate(small, template, result, Imgproc.TM_CCOEFF_NORMED);
            Core.MinMaxLocResult mm = Core.minMaxLoc(result);
            result.release();
            double elapsed = (System.nanoTime() - start) / 1e6;
            if (!Double.isFinite(mm.maxVal) || mm.maxVal < MIN_CONFIDENCE) {
                latest.set(Result.empty(System.currentTimeMillis(), elapsed));
                return frame;
            }
            double x = (mm.maxLoc.x + template.cols() / 2.0) * SCALE;
            double y = (mm.maxLoc.y + template.rows() / 2.0) * SCALE;
            latest.set(new Result(System.currentTimeMillis(), true, x, y,
                    x - frame.cols() / 2.0, y - frame.rows() / 2.0, mm.maxVal, elapsed));
            Imgproc.circle(frame, new Point(x, y), 5, new Scalar(0, 255, 0), 2);
            return frame;
        }

        void release() {
            if (!running) return;
            running = false; gray.release(); small.release();
            if (template != null) { template.release(); template = null; }
        }
    }
}
