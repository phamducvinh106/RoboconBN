package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.features2d.DescriptorMatcher;
import org.opencv.features2d.ORB;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/** Single lifecycle owner for explicit webcam1/webcam2 ORB/template policies. */
public final class ColorContourCamera {
    public enum Mode { SINGLE_TARGET, MULTI_TARGET }
    public enum CameraId { WEBCAM1, WEBCAM2 }
    public enum State { CREATED, OPENING, STREAMING, STOPPING, CLOSED, ERROR }
    public static final int STREAM_WIDTH = 640, STREAM_HEIGHT = 480, MAX_CANDIDATES = 2;
    public static final int MIN_DESCRIPTOR_MATCHES = 8;
    public static final double MIN_MATCH_CONFIDENCE = .35, NMS_OVERLAP = .45, MIN_CENTER_DISTANCE_PX = 24;
    public static final long MAX_RESULT_AGE_MS = 300;

    public static final class ClassConfig {
        public final String label;
        public final Mat template;
        public final double minConfidence;
        public ClassConfig(String label, Mat template, double minConfidence) {
            if (label == null || label.isEmpty() || template == null || template.empty()) throw new IllegalArgumentException("template required");
            this.label = label; this.template = template; this.minConfidence = minConfidence;
        }
    }

    public static final class Candidate {
        public final String label; public final double centerX, centerY, confidence;
        Candidate(String label, double x, double y, double confidence) { this.label=label; centerX=x; centerY=y; this.confidence=confidence; }
    }

    public static final class Result {
        public final long timestampMs; public final boolean valid; public final String label;
        public final double centerX, centerY, dxPx, dyPx, confidence; public final int stableFrames, contourCount;
        public final boolean fastCentering, authorizesMovement; public final State state; public final int errorCode;
        public final List<Candidate> candidates;
        private Result(long time, boolean valid, String label, double x, double y, double dx, double dy,
                       double confidence, int stableFrames, boolean fast, boolean authorized, State state,
                       int errorCode, List<Candidate> candidates) {
            timestampMs=time; this.valid=valid; this.label=label; centerX=x; centerY=y; dxPx=dx; dyPx=dy;
            this.confidence=confidence; this.stableFrames=stableFrames; contourCount=candidates.size(); fastCentering=fast;
            authorizesMovement=authorized; this.state=state; this.errorCode=errorCode;
            this.candidates=Collections.unmodifiableList(new ArrayList<>(candidates));
        }
        static Result empty(long t, State state, int error) { return new Result(t,false,"",Double.NaN,Double.NaN,Double.NaN,Double.NaN,0,0,false,false,state,error,Collections.<Candidate>emptyList()); }
    }

    private final OpenCvWebcam webcam; private final Pipeline pipeline; private final CameraId cameraId;
    private volatile State state = State.CREATED; private volatile int errorCode; private long generation;

    public ColorContourCamera(HardwareMap map, String name, boolean preview, Mode mode) {
        this(map, name, preview, mode, new ClassConfig[0]);
    }
    public ColorContourCamera(HardwareMap map, String name, boolean preview, Mode mode, ClassConfig[] classes) {
        cameraId = "webcam1".equals(name) ? CameraId.WEBCAM1 : "webcam2".equals(name) ? CameraId.WEBCAM2 : failCameraName(name);
        if (mode == Mode.SINGLE_TARGET && cameraId != CameraId.WEBCAM1) throw new IllegalArgumentException("SINGLE_TARGET requires webcam1");
        WebcamName camera = map.get(WebcamName.class, name);
        int view = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName());
        webcam = preview ? OpenCvCameraFactory.getInstance().createWebcam(camera, view) : OpenCvCameraFactory.getInstance().createWebcam(camera);
        pipeline = new Pipeline(cameraId, mode, classes); webcam.setPipeline(pipeline); webcam.setMillisecondsPermissionTimeout(3000);
    }
    private static CameraId failCameraName(String name) { throw new IllegalArgumentException("camera must be webcam1 or webcam2: " + name); }
    public synchronized void startAsync() {
        if (state == State.OPENING || state == State.STREAMING) return;
        final long token = ++generation; state = State.OPENING;
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override public void onOpened() { synchronized (ColorContourCamera.this) { if (token != generation || state != State.OPENING) return; try { webcam.startStreaming(STREAM_WIDTH, STREAM_HEIGHT, OpenCvCameraRotation.UPRIGHT); state=State.STREAMING; } catch (RuntimeException e) { fail(token, -1); } } }
            @Override public void onError(int code) { synchronized (ColorContourCamera.this) { fail(token, code); } }
        });
    }
    private void fail(long token, int code) { if (token != generation) return; errorCode=code; state=State.ERROR; pipeline.invalidate(); }
    public synchronized void stop() { if (state == State.CLOSED || state == State.STOPPING) return; ++generation; state=State.STOPPING; pipeline.invalidate(); try { webcam.stopStreaming(); } catch (RuntimeException ignored) {} try { webcam.closeCameraDevice(); } catch (RuntimeException ignored) {} pipeline.release(); state=State.CLOSED; }
    public Result getLatestResult() { Result r=pipeline.latest.get(); return r.state == state ? r : Result.empty(System.currentTimeMillis(), state, errorCode); }
    public State getCameraState() { return state; } public int getCameraErrorCode() { return errorCode; } public double getProcessingMs() { return pipeline.processingMs; }

    static boolean fresh(Result r, long now) { return r.state == State.STREAMING && r.valid && r.authorizesMovement && now-r.timestampMs >= 0 && now-r.timestampMs <= MAX_RESULT_AGE_MS; }

    private static final class Pipeline extends OpenCvPipeline {
        final CameraId cameraId; final Mode mode; final ClassConfig[] classes; final AtomicReference<Result> latest = new AtomicReference<>();
        final ORB orb=ORB.create(); final DescriptorMatcher matcher=DescriptorMatcher.create(DescriptorMatcher.BRUTEFORCE_HAMMING); final Mat gray=new Mat();
        volatile double processingMs; private boolean released;
        Pipeline(CameraId id, Mode mode, ClassConfig[] classes) { cameraId=id; this.mode=mode; this.classes=classes.clone(); invalidate(); }
        @Override public Mat processFrame(Mat input) { if (released) return input; long start=System.nanoTime(); Imgproc.cvtColor(input, gray, Imgproc.COLOR_RGB2GRAY); MatOfKeyPoint keys=new MatOfKeyPoint(); Mat descriptors=new Mat(); orb.detectAndCompute(gray, new Mat(), keys, descriptors); List<Candidate> found=new ArrayList<>();
            for (ClassConfig c: classes) { MatOfKeyPoint tk=new MatOfKeyPoint(); Mat td=new Mat(); orb.detectAndCompute(c.template, new Mat(), tk, td); if (td.empty() || descriptors.empty()) { tk.release(); td.release(); continue; } List<MatOfDMatch> matches=new ArrayList<>(); matcher.knnMatch(td, descriptors, matches, 2); int good=0; double score=0; for (MatOfDMatch pair: matches) { org.opencv.core.DMatch[] ds=pair.toArray(); if (ds.length>1 && ds[0].distance < .75*ds[1].distance) { good++; score += 1.0/(1.0+ds[0].distance); } pair.release(); } if (good>=MIN_DESCRIPTOR_MATCHES) { double confidence=Math.min(1, score/good); if (confidence>=Math.max(MIN_MATCH_CONFIDENCE,c.minConfidence)) found.add(new Candidate(c.label,input.cols()/2.,input.rows()/2.,confidence)); } tk.release(); td.release(); }
            keys.release(); descriptors.release(); found.sort(Comparator.comparingDouble((Candidate c)->c.confidence).reversed().thenComparing(c->c.label)); List<Candidate> kept=new ArrayList<>(); for (Candidate c:found) { boolean duplicate=false; for(Candidate k:kept) if(Math.hypot(c.centerX-k.centerX,c.centerY-k.centerY)<MIN_CENTER_DISTANCE_PX) { duplicate=true; break; } if(!duplicate && kept.size()<MAX_CANDIDATES) kept.add(c); }
            long now=System.currentTimeMillis(); Candidate best=kept.isEmpty()?null:kept.get(0); boolean valid=best!=null; double x=valid?best.centerX:Double.NaN, y=valid?best.centerY:Double.NaN, dx=valid?x-input.cols()/2.:Double.NaN, dy=valid?y-input.rows()/2.:Double.NaN; boolean fast=mode==Mode.SINGLE_TARGET && cameraId==CameraId.WEBCAM1 && valid; latest.set(new Result(now,valid,valid?best.label:"",x,y,dx,dy,valid?best.confidence:0,valid?1:0,fast,fast,State.STREAMING,0,kept)); processingMs=(System.nanoTime()-start)/1e6; return input; }
        void invalidate() { latest.set(Result.empty(System.currentTimeMillis(), State.CREATED, 0)); }
        void release() { if(released)return; released=true; gray.release(); for(ClassConfig c:classes)c.template.release(); }
    }
}
