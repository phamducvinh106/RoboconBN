package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.MatOfDouble;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

/** One detector for both webcams: webcam1 centers, webcam2 only classifies. */
public final class ColorContourCamera {
    public enum Mode { LEFT_CENTERING, RIGHT_CLASSIFICATION }
    public static final int STREAM_WIDTH = 640, STREAM_HEIGHT = 480;

    public static final class ClassConfig {
        public final String label;
        public final double hueMin, hueMax, saturationMin, valueMin;
        public final double aspectMin, aspectMax, circularityMin;
        public ClassConfig(String label, double hueMin, double hueMax, double saturationMin,
                           double valueMin, double aspectMin, double aspectMax, double circularityMin) {
            this.label = label; this.hueMin = hueMin; this.hueMax = hueMax;
            this.saturationMin = saturationMin; this.valueMin = valueMin;
            this.aspectMin = aspectMin; this.aspectMax = aspectMax; this.circularityMin = circularityMin;
        }
    }

    public static final int CLASS_SWITCH_FRAMES = 10;
    public static final int PATTERN_GRID_SIZE = 4;
    public static final ClassConfig[] DEFAULT_CLASSES = {
            new ClassConfig("BLOCK_1", 0, 45, 45, 35, .35, 2.85, .15),
            new ClassConfig("BLOCK_2", 45, 90, 45, 35, .35, 2.85, .15),
            new ClassConfig("BLOCK_3", 90, 135, 45, 35, .35, 2.85, .15),
            new ClassConfig("BLOCK_4", 135, 179, 45, 35, .35, 2.85, .15)
    };

    public static final class Result {
        public final long timestampMs; public final boolean valid; public final String label;
        public final double centerX, centerY, dxPx, dyPx, confidence;
        public final int contourCount, stableFrames; public final boolean fastCentering;
        private Result(long t, boolean v, String l, double x, double y, double dx, double dy, double c, int n, int s, boolean f) {
            timestampMs=t; valid=v; label=l; centerX=x; centerY=y; dxPx=dx; dyPx=dy; confidence=c; contourCount=n; stableFrames=s; fastCentering=f;
        }
        static Result empty(long t, int n) { return new Result(t,false,"",Double.NaN,Double.NaN,Double.NaN,Double.NaN,0,n,0,false); }
    }

    private final OpenCvWebcam webcam; private final Pipeline pipeline; private volatile String state="CREATED"; private volatile int errorCode;
    public ColorContourCamera(HardwareMap map, String name, boolean preview, Mode mode) { this(map,name,preview,mode,DEFAULT_CLASSES); }
    public ColorContourCamera(HardwareMap map, String name, boolean preview, Mode mode, ClassConfig[] classes) {
        WebcamName camera=map.get(WebcamName.class,name); int view=map.appContext.getResources().getIdentifier("cameraMonitorViewId","id",map.appContext.getPackageName());
        webcam=preview?OpenCvCameraFactory.getInstance().createWebcam(camera,view):OpenCvCameraFactory.getInstance().createWebcam(camera); pipeline=new Pipeline(mode,classes); webcam.setPipeline(pipeline); webcam.setMillisecondsPermissionTimeout(3000);
    }
    public void startAsync() { if ("STREAMING".equals(state)||"OPENING".equals(state)) return; state="OPENING"; webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener(){ public void onOpened(){ try { webcam.startStreaming(STREAM_WIDTH,STREAM_HEIGHT,OpenCvCameraRotation.UPRIGHT); state="STREAMING"; } catch(RuntimeException e){ state="ERROR"; errorCode=-1; }} public void onError(int code){errorCode=code;state="ERROR";} }); }
    public void stop() { state="STOPPING"; try{webcam.stopStreaming();}catch(RuntimeException ignored){} try{webcam.closeCameraDevice();}catch(RuntimeException ignored){} pipeline.release(); state="CLOSED"; }
    public Result getLatestResult(){return pipeline.latest.get();} public String getCameraState(){return state;} public int getCameraErrorCode(){return errorCode;} public double getProcessingMs(){return pipeline.processingMs;}

    static String classify(double hue, double saturation, double value, double aspect, double circularity, double patternScore, ClassConfig[] classes) {
        ClassConfig best=null; double score=0;
        for(ClassConfig c:classes) {
            boolean hueIn=c.hueMin<=hue&&hue<=c.hueMax;
            if(hueIn&&saturation>=c.saturationMin&&value>=c.valueMin&&aspect>=c.aspectMin&&aspect<=c.aspectMax&&circularity>=c.circularityMin) {
                double s=.25+.20*Math.min(1,saturation/255)+.15*Math.min(1,value/255)+.15*Math.min(1,circularity)+.25*patternScore;
                if(s>score){score=s;best=c;}
            }
        }
        return best==null?"":best.label;
    }

    static double patternScore(Mat gray, Rect rect) {
        if (rect.width < PATTERN_GRID_SIZE || rect.height < PATTERN_GRID_SIZE) return 0;
        Mat roi = gray.submat(rect);
        Mat edges = new Mat();
        Imgproc.Canny(roi, edges, 50, 150);
        MatOfDouble mean = new MatOfDouble();
        MatOfDouble std = new MatOfDouble();
        Core.meanStdDev(edges, mean, std);
        double score = Math.min(1.0, mean.get(0, 0)[0] / 96.0);
        mean.release(); std.release(); edges.release(); roi.release();
        return score;
    }
    private static final class Pipeline extends OpenCvPipeline {
        final Mode mode; final ClassConfig[] classes; final AtomicReference<Result> latest=new AtomicReference<>(Result.empty(0,0)); final Mat hsv=new Mat(), gray=new Mat(), mask=new Mat(), hierarchy=new Mat(); double processingMs; String stableLabel=""; int stableFrames; String pendingLabel=""; int pendingFrames;
        Pipeline(Mode m,ClassConfig[] c){mode=m;classes=c.clone();}
        public Mat processFrame(Mat input){ long start=System.nanoTime(); Imgproc.cvtColor(input,hsv,Imgproc.COLOR_RGB2HSV); Imgproc.cvtColor(input,gray,Imgproc.COLOR_RGB2GRAY); Core.inRange(hsv,new Scalar(0,35,35),new Scalar(179,255,255),mask); List<MatOfPoint> contours=new ArrayList<>(); Imgproc.findContours(mask,contours,hierarchy,Imgproc.RETR_EXTERNAL,Imgproc.CHAIN_APPROX_SIMPLE); MatOfPoint best=null; String label=""; double bestScore=0; for(MatOfPoint p:contours){double area=Imgproc.contourArea(p); Rect r=Imgproc.boundingRect(p); if(area<250||r.area()<=0){p.release();continue;} double perimeter=Imgproc.arcLength(new MatOfPoint2f(p.toArray()),true); double circularity=perimeter==0?0:4*Math.PI*area/(perimeter*perimeter); double aspect=(double)r.width/r.height; double hue=Core.mean(hsv.submat(r)).val[0], sat=Core.mean(hsv.submat(r)).val[1], val=Core.mean(hsv.submat(r)).val[2]; String candidate=classify(hue,sat,val,aspect,circularity,patternScore(gray,r),classes); double score=area/10000; if(!candidate.isEmpty()&&score>bestScore){if(best!=null)best.release();best=p;bestScore=Math.min(1,score);label=candidate;}else p.release();} long now=System.currentTimeMillis(); if(best==null)latest.set(Result.empty(now,contours.size())); else {Rect r=Imgproc.boundingRect(best); if (label.equals(stableLabel)) { stableFrames++; pendingLabel=""; pendingFrames=0; } else if (!label.equals(pendingLabel)) { pendingLabel=label; pendingFrames=1; } else if (++pendingFrames >= CLASS_SWITCH_FRAMES) { stableLabel=label; stableFrames=1; pendingLabel=""; pendingFrames=0; } boolean valid=bestScore>=.03; latest.set(new Result(now,valid,stableLabel,r.x+r.width/2.,r.y+r.height/2.,r.x+r.width/2.-input.cols()/2.,r.y+r.height/2.-input.rows()/2.,bestScore,contours.size(),stableFrames,mode==Mode.LEFT_CENTERING&&valid&&stableFrames>=5));best.release();} processingMs=(System.nanoTime()-start)/1e6; return input; }
        void release(){hsv.release();gray.release();mask.release();hierarchy.release();}
    }
}
