package org.firstinspires.ftc.teamcode.core;

public final class LiftingSequenceConfig {
    public enum ElevatorTarget { HOME(0), READY1(844), LIFT1(1781), READY2(4688), LIFT2(5625); public final int steps; ElevatorTarget(int steps){this.steps=steps;} }
    public enum ForkPose { PLACE, HOLD }
    public static final class Pose { public final double x,y,heading; public Pose(double x,double y,double heading){this.x=x;this.y=y;this.heading=heading;} }
    public static final class Factory { public final String type; public final Pose near, placement; public Factory(String type, Pose near, Pose placement){this.type=type;this.near=near;this.placement=placement;} }
    public static final double PLACE_LEFT=.25, PLACE_RIGHT=.75, HOLD_LEFT=.50, HOLD_RIGHT=.50;
    public static final long STEP_HIGH_NS=1_000_000L, STEP_LOW_NS=1_000_000L, IR_DEBOUNCE_NS=100_000_000L;
    public static final long SENSOR_STALE_NS=250_000_000L, ELEVATOR_TIMEOUT_NS=8_000_000_000L, STATE_TIMEOUT_NS=10_000_000_000L;
    public static final int MAX_RETRIES=2; public static final double RELEASE_BACK_OUT_CM=20.0, POSITION_TOLERANCE_CM=1.0, HEADING_TOLERANCE_DEG=3.0;
    private static final Factory[] FACTORIES={
        new Factory("01",new Pose(100,40,0),new Pose(120,40,0)), new Factory("02",new Pose(100,50,0),new Pose(120,50,0)),
        new Factory("03",new Pose(100,60,0),new Pose(120,60,0)), new Factory("04",new Pose(100,70,0),new Pose(120,70,0))};
    private LiftingSequenceConfig(){}
    public static Factory factory(String type){for(Factory f:FACTORIES)if(f.type.equals(type))return f;throw new IllegalArgumentException("unknown block type: "+type);}
    public static void validate(){for(Factory f:FACTORIES)for(Pose p:new Pose[]{f.near,f.placement})if(!finite(p))throw new IllegalStateException("invalid factory pose"); if(STEP_HIGH_NS<=0||STEP_LOW_NS<=0||IR_DEBOUNCE_NS<=0||STATE_TIMEOUT_NS<=0||MAX_RETRIES<0||RELEASE_BACK_OUT_CM<=0||POSITION_TOLERANCE_CM<=0||HEADING_TOLERANCE_DEG<=0)throw new IllegalStateException("non-positive lifting configuration");}
    public static boolean finite(Pose p){return p!=null&&Double.isFinite(p.x)&&Double.isFinite(p.y)&&Double.isFinite(p.heading)&&Math.abs(p.heading)<=360;}
}
