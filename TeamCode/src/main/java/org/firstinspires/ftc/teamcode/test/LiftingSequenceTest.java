package org.firstinspires.ftc.teamcode.test;
import org.firstinspires.ftc.teamcode.core.*;
public final class LiftingSequenceTest {
 static int passed; static void check(String n,boolean ok){if(!ok)throw new AssertionError(n);passed++;}
 static final class C implements LiftingSequenceStateMachine.Clock{long n;public long nowNs(){return n;}}
 static final class H implements LiftingSequenceStateMachine.Actuators{boolean stop;int homes;LiftingSequenceConfig.ForkPose fork;public void stop(){stop=true;}public boolean home(){return ++homes>1;}public boolean elevatorAt(LiftingSequenceConfig.ElevatorTarget t){return true;}public void setFork(LiftingSequenceConfig.ForkPose p){fork=p;}public void drive(double f,double s){}public void stopDrive(){}public boolean atPose(LiftingSequenceConfig.Pose p){return true;}public boolean released(){return true;}public double backOutDistanceCm(){return 20;}}
 static final class Cam implements LiftingSequenceStateMachine.CameraResult{public boolean fresh(long n){return true;}public boolean classificationValid(){return true;}public boolean stableLeftTarget(){return true;}public double leftDxPx(){return 0;}}
 static final class P implements LiftingSequenceStateMachine.PoseProvider{public void update(){}public double x(){return 1;}public double y(){return 2;}public double headingDeg(){return 3;}}
 static void testMapping(){check("factory mapping",LiftingSequenceConfig.factory("03").placement.y==60);try{LiftingSequenceConfig.factory("bad");throw new AssertionError();}catch(IllegalArgumentException ok){passed++;}}
 static void testSerial(){C c=new C();H h=new H();LiftingSequenceStateMachine m=new LiftingSequenceStateMachine(c,h,new Cam(),new P());m.setBlockTypes("03","04");m.setIrState(true,true);for(int i=0;i<80&&m.getCompletedCycles()==0;i++){c.n+=LiftingSequenceConfig.IR_DEBOUNCE_NS;m.tick();}check("first cycle",m.getCompletedCycles()==1);check("serial ends right backout",m.getState()==LiftingSequenceStateMachine.State.HOMING);}
 static void testSafety(){C c=new C();H h=new H();LiftingSequenceStateMachine m=new LiftingSequenceStateMachine(c,h);m.requestStop();m.tick();check("safe stop",m.getState()==LiftingSequenceStateMachine.State.SAFE_STOP&&h.stop);}
 public static void main(String[] a){LiftingSequenceConfig.validate();testMapping();testSerial();testSafety();System.out.println("RESULT: "+passed+" passed, 0 failed");}
}
