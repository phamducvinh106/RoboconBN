package org.firstinspires.ftc.teamcode.test;
import org.firstinspires.ftc.teamcode.core.*;
public final class LiftingSequenceTest {
 static int passed; static void check(String n,boolean ok){if(!ok)throw new AssertionError(n);passed++;}
 static final class C implements LiftingSequenceStateMachine.Clock{long n;public long nowNs(){return n;}}
 static final class H implements LiftingSequenceStateMachine.Actuators{boolean stop;int homes;public void stop(){stop=true;}public boolean home(){return ++homes>1;}public boolean elevatorAt(LiftingSequenceConfig.ElevatorTarget t){return true;}public void setFork(LiftingSequenceConfig.ForkPose p){}public void drive(double f,double s){}public void stopDrive(){}public boolean released(){return true;}public double backOutDistanceCm(){return 20;}public LiftingSequenceStateMachine.PoseReading pose(){return new LiftingSequenceStateMachine.PoseReading(0,0,0,0);}public boolean arrival(LiftingSequenceConfig.Pose p,long n){return true;}}
 static final class Cam implements LiftingSequenceStateMachine.CameraResult{boolean valid=true;public boolean fresh(long n){return valid;}public boolean valid(){return valid;}public boolean stableLeftTarget(){return valid;}public double leftDxPx(){return 0;}}
 static void testSafety(){C c=new C();H h=new H();LiftingSequenceStateMachine m=new LiftingSequenceStateMachine(c,h);m.requestStop();m.tick();check("safe stop",m.getState()==LiftingSequenceStateMachine.State.SAFE_STOP&&h.stop);}
 static void testInvalidCamera(){C c=new C();H h=new H();Cam cam=new Cam();LiftingSequenceStateMachine m=new LiftingSequenceStateMachine(c,h,cam);for(int i=0;i<7;i++)m.tick();cam.valid=false;m.tick();check("invalid camera holds",m.getState()==LiftingSequenceStateMachine.State.SCAN_RIGHT);}
 static void testSerial(){C c=new C();H h=new H();LiftingSequenceStateMachine m=new LiftingSequenceStateMachine(c,h,new Cam());m.setIrState(true,true);for(int i=0;i<120&&m.getCompletedCycles()==0;i++){c.n+=LiftingSequenceConfig.IR_DEBOUNCE_NS;m.tick();}check("first cycle",m.getCompletedCycles()==1);}
 public static void main(String[] a){LiftingSequenceConfig.validate();testSafety();testInvalidCamera();testSerial();System.out.println("RESULT: "+passed+" passed, 0 failed");}
}
