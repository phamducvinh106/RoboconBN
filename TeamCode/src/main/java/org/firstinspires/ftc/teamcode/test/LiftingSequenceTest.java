package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.teamcode.core.LiftingSequenceConfig;
import org.firstinspires.ftc.teamcode.core.LiftingSequenceStateMachine;

public final class LiftingSequenceTest {
    static int passed;
    static final class Clock implements LiftingSequenceStateMachine.Clock { long now; public long nowNs(){return now;} }
    static final class Hardware implements LiftingSequenceStateMachine.Actuators {
        boolean stopped; int homes; LiftingSequenceConfig.ForkPose pose; LiftingSequenceStateMachine.State state;
        public void stop(){stopped=true;} public boolean home(){return ++homes>1;}
        public boolean elevatorAt(LiftingSequenceConfig.ElevatorTarget target){return true;}
        public void setFork(LiftingSequenceConfig.ForkPose p){pose=p;} public void drive(double f,double s){}
        public void stopDrive(){}
    }
    static final class Camera implements LiftingSequenceStateMachine.CameraResult {
        public boolean fresh(long n){return true;} public boolean classificationValid(){return true;}
        public boolean stableLeftTarget(){return true;} public double leftDxPx(){return 0;}
    }
    static final class Pose implements LiftingSequenceStateMachine.PoseProvider {
        public void update(){} public double x(){return 1;} public double y(){return 2;} public double headingDeg(){return 3;}
    }
    static void check(String n, boolean ok){if(!ok) throw new AssertionError(n); passed++;}
    static void testCanonicalOrderAndCycleGate(){
        Clock c=new Clock(); Hardware h=new Hardware(); LiftingSequenceStateMachine m=new LiftingSequenceStateMachine(c,h,new Camera(),new Pose());
        check("start homing", step(m,LiftingSequenceStateMachine.State.HOMING)); m.tick(); m.tick();
        check("set place", m.getState()==LiftingSequenceStateMachine.State.SET_PLACE); m.tick(); m.tick(); m.tick(); m.tick();
        check("ready1", m.getState()==LiftingSequenceStateMachine.State.READY1); m.tick();
        LiftingSequenceStateMachine.State[] pickup={LiftingSequenceStateMachine.State.SCAN_RIGHT,LiftingSequenceStateMachine.State.SCAN_LEFT,LiftingSequenceStateMachine.State.CENTER_LEFT_SLOW,LiftingSequenceStateMachine.State.APPROACH_IR_SLOW};
        for(LiftingSequenceStateMachine.State s:pickup){check(s.name(),m.getState()==s);m.tick();}
        m.setIrState(true,true); for (int i=0; i<8 && m.getShelfPose()==null; i++) { c.now+=LiftingSequenceConfig.IR_DEBOUNCE_NS; m.tick(); }
        check("pose save",m.getShelfPose()!=null); while(m.getState()!=LiftingSequenceStateMachine.State.CYCLE_COMPLETE)m.tick();
        check("cycle gate",m.getCompletedCycles()==0); m.tick(); check("first complete",m.getCompletedCycles()==1);
    }
    static boolean step(LiftingSequenceStateMachine m,LiftingSequenceStateMachine.State expected){m.tick();return m.getState()==expected;}
    static void testSafety(){Clock c=new Clock();Hardware h=new Hardware();LiftingSequenceStateMachine m=new LiftingSequenceStateMachine(c,h);m.requestStop();m.tick();check("safe stop",m.getState()==LiftingSequenceStateMachine.State.SAFE_STOP);check("stopped",h.stopped);}
    public static void main(String[] args){LiftingSequenceConfig.validate();testCanonicalOrderAndCycleGate();testSafety();System.out.println("RESULT: "+passed+" passed, 0 failed");}
}
