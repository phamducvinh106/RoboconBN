package org.firstinspires.ftc.teamcode.core;

public final class StepperElevatorManager {
    private final HardwareContracts.BinaryChannel step,dir; private final EndstopManager endstop; private final long highNs,lowNs; private final int maxSteps;
    private long highUntil,lowUntil; private boolean pulsing,up; private int position; private boolean known;
    public StepperElevatorManager(HardwareContracts.BinaryChannel step,HardwareContracts.BinaryChannel dir,EndstopManager endstop,long highNs,long lowNs,int maxSteps){if(step==null||dir==null||endstop==null||highNs<=0||lowNs<=0||maxSteps<1)throw new IllegalArgumentException();this.step=step;this.dir=dir;this.endstop=endstop;this.highNs=highNs;this.lowNs=lowNs;this.maxSteps=maxSteps;}
    public boolean moveToward(int target,long nowNs){if(target<0||target>maxSteps)throw new IllegalArgumentException("target out of bounds"); if(endstop.active()){step.setHigh(false);pulsing=false;position=0;known=true;if(target==0)return true;} if(target!=0&&!known)return false; int delta=target-position;if(delta==0){step.setHigh(false);return true;} boolean requestedUp=delta>0;if(pulsing&&requestedUp!=up)return false;if(nowNs<lowUntil)return false;if(pulsing){if(nowNs<highUntil)return false;step.setHigh(false);pulsing=false;lowUntil=nowNs+lowNs;if(!up&&endstop.active()){position=0;known=true;return true;}position+=up?1:-1;return position==target;} step.setHigh(false);dir.setHigh(requestedUp);up=requestedUp;step.setHigh(true);pulsing=true;highUntil=nowNs+highNs;return false;}
    public void stop(){step.setHigh(false);pulsing=false;}
    public int position(){return position;} public boolean positionKnown(){return known;} public boolean pulsing(){return pulsing;}
}
