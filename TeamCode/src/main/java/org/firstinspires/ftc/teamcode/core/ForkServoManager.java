package org.firstinspires.ftc.teamcode.core;

public final class ForkServoManager {
    public enum Pose { PLACE, HOLD }
    private final HardwareContracts.ServoChannel left,right; private final double placeLeft,placeRight,holdLeft,holdRight; private Pose pose=Pose.PLACE;
    public ForkServoManager(HardwareContracts.ServoChannel left,HardwareContracts.ServoChannel right,double pl,double pr,double hl,double hr){this.left=left;this.right=right;placeLeft=finite(pl);placeRight=finite(pr);holdLeft=finite(hl);holdRight=finite(hr);}
    private static double finite(double v){if(!Double.isFinite(v)||v<0||v>1)throw new IllegalArgumentException("servo position");return v;}
    public void setPose(Pose p){if(p==null)throw new IllegalArgumentException("pose");left.setPosition(p==Pose.PLACE?placeLeft:holdLeft);right.setPosition(p==Pose.PLACE?placeRight:holdRight);pose=p;}
    public Pose pose(){return pose;}
}
