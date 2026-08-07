package org.firstinspires.ftc.teamcode.core;

public final class IrSensorManager {
    private final HardwareContracts.BooleanSensor left,right; public IrSensorManager(HardwareContracts.BooleanSensor left,HardwareContracts.BooleanSensor right){this.left=left;this.right=right;}
    public boolean leftActive(){return left.active();} public boolean rightActive(){return right.active();} public boolean bothActive(){return leftActive()&&rightActive();}
}
