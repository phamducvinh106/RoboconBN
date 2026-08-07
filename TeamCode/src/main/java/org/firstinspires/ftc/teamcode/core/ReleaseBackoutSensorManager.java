package org.firstinspires.ftc.teamcode.core;

public final class ReleaseBackoutSensorManager { private final HardwareContracts.BooleanSensor released; private final HardwareContracts.PoseSource pose; public ReleaseBackoutSensorManager(HardwareContracts.BooleanSensor released,HardwareContracts.PoseSource pose){this.released=released;this.pose=pose;} public boolean released(){return released.active();} public HardwareContracts.PoseReading reading(){return pose.read();} }
