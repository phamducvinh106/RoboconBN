package org.firstinspires.ftc.teamcode.core;

public final class EndstopManager { private final HardwareContracts.BooleanSensor sensor; public EndstopManager(HardwareContracts.BooleanSensor sensor){this.sensor=sensor;} public boolean active(){return sensor.active();} }
