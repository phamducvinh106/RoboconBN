package org.firstinspires.ftc.teamcode.core;

public final class EncoderLocalizerManager { private final HardwareContracts.PoseSource source; public EncoderLocalizerManager(HardwareContracts.PoseSource source){this.source=source;} public HardwareContracts.PoseReading reading(){return source.read();} public boolean valid(){return reading().valid;} }
