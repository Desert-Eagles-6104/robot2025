// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems.ServoSubsystem;

import frc.DELib25.Motors.PIDContainer;
import frc.DELib25.Motors.MotorConstants;
import frc.DELib25.Motors.MotorType;
/** Add your docs here. */
public class ServoSubsystemConfiguration {
    public MotorType motorType = MotorType.talonFX;

    public String subsystemName = "";

    public MotorConstants master = new MotorConstants(-1,"",false,false);

    public MotorConstants slaves[] = null;

    public double rotationsPerPositionUnit = 1.0;

    public double sensorToMechanismRatio = 1.0;
    
    public PIDContainer pidContainerSlot0 = new PIDContainer(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    public PIDContainer pidContainerSlot1 = new PIDContainer(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    //#region motion magic values
    public double motionMagicCruiseVelocity = 0.0;
    
    public double motionMagicAcceleration = 0.0;

    public double motionMagicJerk = 0.0;
    //#endregion motion magic values

    //#region cuurent limit
    public int supplyCurrentLimit = 60; 

    public boolean enableSupplyCurrentLimit = false;

    public int statorCurrentLimit = 40;

    public boolean enableStatorCurrentLimit = false;
    //#endregion current limit

    //#region soft limits 
    public double forwardSoftLimit = -99999;

    public double reverseSoftLimit = -99999;
    //#endregion sofr limits

    public double allowableError = 4.0;

    public double homePosition = 0.0;
}
