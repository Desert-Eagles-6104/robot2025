// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems.VelocitySubsystem;

import frc.DELib25.Motors.PIDContainer;
import frc.DELib25.Motors.MotorConstants;
/** Add your docs here. */
public class VelocitySubsystemConfiguration {
    //public MotorType motorType = MotorType.talonFX;

    public String subsystemName = "";

    public MotorConstants master = new MotorConstants(-1,"",false,false);

    public MotorConstants slaves[] = null;

    public double rotationsPerPositionUnit = 1.0;

    public double sensorToMechanismRatio = 1.0;
    
    public PIDContainer pidContainer = new PIDContainer(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

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

    public double allowableError = 0.0;

    public String fileLocation  = "";
}
