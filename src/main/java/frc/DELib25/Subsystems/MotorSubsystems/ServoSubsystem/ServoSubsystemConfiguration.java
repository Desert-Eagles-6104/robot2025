// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems.MotorSubsystems.ServoSubsystem;

import frc.DELib25.Motors.PIDContainer;
import frc.DELib25.Motors.MotorConstants;
/** Add your docs here. */
public class ServoSubsystemConfiguration {
    public String subsystemName = ""; 

    public MotorConstants master = new MotorConstants(-1,"",false,false); // generic config of the master

    public MotorConstants slaves[] = null;

    public double rotationsPerPositionUnit = 1.0; // the ration between the rotations of the motor to the position of the system. i.e in an elevator it is the ration between the rotation of the motor to the height of the carriage.

    public double sensorToMechanismRatio = 1.0; 
    
    public PIDContainer pidContainerSlot0 = new PIDContainer(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    public PIDContainer pidContainerSlot1 = new PIDContainer(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    //#region motion magic values
    public double motionMagicCruiseVelocity = 0.0; // the cruise velocity in a motion magic profile
    
    public double motionMagicAcceleration = 0.0; // the acceleration in a motion magic profile

    public double motionMagicJerk = 0.0; // the jerk in a motion magic profile
    //#endregion motion magic values

    //#region cuurent limit
    public int supplyCurrentLimit = 60; 

    public boolean enableSupplyCurrentLimit = false;

    public int statorCurrentLimit = 40;

    public boolean enableStatorCurrentLimit = false;
    //#endregion current limit

    //#region soft limits 
    public double forwardSoftLimit = -99999; // the forward range limit of the mechanism.

    public double reverseSoftLimit = -99999; // the reverse range limit of the mechanism.
    //#endregion sofr limits

    public double allowableError = 6.0; // allowable error of the pid algorithm from the actual current position

    public double homePosition = 0.0;

    public double angleOffset = 0.0;
}
