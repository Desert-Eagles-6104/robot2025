package frc.DELib25.Subsystems.MotorSubsystems.MotorBase;

import frc.DELib25.Motors.motorConfiguration;
import frc.DELib25.Util.ProjectConstants;
import frc.DELib25.Motors.PIDContainer;
 
public class MotorSubsystemConfiguration {

    public String subsystemName = ""; 

    public motorConfiguration master = new motorConfiguration(-1,"",false,false); // generic config of the master

    public motorConfiguration slaves[] = null;

    public double rotationsPerPositionUnit = 1.0; // the ration between the rotations of the motor to the position of the system. i.e in an elevator it is the ration between the rotation of the motor to the height of the carriage.

    public double sensorToMechanismRatio = 1.0; 
    
    public PIDContainer pidContainerSlot0 = new PIDContainer(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    public PIDContainer pidContainerSlot1 = new PIDContainer(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

    public double motionMagicCruiseVelocity = 0.0; // the cruise velocity in a motion magic profile
    
    public double motionMagicAcceleration = 0.0; // the acceleration in a motion magic profile

    public double motionMagicJerk = 0.0; // the jerk in a motion magic profile

    public int supplyCurrentLimit = 60; 

    public boolean enableSupplyCurrentLimit = false;

    public int statorCurrentLimit = 40;

    public boolean enableStatorCurrentLimit = false;

    public double forwardSoftLimit = ProjectConstants.ERROR_CODE; // the forward range limit of the mechanism.

    public double reverseSoftLimit = ProjectConstants.ERROR_CODE; // the reverse range limit of the mechanism.

    public String fileLocation = "";

    public double allowableError = 6.0; // allowable error of the pid algorithm from the actual current position

    public double homePosition = 0.0;

    public double angleOffset = 0.0;
}
