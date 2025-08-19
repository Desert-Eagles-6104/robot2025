package frc.DELib25.Subsystems.MotorSubsystems.MotorBase;


import frc.DELib25.Motors.MotorConstants;

public class MotorSubsystemConfiguration {

    /** The name of this subsystem. */
    public String subsystemName = "";

    /** Master motor configuration (required). */
    public MotorConstants master = null;

    /** Slave motor configurations (optional). */
    public MotorConstants[] slaves = null;

    public double allowableError = 6.0;

    public double homePosition = 0.0; // TODO destroy later

    public double angleOffset = 0.0;

}
