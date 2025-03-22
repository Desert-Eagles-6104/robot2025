package frc.DELib25.Subsystems.ServoSubsystem.Base;

import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Voltage;

public interface IServoSubsystemBase {
    /**
     * Converts from real world units to sesnsor units
     * @param units - Position in real world units
     * @return Position in sensor units
     */
    public abstract double toRotations(double units);
    
    /**
     * Converts from sensor units to real world units
     * @param rotations - Position in sensor units
     * @return Position in real world units
     */
    public abstract double fromRotations(double rotations);

    /**
     * set the wanted position of mechanism to the given position using a motion magic profile.
     * @param position wanted position.
     */
    public abstract void setMotionMagicPosition(double position);

     /**
     * set the wanted position of mechanism to the given position using simple pid.
     * @param position wanted position.
     */
    public abstract void setPosition(double position);

    public abstract void setPrecentOutput(double precent);
    
    public abstract double getPosition();

    public abstract double getVelocity();

    public abstract double getMotorCurrent();

    public abstract double getClosedLoopError();

    public abstract void ControlSoftLimit(boolean enableSoftLimit);

    public abstract void resetPosition(double position);

    public abstract boolean isAtSetpoint();

    public abstract void changeNeutralMode(NeutralModeValue NeutralMode);

    public abstract void disableMotors();

     /** 
   * gets the homePosition of the subsystem from the generic Configuration
   * and resets the physical position of the system to the home position
   */
    public abstract void resetSubsystemToInitialState();

    public abstract void runCharacterization(Voltage volts);
}