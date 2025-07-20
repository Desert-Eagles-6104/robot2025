package frc.DELib25.Subsystems.MotorSubsystems.VelocitySubsystem.Base;

import java.util.function.BooleanSupplier;
import edu.wpi.first.units.measure.Voltage;

public interface IVelocitySubsystemBase {
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

    public abstract void setMotionMagicVelocity(double velocity);

    public abstract void setVelocity(double velocity);

    public abstract void setPrecentOutput(double precent);

    public abstract void setUsingInterpulation(double value);

    public abstract void setUsingInterpulationMotionMagic(double value);

    public abstract double getInterpulationVelocity(double value);

    public abstract double getVelocity();

    public abstract double getMotorCurrent();

    public abstract void setShootingTable(String Filelocation);

    public abstract boolean isAtSetpoint();

    public abstract void disableMotors();

    public abstract void runCharacterization(Voltage volts);

    public abstract void log(BooleanSupplier startLog);
}