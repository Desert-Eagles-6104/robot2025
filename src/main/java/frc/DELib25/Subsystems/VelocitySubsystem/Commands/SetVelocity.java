package frc.DELib25.Subsystems.VelocitySubsystem.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.Subsystems.VelocitySubsystem.Base.Motor.VelocitySubsystemTalon;

public class SetVelocity extends Command {
    private DoubleSupplier velocityRpmSupplier;
    private VelocitySubsystemTalon subsystemTalon;

    public SetVelocity(sub){

    }
}
