package frc.DELib25.Subsystems.MotorSubsystems.VelocitySubsystem.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.DELib25.Subsystems.MotorSubsystems.VelocitySubsystem.Base.Motor.VelocitySubsystemTalon;

public class VelocitySubsystemSetPrecentage extends InstantCommand {
	private VelocitySubsystemTalon subsystemTalon;
	private DoubleSupplier velocityRpmSupplier;
	
	public VelocitySubsystemSetPrecentage(VelocitySubsystemTalon subsystemTalon, DoubleSupplier velocityRpmSupplier) {
		this.subsystemTalon = subsystemTalon;
		this.velocityRpmSupplier = velocityRpmSupplier;
		addRequirements(subsystemTalon);
	}

	@Override
	public void initialize() {
		subsystemTalon.setPrecentOutput(velocityRpmSupplier.getAsDouble());
	}
}
