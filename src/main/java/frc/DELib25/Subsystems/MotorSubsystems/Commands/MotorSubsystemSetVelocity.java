package frc.DELib25.Subsystems.MotorSubsystems.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemTalon;

public class MotorSubsystemSetVelocity extends Command {
	private DoubleSupplier velocityRpmSupplier;
	private MotorSubsystemTalon subsystemTalon;

	public MotorSubsystemSetVelocity(MotorSubsystemTalon subsystemTalon,DoubleSupplier velocityRpmSupplier) {
		this.velocityRpmSupplier = velocityRpmSupplier;
		this.subsystemTalon = subsystemTalon;
		addRequirements(subsystemTalon);
	}

	@Override
	public void initialize() {
		this.subsystemTalon.setVelocity(this.velocityRpmSupplier.getAsDouble());
	}

	@Override
	public void execute() {
		this.subsystemTalon.setVelocity(this.velocityRpmSupplier.getAsDouble());
	}

	@Override
	public void end(boolean interrupted) {
		this.subsystemTalon.disableMotors();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
