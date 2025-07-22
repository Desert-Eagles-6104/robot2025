package frc.DELib25.Subsystems.MotorSubsystems.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemTalon;

public class MotorSubsystemSetVelocityFromInterpolationTable extends Command {
	private MotorSubsystemTalon subsystemTalon;
	private DoubleSupplier distanceMetersSupplier;
	

	public MotorSubsystemSetVelocityFromInterpolationTable(MotorSubsystemTalon subsystemTalon,DoubleSupplier distanceMetersSupplier) {
		this.distanceMetersSupplier = distanceMetersSupplier;
		this.subsystemTalon = subsystemTalon;
		addRequirements(subsystemTalon);
	}

	@Override
	public void initialize() {
		this.subsystemTalon.setUsingInterpulation(this.distanceMetersSupplier.getAsDouble());
	}

	@Override
	public void execute() {
		this.subsystemTalon.setUsingInterpulation(this.distanceMetersSupplier.getAsDouble());
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