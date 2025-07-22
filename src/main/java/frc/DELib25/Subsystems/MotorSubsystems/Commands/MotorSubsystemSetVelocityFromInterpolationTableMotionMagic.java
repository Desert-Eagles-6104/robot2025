package frc.DELib25.Subsystems.MotorSubsystems.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemTalon;

public class MotorSubsystemSetVelocityFromInterpolationTableMotionMagic extends Command {
	private MotorSubsystemTalon subsystemTalon;
	private DoubleSupplier distanceMetersSupplier;

	public MotorSubsystemSetVelocityFromInterpolationTableMotionMagic(MotorSubsystemTalon subsystemTalon, DoubleSupplier distanceMetersSupplier) {
		this.subsystemTalon = subsystemTalon;
		this.distanceMetersSupplier = distanceMetersSupplier;
		addRequirements(subsystemTalon);
	}

	@Override
	public void initialize() {
		this.subsystemTalon.setUsingInterpulationMotionMagic(this.distanceMetersSupplier.getAsDouble());
	}

	@Override
	public void execute() {
		this.subsystemTalon.setUsingInterpulationMotionMagic(this.distanceMetersSupplier.getAsDouble());
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