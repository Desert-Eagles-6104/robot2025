package frc.DELib25.Subsystems.MotorSubsystems.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemTalon;

public class MotorSubsystemSetVelocityMotionMagic extends Command {
	private MotorSubsystemTalon subsystemTalon;
	private DoubleSupplier velocityRpmSupplier;


	public MotorSubsystemSetVelocityMotionMagic(MotorSubsystemTalon subsystemTalon, DoubleSupplier velocityRpmSupplier) {
		this.subsystemTalon = subsystemTalon;
		this.velocityRpmSupplier = velocityRpmSupplier;
		addRequirements(subsystemTalon);
	}

	@Override
	public void execute() {
		this.subsystemTalon.setMotionMagicVelocity(this.velocityRpmSupplier.getAsDouble());
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
