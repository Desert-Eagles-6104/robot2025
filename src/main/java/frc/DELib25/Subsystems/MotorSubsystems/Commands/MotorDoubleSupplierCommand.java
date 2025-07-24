package frc.DELib25.Subsystems.MotorSubsystems.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemTalon;

public abstract class MotorDoubleSupplierCommand extends Command{
    protected MotorSubsystemTalon subsystemTalon;
	protected DoubleSupplier valueSupplier;

	public MotorDoubleSupplierCommand(MotorSubsystemTalon subsystemTalon,DoubleSupplier valueSupplier) {
		this.valueSupplier = valueSupplier;
		this.subsystemTalon = subsystemTalon;
		addRequirements(subsystemTalon);
	}

	@Override
	abstract public void execute();

	@Override
	public void end(boolean interrupted) {
		this.subsystemTalon.disableMotors();
	}

	@Override
	public boolean isFinished() {
		return false;
	}
}
