package frc.DELib25.Subsystems.MotorSubsystems.Commands;

import java.util.function.DoubleSupplier;

import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemTalon;

public class MotorSubsystemSetVelocity extends MotorDoubleSupplierCommand {
	protected MotorSubsystemTalon subsystemTalon;
	protected DoubleSupplier valueSupplier;

	public MotorSubsystemSetVelocity(MotorSubsystemTalon subsystemTalon,DoubleSupplier valueSupplier) {
		super(subsystemTalon, valueSupplier);
	}

	@Override
	public void execute() {
		this.subsystemTalon.setVelocity(this.valueSupplier.getAsDouble());
	}
}
