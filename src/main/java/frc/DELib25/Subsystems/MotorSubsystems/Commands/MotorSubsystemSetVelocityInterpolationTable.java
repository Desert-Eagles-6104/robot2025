package frc.DELib25.Subsystems.MotorSubsystems.Commands;

import java.util.function.DoubleSupplier;

import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemTalon;

public class MotorSubsystemSetVelocityInterpolationTable extends MotorSubsystemSetVelocity {

	public MotorSubsystemSetVelocityInterpolationTable(MotorSubsystemTalon subsystemTalon, DoubleSupplier valueSupplier) {
		super(subsystemTalon, valueSupplier);
	}

	@Override
	public void execute() {
		this.subsystemTalon.setVelocityUsingInterpulation(this.valueSupplier.getAsDouble());
	}

}