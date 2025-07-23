package frc.DELib25.Subsystems.MotorSubsystems.Commands;

import java.util.function.DoubleSupplier;
import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemTalon;

public class MotorSubsystemSetPrecentage extends MotorSubsystemSetVelocity {

	public MotorSubsystemSetPrecentage(MotorSubsystemTalon subsystemTalon, DoubleSupplier valueSupplier) {
		super(subsystemTalon, valueSupplier);
	}

	@Override
	public void execute() {
		this.subsystemTalon.setPrecentOutput(this.valueSupplier.getAsDouble());
	}

}