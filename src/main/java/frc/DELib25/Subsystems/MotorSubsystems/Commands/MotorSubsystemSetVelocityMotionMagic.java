package frc.DELib25.Subsystems.MotorSubsystems.Commands;

import java.util.function.DoubleSupplier;

import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemTalon;

public class MotorSubsystemSetVelocityMotionMagic extends MotorSubsystemSetVelocity {

	public MotorSubsystemSetVelocityMotionMagic(MotorSubsystemTalon subsystemTalon, DoubleSupplier valueSupplier) {
		super(subsystemTalon, valueSupplier);
	}

	@Override
	public void execute() {
		this.subsystemTalon.setMotionMagicVelocity(this.valueSupplier.getAsDouble());
	}

}