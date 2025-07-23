package frc.DELib25.Subsystems.MotorSubsystems.Commands;

import java.util.function.DoubleSupplier;

import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemTalon;

public class MotorSubsystemSetVelocityFromInterpolationTableMotionMagic extends MotorSubsystemSetVelocity {

	public MotorSubsystemSetVelocityFromInterpolationTableMotionMagic(MotorSubsystemTalon subsystemTalon, DoubleSupplier valueSupplier) {
		super(subsystemTalon, valueSupplier);
	}

	@Override
	public void execute() {
		this.subsystemTalon.setVelocityUsingInterpulationMotionMagic(this.valueSupplier.getAsDouble());
	}

}