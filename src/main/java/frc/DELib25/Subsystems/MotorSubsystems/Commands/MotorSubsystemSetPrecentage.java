package frc.DELib25.Subsystems.MotorSubsystems.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemTalon;

public class MotorSubsystemSetPrecentage extends InstantCommand {
	private MotorSubsystemTalon subsystemTalon;
	private double precentOutput;

	public MotorSubsystemSetPrecentage(MotorSubsystemTalon subsystemTalon, double precentOutput) {
		this.subsystemTalon = subsystemTalon;
		this.precentOutput = precentOutput;
		addRequirements(subsystemTalon);
	}

	@Override
	public void initialize() {
		this.subsystemTalon.setPrecentOutput(this.precentOutput);
	}
}
