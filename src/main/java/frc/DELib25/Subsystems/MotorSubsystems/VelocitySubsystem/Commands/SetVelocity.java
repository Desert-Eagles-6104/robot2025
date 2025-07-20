// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems.MotorSubsystems.VelocitySubsystem.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.Subsystems.MotorSubsystems.VelocitySubsystem.Base.Motor.VelocitySubsystemTalon;

public class SetVelocity extends Command {
	private DoubleSupplier velocityRpmSupplier;
	private VelocitySubsystemTalon subsystemTalon;

	public SetVelocity(DoubleSupplier velocityRpmSupplier, VelocitySubsystemTalon subsystemTalon) {
		this.velocityRpmSupplier = velocityRpmSupplier;
		this.subsystemTalon = subsystemTalon;
		addRequirements(subsystemTalon);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		this.subsystemTalon.setVelocity(this.velocityRpmSupplier.getAsDouble());
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		this.subsystemTalon.setVelocity(this.velocityRpmSupplier.getAsDouble());
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		this.subsystemTalon.disableMotors();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
