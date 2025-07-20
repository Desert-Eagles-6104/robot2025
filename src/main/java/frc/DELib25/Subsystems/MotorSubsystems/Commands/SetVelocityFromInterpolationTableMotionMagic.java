// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems.MotorSubsystems.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.Subsystems.MotorSubsystems.Base.Motor.VelocitySubsystemTalon;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SetVelocityFromInterpolationTableMotionMagic extends Command {
	private DoubleSupplier distanceMetersSupplier;
	private VelocitySubsystemTalon subsystemTalon;

	public SetVelocityFromInterpolationTableMotionMagic(DoubleSupplier distanceMetersSupplier,
			VelocitySubsystemTalon subsystemTalon) {
		this.distanceMetersSupplier = distanceMetersSupplier;
		this.subsystemTalon = subsystemTalon;
		addRequirements(subsystemTalon);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		this.subsystemTalon.setUsingInterpulationMotionMagic(this.distanceMetersSupplier.getAsDouble());
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		this.subsystemTalon.setUsingInterpulationMotionMagic(this.distanceMetersSupplier.getAsDouble());
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