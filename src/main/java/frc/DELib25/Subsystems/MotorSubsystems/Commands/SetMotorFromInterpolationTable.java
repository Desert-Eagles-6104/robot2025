// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems.MotorSubsystems.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.Subsystems.MotorSubsystems.MotorBase.MotorSubsystemTalon;
public class SetMotorFromInterpolationTable extends Command {
	private DoubleSupplier distanceMetersSupplier;
	private MotorSubsystemTalon subsystemTalon;

	public SetMotorFromInterpolationTable(DoubleSupplier distanceMetersSupplier,MotorSubsystemTalon subsystemTalon) {
		this.distanceMetersSupplier = distanceMetersSupplier;
		this.subsystemTalon = subsystemTalon;
		addRequirements(subsystemTalon);
	}

	@Override
	public void initialize() {
		this.subsystemTalon.setUsingInterpulation(this.distanceMetersSupplier.getAsDouble());
	}

	@Override
	public void execute() {
		this.subsystemTalon.setUsingInterpulation(this.distanceMetersSupplier.getAsDouble());
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