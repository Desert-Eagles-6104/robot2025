package frc.DELib25.Subsystems.Swerve.SwerveCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.DELib25.Subsystems.drive.SwerveSubsystem;

public class SwerveDisableMotors extends InstantCommand {

	SwerveSubsystem swerveSubsystem;

	public SwerveDisableMotors(SwerveSubsystem swerveSubsystem) {
		this.swerveSubsystem = swerveSubsystem;
	}

	@Override
	public void initialize() {
		this.swerveSubsystem.zeroOutputs();
	}
}