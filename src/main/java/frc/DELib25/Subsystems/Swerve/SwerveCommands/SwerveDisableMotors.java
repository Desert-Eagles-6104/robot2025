// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems.Swerve.SwerveCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.DELib25.Subsystems.Swerve.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SwerveDisableMotors extends InstantCommand {
 SwerveSubsystem m_swerveSubsystem;
  public SwerveDisableMotors(SwerveSubsystem swerveSubsystem) {
    m_swerveSubsystem = swerveSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerveSubsystem.disableModules();
  }
}