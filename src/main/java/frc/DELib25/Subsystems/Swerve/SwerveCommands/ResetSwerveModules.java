// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.DELib25.Subsystems.Swerve.SwerveCommands;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.Subsystems.Swerve.SwerveSubsystem;

public class ResetSwerveModules extends Command {
  /** Creates a new ResetSwerveModules. */
  private SwerveSubsystem m_swerve;
  private boolean m_confirmOffsets = false;
  public ResetSwerveModules(SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);
    m_swerve = swerve;
    SmartDashboard.putBoolean("Confirm", false);
    SmartDashboard.putBoolean("Finish", false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_swerve.disableModules();
    m_swerve.setModulesNetrualMode(NeutralModeValue.Coast);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_confirmOffsets = SmartDashboard.getBoolean("Confirm", false);    ;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      m_swerve.setModulesNetrualMode(NeutralModeValue.Brake);
    if(m_confirmOffsets){
        // m_swerve.updateAngleOffsets();
        // m_swerve.readAngleOffsets();
    }
    m_swerve.disableModules();
    SmartDashboard.putBoolean("Confirm", false);
    SmartDashboard.putBoolean("Finish", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return SmartDashboard.getBoolean("Finish", false);
  }
}
