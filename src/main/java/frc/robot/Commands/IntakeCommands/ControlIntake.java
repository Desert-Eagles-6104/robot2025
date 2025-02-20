// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ControlIntake extends Command {
  private IntakeSubsystem m_AlgaeIntake;
  private double m_output = 0;
  /** Creates a new ControlAlgaeIntake. */
  public ControlIntake(IntakeSubsystem algaeIntake , double output) {
    m_AlgaeIntake = algaeIntake;
    m_output = output;
    addRequirements(m_AlgaeIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_AlgaeIntake.setMotorPercent(m_output);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_AlgaeIntake.disableMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
