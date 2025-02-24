// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.GripperCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper2Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GripperSet extends Command {
  Gripper2Subsystem m_gripper;
  double m_output;
  /** Creates a new GripperSet. */
  public GripperSet(Gripper2Subsystem gripper , double output ) {
    m_gripper = gripper;
    m_output = output;
    addRequirements(m_gripper);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gripper.setPercent(m_output);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_gripper.disableMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
