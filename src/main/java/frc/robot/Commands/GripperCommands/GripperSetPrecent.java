// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.GripperCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper2Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GripperSetPrecent extends Command {
  Gripper2Subsystem m_gripper2;
  private double m_output = 0;

  /** Creates a new Controlintake. */
  public GripperSetPrecent(Gripper2Subsystem coralIntakeSubsystem , double output) {
    m_gripper2 = coralIntakeSubsystem;
    double m_output = output;
    addRequirements(m_gripper2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_gripper2.setPercent(m_output);
  }
}
