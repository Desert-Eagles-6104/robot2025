// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Gripper2Subsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EatUntilCoral extends Command {
  private Gripper2Subsystem m_gripper;
  private double m_output = 0.6; //TODO: constants
  
  public EatUntilCoral(Gripper2Subsystem gripper) {
    m_gripper = gripper;
    addRequirements(m_gripper);  
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!Gripper2Subsystem.HasGamePiece()){
      m_gripper.setPercent(m_output);
    }
    else{
      m_gripper.disableMotors();
    }
  }

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

