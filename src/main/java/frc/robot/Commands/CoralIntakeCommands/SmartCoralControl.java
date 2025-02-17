// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CoralIntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.subsystems.CoralIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SmartCoralControl extends Command {
  CoralIntakeSubsystem m_CoralIntakeSubsystem;
  private double m_output = 0;
  private InstantCoralControl m_coralControl;
  

  /** Creates a new Controlintake. */
  public SmartCoralControl(CoralIntakeSubsystem coralIntakeSubsystem , double output) {
    m_CoralIntakeSubsystem = coralIntakeSubsystem;
    double m_output = output;
    addRequirements(coralIntakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(CoralIntakeSubsystem.getInstance().getApprove()){
      CoralIntakeSubsystem.getInstance().setMotorPrecent(m_output);
    }
    else{
      CoralIntakeSubsystem.getInstance().disableMotors();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_CoralIntakeSubsystem.disableMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
