// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CoralArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.BooleanUtil.StableBoolean;
import frc.robot.subsystems.CoralArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class CoralArmHoming extends Command {
  CoralArmSubsystem m_CoralArmSubsystem;
  StableBoolean isAtResetPoint;
  boolean done = false;
  boolean skipSetPosition = false;
  /** Creates a new ArmHoming. */
  public CoralArmHoming(CoralArmSubsystem arm) {
    m_CoralArmSubsystem = arm;
    isAtResetPoint = new StableBoolean(0.2);
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_CoralArmSubsystem.ControlSoftLimit(false);
    done = false;
    if(m_CoralArmSubsystem.getPosition()<20){
    skipSetPosition = true;
    }
    if(!skipSetPosition){
      m_CoralArmSubsystem.setMotionMagicPosition(20); //TODO: THIS MUST BE TUNED TO SPECIFIC ROBOT
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_CoralArmSubsystem.isAtSetpoint() || skipSetPosition){
      m_CoralArmSubsystem.setPrecentOutput(-0.07);
    }
    if (isAtResetPoint.get(m_CoralArmSubsystem.getMagnetState())) {
      m_CoralArmSubsystem.resetSubsystemToInitialState();
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_CoralArmSubsystem.disableMotors();
    skipSetPosition = false;
    done = false;
    m_CoralArmSubsystem.ControlSoftLimit(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
