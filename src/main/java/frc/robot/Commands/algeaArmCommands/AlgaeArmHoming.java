// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.algeaArmCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.BooleanUtil.StableBoolean;
import frc.robot.subsystems.AlgaeArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class AlgaeArmHoming extends Command {
  AlgaeArmSubsystem m_AlgaeArmSubsystem;
  StableBoolean isAtResetPoint;
  boolean done = false;
  boolean skipSetPosition = false;
  /** Creates a new ArmHoming. */
  public AlgaeArmHoming(AlgaeArmSubsystem arm) {
    m_AlgaeArmSubsystem = arm;
    isAtResetPoint = new StableBoolean(0.2);
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_AlgaeArmSubsystem.ControlSoftLimit(false);
    done = false;
    if(m_AlgaeArmSubsystem.getPosition()<20){
    skipSetPosition = true;
    }
    if(!skipSetPosition){
      m_AlgaeArmSubsystem.setMotionMagicPosition(20); //TODO: THIS MUST BE TUNED TO SPECIFIC ROBOT
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_AlgaeArmSubsystem.isAtSetpoint() || skipSetPosition){
      m_AlgaeArmSubsystem.setPrecentOutput(-0.07);
    }
    if (isAtResetPoint.get(m_AlgaeArmSubsystem.getMagnetState())) {
      m_AlgaeArmSubsystem.resetSubsystemToInitialState();
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_AlgaeArmSubsystem.disableMotors();
    skipSetPosition = false;
    done = false;
    m_AlgaeArmSubsystem.ControlSoftLimit(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
