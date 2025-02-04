// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.BooleanUtil.StableBoolean;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorHoming extends Command {
  ElevatorSubsystem m_elevator;
  StableBoolean isAtResetPoint;
  boolean done = false;
  boolean skipSetPosition = false;
  /** Creates a new ArmHoming. */
  public ElevatorHoming(ElevatorSubsystem elevator) {
    m_elevator = elevator;
    isAtResetPoint = new StableBoolean(0.2);
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_elevator.ControlSoftLimit(false);
    done = false;
    if(m_elevator.getPosition()<20){
    skipSetPosition = true;
    }
    if(!skipSetPosition){
      m_elevator.setMotionMagicPosition(20); //TODO: THIS MUST BE TUNED TO SPECIFIC ROBOT
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_elevator.isAtSetpoint() || skipSetPosition){
      m_elevator.setPrecentOutput(-0.07);
    }
    if (isAtResetPoint.get(m_elevator.getMagnetState())) {
      m_elevator.resetSubsystemToInitialState();
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.disableMotors();
    skipSetPosition = false;
    done = false;
    m_elevator.ControlSoftLimit(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
