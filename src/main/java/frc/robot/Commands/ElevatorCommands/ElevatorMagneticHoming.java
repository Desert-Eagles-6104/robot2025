// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ElevatorCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.BooleanUtil.StableBoolean;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorMagneticHoming extends Command {
  ElevatorSubsystem m_elevator;
  StableBoolean isAtResetPoint;
  private Timer m_timer;
  boolean done = false;
  boolean skipSetPosition = false;
  /** Creates a new ArmHoming. */
  public ElevatorMagneticHoming(ElevatorSubsystem Elevator) {
    m_elevator = Elevator;
    isAtResetPoint = new StableBoolean(0.2);
    addRequirements(Elevator);
    m_timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_timer.reset();
    m_elevator.ControlSoftLimit(false);
    done = false;
    if(m_elevator.getPosition()<20){
    skipSetPosition = true;
    }
    if(!skipSetPosition){
      m_elevator.setMotionMagicPosition(20);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_elevator.isAtSetpoint() || skipSetPosition){
      m_elevator.setPrecentOutput(-0.1);
    }
    if (isAtResetPoint.update(m_elevator.getMagnetState())) {
      m_timer.reset();
      if(m_timer.hasElapsed(1)){
      m_elevator.resetPosition(0);
      m_elevator.disableMotors();
      }
      else{
        m_elevator.disableMotors();
      }
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.disableMotors();
    skipSetPosition = false;
    done = false;
   // m_elevator.ControlSoftLimit(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
