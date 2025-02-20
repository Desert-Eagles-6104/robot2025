// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.BooleanUtil.TimeDelayedBoolean;
import frc.DELib25.Subsystems.ServoSubsystem.Base.Motor.ServoSubsystemTalon;

public class HomingElevator extends Command {
  /** Creates a new HomingElevator. */
  private ServoSubsystemTalon m_elevator;
  private double m_currentThreshold = 0.98; //TODO: need to be in const
  private double m_velocityThreshold = 0.1; //TODO: need to be in const
  private TimeDelayedBoolean m_condition;
  public HomingElevator(ServoSubsystemTalon elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
    m_elevator = elevator;
    m_condition = new TimeDelayedBoolean();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_elevator.setMotionMagicPosition(0.05);
    m_elevator.ControlSoftLimit(false);
    m_elevator.setPrecentOutput(-0.05);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
       //need to be homing speed in constants
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_elevator.resetPosition(0);
    m_elevator.disableMotors();
    m_elevator.ControlSoftLimit(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_condition.update(m_elevator.getMotorCurrent() > m_currentThreshold && m_elevator.getVelocity() < m_velocityThreshold, 0.2);
  }
}