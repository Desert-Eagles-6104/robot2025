// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.IntakeCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.GripperSubsystem;

public class IntakeForTime extends Command {
  /** Creates a new IntakeForTime. */
  private GripperSubsystem m_gripper;
  private double m_power;
  private double m_timeToFinish;
  private Timer m_timer;
  private BooleanSupplier m_approve;

  public IntakeForTime(GripperSubsystem gripper, double power, double timeToFinish , BooleanSupplier Approve) {
    m_gripper = gripper;
    m_power = power;
    m_timeToFinish = timeToFinish;
    m_timer = new Timer();
    addRequirements(gripper);
    m_approve = Approve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(m_approve.getAsBoolean()){
    m_timer.reset();
    m_timer.start();
    m_gripper.setMotorPercent(m_power);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_gripper.setMotorPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_timeToFinish);
  }
}
