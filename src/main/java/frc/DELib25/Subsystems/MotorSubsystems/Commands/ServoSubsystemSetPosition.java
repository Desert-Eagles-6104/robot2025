// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems.MotorSubsystems.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.Subsystems.MotorSubsystems.Base.Motor.ServoSubsystemTalon;

public class ServoSubsystemSetPosition extends Command {
  private ServoSubsystemTalon m_ServoSubsystemTalon;
  private double m_Position; /*homePosition/* */
  private boolean m_MotionMagic;
  /** Creates a new ServoSubsystemHoming. */
  
  public ServoSubsystemSetPosition(ServoSubsystemTalon ServoSubsystemTalon , double Position) {
    m_ServoSubsystemTalon = ServoSubsystemTalon;
    m_Position = Position;
    m_MotionMagic = false;
    addRequirements(ServoSubsystemTalon);
  }

  public ServoSubsystemSetPosition(ServoSubsystemTalon ServoSubsystemTalon , double Position , boolean motionMagic) {
    m_ServoSubsystemTalon = ServoSubsystemTalon;
    m_Position = Position;
    m_MotionMagic = motionMagic;
    addRequirements(ServoSubsystemTalon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_MotionMagic) m_ServoSubsystemTalon.setMotionMagicPosition(m_Position);
    else m_ServoSubsystemTalon.setPosition(m_Position);
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_ServoSubsystemTalon.isAtSetpoint();
  }
}
