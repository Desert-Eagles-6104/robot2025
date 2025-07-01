// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems.ServoSubsystem.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.DELib25.Subsystems.ServoSubsystem.Base.Motor.ServoSubsystemTalon;

public class ServoSubsystemHoming extends Command {
  private ServoSubsystemTalon m_ServoSubsystemTalon;
  private double m_resistPrecent = 0;
  private double m_currentThreshold = 0;
  private double m_velocityThreshold = 0;
  /** Creates a new ServoSubsystemHoming. 
   * @param velocityThreshold */
  
  public ServoSubsystemHoming(ServoSubsystemTalon ServoSubsystemTalon, double resistPrecent, double currentThreshold, double velocityThreshold) {
    m_ServoSubsystemTalon = ServoSubsystemTalon;
    m_currentThreshold = currentThreshold;
    m_velocityThreshold = velocityThreshold;
    addRequirements(ServoSubsystemTalon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ServoSubsystemTalon.ControlSoftLimit(false);
    m_ServoSubsystemTalon.setPosition(m_ServoSubsystemTalon.m_configuration.homePosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_ServoSubsystemTalon.isAtSetpoint()) {
      m_ServoSubsystemTalon.setPrecentOutput(m_resistPrecent);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_ServoSubsystemTalon.ControlSoftLimit(true);
    m_ServoSubsystemTalon.disableMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return m_ServoSubsystemTalon.getMotorCurrent() > m_currentThreshold && m_ServoSubsystemTalon.getVelocity() < m_velocityThreshold;
  }
}