// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems.ServoSubsystem.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.DELib25.Subsystems.ServoSubsystem.Base.Motor.ServoSubsystemTalon;

public class ServoSubsystemSetPrecentage extends InstantCommand {
  private ServoSubsystemTalon m_ServoSubsystemTalon;
  private double m_precentOutput; /*homePosition/* */
  /** Creates a new ServoSubsystemHoming. */
  
  public ServoSubsystemSetPrecentage(ServoSubsystemTalon ServoSubsystemTalon , double precentOutput) {
    m_ServoSubsystemTalon = ServoSubsystemTalon;
    m_precentOutput = precentOutput;
    addRequirements(ServoSubsystemTalon);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_ServoSubsystemTalon.setPrecentOutput(m_precentOutput);
  }
}
