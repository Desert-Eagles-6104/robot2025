// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.DELib25.Subsystems.VelocitySubsystem.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.DELib25.Subsystems.VelocitySubsystem.Base.Motor.VelocitySubsystemTalon;

public class VelocitySubsystemSetPrecentage extends InstantCommand {
  /** Creates a new ServoSubsystemSetPrecente. */
    private VelocitySubsystemTalon m_velocitySubsystemTalon;
    private double m_PrecentOutput;
    
    public VelocitySubsystemSetPrecentage(VelocitySubsystemTalon velocitySubsystemTalon , double PrecentOutput) {
      m_velocitySubsystemTalon = velocitySubsystemTalon;
      m_PrecentOutput = PrecentOutput;
      addRequirements(velocitySubsystemTalon);
    }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_velocitySubsystemTalon.setPrecentOutput(m_PrecentOutput);
  }
}
